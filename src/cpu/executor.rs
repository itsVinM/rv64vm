use crate::cpu::csr::{
    csr_access_ok, CsrFile, CsrOp, Privilege, CSR_MEPC, CSR_MSTATUS, CSR_SATP, CSR_SEPC,
    MSTATUS_MIE, MSTATUS_MPIE, MSTATUS_MPP, MSTATUS_SIE, MSTATUS_SPIE, MSTATUS_SPP, SATP_MODE_BARE,
    SATP_MODE_SV39, SATP_PPN,
};
use crate::cpu::decoder::{AmoOp, BOp, COp, Inst, IOp, LOp, ROp, SOp};
use crate::cpu::Cpu;
use crate::Bus;
use crate::traps::TrapCause;

fn sext32(x: u32) -> u64 {
    (x as i32) as i64 as u64
}
// Signed div/rem honoring RISC-V edge cases: X / 0 -> -1, MIN / -1 -> MIN,
// X rem 0 -> X, MIN rem -1 -> 0.
fn div64(a: u64, b: u64) -> u64 {
    let (a, b) = (a as i64, b as i64);
    if b == 0 {
        u64::MAX
    } else if a == i64::MIN && b == -1 {
        i64::MIN as u64
    } else {
        (a / b) as u64
    }
}
fn rem64(a: u64, b: u64) -> u64 {
    let (a, b) = (a as i64, b as i64);
    if b == 0 {
        a as u64
    } else if a == i64::MIN && b == -1 {
        0
    } else {
        (a % b) as u64
    }
}
fn div32(a: u64, b: u64) -> u64 {
    let (a, b) = (a as i32, b as i32);
    if b == 0 {
        u64::MAX
    } else if a == i32::MIN && b == -1 {
        a as i64 as u64
    } else {
        (a / b) as i64 as u64
    }
}
fn rem32(a: u64, b: u64) -> u64 {
    let (a, b) = (a as i32, b as i32);
    if b == 0 {
        a as i64 as u64
    } else if a == i32::MIN && b == -1 {
        0
    } else {
        (a % b) as i64 as u64
    }
}

/// Sign-extend `v` from `w` bits (8/16/32; 64 is a no-op).
#[inline(always)]
fn sext(v: u64, w: u64) -> u64 {
    ((v << (64 - w)) as i64 >> (64 - w)) as u64
}


pub struct ExecResult {
    pub next_pc: u64,
    pub trap: Option<TrapCause>,
    pub halt: bool,
    pub new_priv: Option<Privilege>,
}

fn satp_ppn(satp: u64) -> u64 {
    satp & SATP_PPN
}
fn satp_mode(satp: u64) -> u64 {
    (satp >> 60) & 0xF
}
fn pte_ppn(pte: u64) -> u64 {
    (pte >> 10) & 0x00FF_FFFF_FFFF
}

fn csr_satp(csr: &CsrFile) -> u64 {
    csr.read(CSR_SATP, Privilege::M)
}
fn csr_mstatus(csr: &CsrFile) -> u64 {
    csr.mstatus()
}

const PTE_R: u64 = 1 << 1;
const PTE_W: u64 = 1 << 2;
const PTE_X: u64 = 1 << 3;
const PTE_U: u64 = 1 << 4;
const _PTE_A: u64 = 1 << 6;
const _PTE_D: u64 = 1 << 7;

fn translate<B: Bus>(
    va: u64,
    priv_level: Privilege,
    bus: &B,
    satp: u64,
    mstatus: u64,
    access_type: AccessType,
) -> Result<u64, TrapCause> {
    if priv_level == Privilege::M {
        return Ok(va);
    }
    if satp_mode(satp) == SATP_MODE_BARE {
        return Ok(va);
    }
    if satp_mode(satp) != SATP_MODE_SV39 {
        return Err(TrapCause::IllegalInstruction(0));
    }

    let sum = (mstatus >> 18) & 1;
    let mxr = (mstatus >> 19) & 1;

    let vpn = |level: u64| -> u64 { (va >> (12 + level * 9)) & 0x1FF };
    let root_ppn = satp_ppn(satp);

    let mut pte_addr = (root_ppn << 12) + (vpn(2) << 3);
    let mut pte = bus
        .read_phys64(pte_addr)
        .map_err(|_| page_fault(access_type))?;

    if pte & 1 == 0 {
        return Err(page_fault(access_type));
    }

    // Level at which the leaf PTE was found (2 = gigapage, 1 = megapage,
    // 0 = 4 KiB page). Needed to merge VA index bits into the final PA.
    let mut leaf_level = 2u64;
    for level in (0..=1).rev() {
        if pte & (PTE_R | PTE_W | PTE_X) != 0 {
            break;
        }
        let ppn = pte_ppn(pte);
        pte_addr = (ppn << 12) + (vpn(level) << 3);
        pte = bus
            .read_phys64(pte_addr)
            .map_err(|_| page_fault(access_type))?;
        if pte & 1 == 0 {
            return Err(page_fault(access_type));
        }
        leaf_level = level;
    }

    if pte & (PTE_R | PTE_W | PTE_X) == 0 {
        return Err(page_fault(access_type));
    }

    // Permission check with SUM and MXR
    let user_page = (pte & PTE_U) != 0;
    let readable = pte & (PTE_R | PTE_X) != 0 || (mxr != 0 && pte & PTE_X != 0);
    let writable = pte & (PTE_R | PTE_W) == (PTE_R | PTE_W);

    if priv_level == Privilege::U && !user_page {
        return Err(page_fault(access_type));
    }
    if priv_level == Privilege::S && user_page && sum == 0 {
        return Err(page_fault(access_type));
    }

    match access_type {
        AccessType::Fetch if pte & PTE_X == 0 => return Err(page_fault(access_type)),
        AccessType::Read if !readable => return Err(page_fault(access_type)),
        AccessType::Write if !writable => return Err(page_fault(access_type)),
        _ => {}
    }

    let ppn = pte_ppn(pte);
    let va_low_bits = 12 + leaf_level * 9;
    let pa = ((ppn << 12) & !((1u64 << va_low_bits) - 1)) | (va & ((1u64 << va_low_bits) - 1));

    Ok(pa)
}

#[derive(Clone, Copy, PartialEq, Eq)]
pub(crate) enum AccessType {
    Fetch,
    Read,
    Write,
}

/// Map an access kind to the page-fault trap it must raise.
fn page_fault(access: AccessType) -> TrapCause {
    match access {
        AccessType::Fetch => TrapCause::InstructionPageFault,
        AccessType::Read => TrapCause::LoadPageFault,
        AccessType::Write => TrapCause::StorePageFault,
    }
}

/// Physical address of the next instruction fetch, raising the fetch fault on
/// a failed walk. Reuses the *same* page walker as data accesses — mod.rs used
/// to carry a divergent `translate_fetch()` copy that missed SUM/MXR handling.
pub(crate) fn fetch_phys<B: Bus>(cpu: &Cpu, bus: &B) -> Result<u64, TrapCause> {
    let csr = &cpu.csr;
    translate(
        cpu.pc,
        cpu.priv_level,
        bus,
        csr_satp(csr),
        csr_mstatus(csr),
        AccessType::Fetch,
    )
}

/// Translate a vaddr, then run `op` on the resulting physical address.
///
/// Single funnel for every load / store / AMO:
///  • translate() raises LoadPageFault / StorePageFault from the page walk,
///  • the bus-level op raises LoadAccessFault / StoreAccessFault on physical
///    access failure.
/// Nothing silently aliases VA→PA anymore (the old `translate(..).unwrap_or(addr)`).
fn mem_op<T, B: Bus, F>(
    addr: u64,
    priv_level: Privilege,
    csr: &CsrFile,
    bus: &mut B,
    access: AccessType,
    op: F,
) -> Result<T, TrapCause>
where
    F: FnOnce(&mut B, u64) -> Result<T, TrapCause>,
{
    let pa = translate(addr, priv_level, bus, csr_satp(csr), csr_mstatus(csr), access)?;
    op(bus, pa)
}

/// Execute one decoded instruction against CPU state and the memory bus.
///
/// `cpu` and `bus` are *independent* mutable borrows (the borrow checker keeps
/// the VM state apart from RAM), so this stays sound under `no_std` and never
/// needs interior mutability.
pub fn execute<B: Bus>(cpu: &mut Cpu, bus: &mut B, inst: Inst) -> ExecResult {
    let pc = cpu.pc;
    let priv_level = cpu.priv_level;
    let regs = &mut cpu.regs;
    let csr = &mut cpu.csr;
    let mut next_pc = pc.wrapping_add(4);
    let mut trap: Option<TrapCause> = None;
    let mut halt = false;
    let mut new_priv: Option<Privilege> = None;

    macro_rules! reg {
        ($r:expr) => {
            regs[$r as usize]
        };
    }
    macro_rules! set {
        ($r:expr, $v:expr) => {
            if $r != 0 {
                regs[$r as usize] = $v;
            }
        };
    }

    match inst {
        Inst::Lui { rd, imm } => set!(rd, imm as u64),
        Inst::Auipc { rd, imm } => set!(rd, pc.wrapping_add(imm as u64)),

        Inst::Jal { rd, imm } => {
            set!(rd, next_pc);
            next_pc = pc.wrapping_add(imm as u64);
        }
        Inst::Jalr { rd, rs1, imm } => {
            let t = next_pc;
            next_pc = (reg!(rs1).wrapping_add(imm as u64)) & !1;
            set!(rd, t);
        }

        Inst::Branch { op, rs1, rs2, imm } => {
            if match op {
                BOp::Beq => reg!(rs1) == reg!(rs2),
                BOp::Bne => reg!(rs1) != reg!(rs2),
                BOp::Blt => (reg!(rs1) as i64) < (reg!(rs2) as i64),
                BOp::Bge => (reg!(rs1) as i64) >= (reg!(rs2) as i64),
                BOp::Bltu => reg!(rs1) < reg!(rs2),
                BOp::Bgeu => reg!(rs1) >= reg!(rs2),
            } {
                next_pc = pc.wrapping_add(imm as u64);
            }
        }

        Inst::Load { op, rd, rs1, imm } => {
            let addr = reg!(rs1).wrapping_add(imm as u64);
            let (w, se) = match op {
                LOp::Lb => (8, true),
                LOp::Lh => (16, true),
                LOp::Lw => (32, true),
                LOp::Ld => (64, false),
                LOp::Lbu => (8, false),
                LOp::Lhu => (16, false),
                LOp::Lwu => (32, false),
            };
            match mem_op(addr, priv_level, csr, bus, AccessType::Read, |b, pa| match w {
                8 => b.read8(pa).map(|v| v as u64),
                16 => b.read16(pa).map(|v| v as u64),
                32 => b.read32(pa).map(|v| v as u64),
                _ => b.read64(pa).map(|v| v as u64),
            }) {
                Ok(v) => set!(rd, if se { sext(v, w) } else { v }),
                Err(e) => trap = Some(e),
            }
        }

        Inst::Store { op, rs1, rs2, imm } => {
            let addr = reg!(rs1).wrapping_add(imm as u64);
            let w = match op {
                SOp::Sb => 8,
                SOp::Sh => 16,
                SOp::Sw => 32,
                SOp::Sd => 64,
            };
            let v = reg!(rs2);
            if let Err(e) = mem_op(addr, priv_level, csr, bus, AccessType::Write, |b, pa| match w {
                8 => b.write8(pa, v as u8),
                16 => b.write16(pa, v as u16),
                32 => b.write32(pa, v as u32),
                _ => b.write64(pa, v),
            }) {
                trap = Some(e);
            }
        }

        Inst::IAlu { op, rd, rs1, imm } => {
            let x = reg!(rs1);
            let u = imm as u64;
            let v = match op {
                IOp::Addi => x.wrapping_add(u),
                IOp::Slti => ((x as i64) < imm) as u64,
                IOp::Sltiu => (x < u) as u64,
                IOp::Xori => x ^ u,
                IOp::Ori => x | u,
                IOp::Andi => x & u,
                IOp::Slli => x << u,
                IOp::Srli => x >> u,
                IOp::Srai => ((x as i64) >> u) as u64,
                IOp::Addiw => sext32(x.wrapping_add(u) as u32),
                IOp::Slliw => sext32((x as u32) << u),
                IOp::Srliw => sext32((x as u32) >> u),
                IOp::Sraiw => ((x as i32) >> u) as i64 as u64,
            };
            set!(rd, v);
        }

        Inst::RR { op, rd, rs1, rs2 } => {
            let a = reg!(rs1);
            let b = reg!(rs2);
            let sh = b & 63;
            let shw = b & 31;
            let v = match op {
                ROp::Add => a.wrapping_add(b),
                ROp::Sub => a.wrapping_sub(b),
                ROp::Sll => a << sh,
                ROp::Slt => ((a as i64) < (b as i64)) as u64,
                ROp::Sltu => (a < b) as u64,
                ROp::Xor => a ^ b,
                ROp::Srl => a >> sh,
                ROp::Sra => ((a as i64) >> sh) as u64,
                ROp::Or => a | b,
                ROp::And => a & b,
                ROp::Addw => sext32(a.wrapping_add(b) as u32),
                ROp::Subw => sext32(a.wrapping_sub(b) as u32),
                ROp::Sllw => sext32((a as u32) << shw),
                ROp::Srlw => sext32((a as u32) >> shw),
                ROp::Sraw => ((a as i32) >> shw) as i64 as u64,
                ROp::Mul => a.wrapping_mul(b),
                ROp::Mulh => ((a as i64 as i128 * b as i64 as i128) >> 64) as u64,
                ROp::Mulhsu => ((a as i64 as i128 * b as u128 as i128) >> 64) as u64,
                ROp::Mulhu => ((a as u128 * b as u128) >> 64) as u64,
                ROp::Div => div64(a, b),
                ROp::Divu => a.checked_div(b).unwrap_or(u64::MAX),
                ROp::Rem => rem64(a, b),
                ROp::Remu => {
                    if b == 0 {
                        a
                    } else {
                        a % b
                    }
                },
                ROp::Mulw => sext32(a.wrapping_mul(b) as u32),
                ROp::Divw => div32(a, b),
                ROp::Divuw => sext32((a as u32).checked_div(b as u32).unwrap_or(u32::MAX)),
                ROp::Remw => rem32(a, b),
                ROp::Remuw => {
                    let b32 = b as u32;
                    sext32(if b32 == 0 { a as u32 } else { a as u32 % b32 })
                }
            };
            set!(rd, v);
        }

        Inst::Amo { op, rd, rs1, rs2, .. } => {
            let addr = reg!(rs1);
            let access = match op {
                AmoOp::LrW | AmoOp::LrD => AccessType::Read,
                _ => AccessType::Write,
            };
            let r = mem_op(addr, priv_level, csr, bus, access, |b, pa| match op {
                AmoOp::AmoaddW => {
                    let old = b.read32(pa)?;
                    b.write32(pa, old.wrapping_add(reg!(rs2) as u32))?;
                    Ok(old as i32 as i64 as u64)
                }
                AmoOp::AmoswapW => {
                    let old = b.read32(pa)?;
                    b.write32(pa, reg!(rs2) as u32)?;
                    Ok(old as i32 as i64 as u64)
                }
                AmoOp::LrW => Ok(b.read32(pa)? as i32 as i64 as u64),
                AmoOp::ScW => {
                    b.write32(pa, reg!(rs2) as u32)?;
                    Ok(0)
                }
                AmoOp::LrD => b.read64(pa),
                AmoOp::ScD => {
                    b.write64(pa, reg!(rs2))?;
                    Ok(0)
                }
            });
            match r {
                Ok(v) => set!(rd, v),
                Err(e) => trap = Some(e),
            }
        }

        Inst::Csr { op, rd, src, csr: addr } => {
            if !csr_access_ok(addr as usize, priv_level) {
                trap = Some(TrapCause::IllegalInstruction(addr as u32));
            } else {
                let (cop, zimm) = match op {
                    COp::Csrrw => (CsrOp::Write, reg!(src)),
                    COp::Csrrs => (CsrOp::Set, reg!(src)),
                    COp::Csrrc => (CsrOp::Clear, reg!(src)),
                    COp::Csrrwi => (CsrOp::Write, src as u64),
                    COp::Csrrsi => (CsrOp::Set, src as u64),
                    COp::Csrrci => (CsrOp::Clear, src as u64),
                };
                let v = csr.read_write(addr as usize, zimm, cop, priv_level);
                set!(rd, v);
            }
        }

        Inst::Ecall => {
            let cause = match priv_level {
                Privilege::U => TrapCause::EcallFromU,
                Privilege::S => TrapCause::EcallFromS,
                Privilege::M => TrapCause::EcallFromM,
            };
            trap = Some(cause);
        }
        Inst::Ebreak => {
            halt = true;
        }
        Inst::Fence => {}

        Inst::Mret => {
            if priv_level != Privilege::M {
                trap = Some(TrapCause::IllegalInstruction(0x30200073));
            } else {
                next_pc = csr.read(CSR_MEPC, Privilege::M);
                let ms = csr.mstatus();
                let mpp = (ms >> 11) & 0b11;
                let mpie = (ms >> 7) & 1;
                let mut new_ms = ms & !(MSTATUS_MIE | MSTATUS_MPP | MSTATUS_MPIE);
                new_ms |= mpie << 3; // MIE = MPIE
                new_ms |= 1 << 7; // MPIE = 1
                new_ms |= mpp << 11; // preserve MPP for priv update in step()
                csr.write(CSR_MSTATUS, new_ms, Privilege::M);
                new_priv = Some(match mpp {
                    1 => Privilege::S,
                    0 => Privilege::U,
                    _ => Privilege::M,
                });
            }
        }

        Inst::Sret => {
            let spp = (csr.mstatus() >> 8) & 1;
            let spie = (csr.mstatus() >> 5) & 1;
            let mut new_ms = csr.mstatus();
            new_ms &= !(MSTATUS_SIE | MSTATUS_SPP | MSTATUS_SPIE);
            new_ms |= spie << 1; // SIE = SPIE
            new_ms |= 1 << 5; // SPIE = 1
            new_ms |= spp << 8; // preserve SPP for priv update
            if priv_level < Privilege::S {
                trap = Some(TrapCause::IllegalInstruction(0x10200073));
            } else {
                next_pc = csr.read(CSR_SEPC, priv_level);
                csr.write(CSR_MSTATUS, new_ms, Privilege::M);
                new_priv = Some(if spp == 1 { Privilege::S } else { Privilege::U });
            }
        }

        Inst::Illegal(raw) => trap = Some(TrapCause::IllegalInstruction(raw)),
    }


    ExecResult {
        next_pc,
        trap,
        halt,
        new_priv,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::cpu::csr::{Privilege, CSR_MSTATUS};
    use crate::cpu::decoder::{BOp, COp, Inst, IOp, ROp};
    use crate::mmu::Mmu;

    /// Run one inst against a default CSR file; return final regs + result.
    fn rd(inst: Inst, init: impl FnOnce(&mut [u64; 32])) -> ([u64; 32], ExecResult) {
        let mut regs = [0; 32];
        init(&mut regs);
        let mut csr = CsrFile::new();
        let res = run(inst, &mut regs, &mut csr);
        (regs, res)
    }

    fn run(inst: Inst, regs: &mut [u64; 32], csr: &mut CsrFile) -> ExecResult {
        let mut bus = Mmu::new(&[]);
        let mut cpu = Cpu::new(0);
        cpu.pc = 0x1000;
        core::mem::swap(&mut cpu.regs, regs);
        core::mem::swap(&mut cpu.csr, csr);
        let res = execute(&mut cpu, &mut bus, inst);
        core::mem::swap(&mut cpu.regs, regs);
        core::mem::swap(&mut cpu.csr, csr);
        res
    }

    #[test]
    fn alu_imm() {
        let (regs, _) = rd(Inst::IAlu { op: IOp::Addi, rd: 2, rs1: 1, imm: 3 }, |r| r[1] = 5);
        assert_eq!(regs[2], 8);
        let (regs, _) = rd(Inst::IAlu { op: IOp::Addi, rd: 2, rs1: 1, imm: -3 }, |r| r[1] = 5);
        assert_eq!(regs[2], 2);
        let (regs, _) = rd(Inst::IAlu { op: IOp::Addi, rd: 0, rs1: 1, imm: 42 }, |r| r[1] = 5);
        assert_eq!(regs[0], 0);
        let (regs, _) = rd(Inst::IAlu { op: IOp::Slti, rd: 2, rs1: 1, imm: 10 }, |r| r[1] = 5);
        assert_eq!(regs[2], 1);
        let (regs, _) = rd(Inst::IAlu { op: IOp::Slti, rd: 2, rs1: 1, imm: 3 }, |r| r[1] = 5);
        assert_eq!(regs[2], 0);
    }

    #[test]
    fn alu_reg() {
        let (regs, _) = rd(Inst::RR { op: ROp::Add, rd: 3, rs1: 1, rs2: 2 }, |r| { r[1] = 10; r[2] = 20; });
        assert_eq!(regs[3], 30);
        let (regs, _) = rd(Inst::RR { op: ROp::Sub, rd: 3, rs1: 1, rs2: 2 }, |r| { r[1] = 20; r[2] = 10; });
        assert_eq!(regs[3], 10);
        let (regs, _) = rd(Inst::RR { op: ROp::Xor, rd: 3, rs1: 1, rs2: 2 }, |r| { r[1] = 0xFF00; r[2] = 0x0FF0; });
        assert_eq!(regs[3], 0xF0F0);
        let (regs, _) = rd(Inst::RR { op: ROp::Or, rd: 3, rs1: 1, rs2: 2 }, |r| { r[1] = 0xFF00; r[2] = 0x0FF0; });
        assert_eq!(regs[3], 0xFFF0);
        let (regs, _) = rd(Inst::RR { op: ROp::And, rd: 3, rs1: 1, rs2: 2 }, |r| { r[1] = 0xFF00; r[2] = 0x0FF0; });
        assert_eq!(regs[3], 0x0F00);
    }

    #[test]
    fn shifts() {
        let (regs, _) = rd(Inst::IAlu { op: IOp::Slli, rd: 2, rs1: 1, imm: 3 }, |r| r[1] = 5);
        assert_eq!(regs[2], 40);
        let (regs, _) = rd(Inst::IAlu { op: IOp::Srli, rd: 2, rs1: 1, imm: 4 }, |r| r[1] = 0x100);
        assert_eq!(regs[2], 0x10);
        let (regs, _) = rd(Inst::IAlu { op: IOp::Srai, rd: 2, rs1: 1, imm: 4 }, |r| r[1] = -256i64 as u64);
        assert_eq!(regs[2] as i64, -16);
    }

    #[test]
    fn mul_div() {
        let (regs, _) = rd(Inst::RR { op: ROp::Mul, rd: 3, rs1: 1, rs2: 2 }, |r| { r[1] = 1000; r[2] = 2000; });
        assert_eq!(regs[3], 2_000_000);
        let (regs, _) = rd(Inst::RR { op: ROp::Mulh, rd: 3, rs1: 1, rs2: 2 }, |r| {
            r[1] = 0xABCD_EF01_2345_6789u64 as i64 as u64;
            r[2] = 0x1234_5678_9ABC_DEF0u64 as i64 as u64;
        });
        let a = regs[1] as i64 as i128;
        let b = regs[2] as i64 as i128;
        assert_eq!(regs[3], ((a * b) >> 64) as u64);
        let (regs, _) = rd(Inst::RR { op: ROp::Divu, rd: 3, rs1: 1, rs2: 2 }, |r| { r[1] = 100; r[2] = 7; });
        assert_eq!(regs[3], 14);
        let (regs, _) = rd(Inst::RR { op: ROp::Div, rd: 3, rs1: 1, rs2: 2 }, |r| { r[1] = 100; r[2] = 0; });
        assert_eq!(regs[3], u64::MAX);
        let (regs, _) = rd(Inst::RR { op: ROp::Rem, rd: 3, rs1: 1, rs2: 2 }, |r| { r[1] = 100; r[2] = 0; });
        assert_eq!(regs[3], 100);
        let (regs, _) = rd(Inst::RR { op: ROp::Div, rd: 3, rs1: 1, rs2: 2 }, |r| { r[1] = i64::MIN as u64; r[2] = -1i64 as u64; });
        assert_eq!(regs[3], i64::MIN as u64);
    }

    #[test]
    fn jumps() {
        let (regs, res) = rd(Inst::Jal { rd: 1, imm: 0x100 }, |_| {});
        assert_eq!(regs[1], 0x1004);
        assert_eq!(res.next_pc, 0x1100);
        let (regs, res) = rd(Inst::Jalr { rd: 1, rs1: 2, imm: 0x100 }, |r| r[2] = 0x2000);
        assert_eq!(regs[1], 0x1004);
        assert_eq!(res.next_pc, 0x2100);
    }

    #[test]
    fn branches() {
        let (_, res) = rd(Inst::Branch { op: BOp::Beq, rs1: 1, rs2: 2, imm: 0x20 }, |r| { r[1] = 42; r[2] = 42; });
        assert_eq!(res.next_pc, 0x1020);
        let (_, res) = rd(Inst::Branch { op: BOp::Beq, rs1: 1, rs2: 2, imm: 0x20 }, |r| { r[1] = 42; r[2] = 43; });
        assert_eq!(res.next_pc, 0x1004);
        let (_, res) = rd(Inst::Branch { op: BOp::Blt, rs1: 1, rs2: 2, imm: 0x30 }, |r| { r[1] = 5; r[2] = 10; });
        assert_eq!(res.next_pc, 0x1030);
        let (_, res) = rd(Inst::Branch { op: BOp::Bge, rs1: 1, rs2: 2, imm: 0x30 }, |r| { r[1] = 10; r[2] = 5; });
        assert_eq!(res.next_pc, 0x1030);
        let (_, res) = rd(Inst::Branch { op: BOp::Bltu, rs1: 1, rs2: 2, imm: 0x40 }, |r| { r[1] = 5; r[2] = 10; });
        assert_eq!(res.next_pc, 0x1040);
    }

    #[test]
    fn lui_auipc() {
        let (regs, _) = rd(Inst::Lui { rd: 1, imm: 0x12345000 }, |_| {});
        assert_eq!(regs[1], 0x12345000);
        let (regs, _) = rd(Inst::Auipc { rd: 1, imm: 0x1000 }, |_| {});
        assert_eq!(regs[1], 0x2000);
    }

    #[test]
    fn word_ops() {
        let (regs, _) = rd(Inst::IAlu { op: IOp::Addiw, rd: 2, rs1: 1, imm: 1 }, |r| r[1] = 0xFFFF_FFFFu64);
        assert_eq!(regs[2], 0);
        let (regs, _) = rd(Inst::RR { op: ROp::Addw, rd: 3, rs1: 1, rs2: 2 }, |r| { r[1] = 0xFFFF_FFFFu64; r[2] = 2; });
        assert_eq!(regs[3], 1);
        let (regs, _) = rd(Inst::RR { op: ROp::Mulw, rd: 3, rs1: 1, rs2: 2 }, |r| { r[1] = 100_000; r[2] = 200_000; });
        assert_eq!(regs[3], ((100_000u64 * 200_000) as u32) as i32 as i64 as u64);
    }

    #[test]
    fn csr_rw() {
        let mut regs = [0; 32];
        regs[10] = 0xABCD;
        let mut csr = CsrFile::new();
        run(Inst::Csr { op: COp::Csrrw, rd: 1, src: 10, csr: 0x300 }, &mut regs, &mut csr);
        assert_eq!(regs[1], 0);
        assert_eq!(csr.read(CSR_MSTATUS, Privilege::M), 0xABCD);

        let mut regs = [0; 32];
        regs[10] = 0x80;
        let mut csr = CsrFile::new();
        run(Inst::Csr { op: COp::Csrrs, rd: 1, src: 10, csr: 0x300 }, &mut regs, &mut csr);
        assert_eq!(regs[1], 0);
        assert!(csr.read(CSR_MSTATUS, Privilege::M) & (1 << 7) != 0);
    }

    #[test]
    fn system() {
        let (_, res) = rd(Inst::Ecall, |_| {});
        assert_eq!(res.trap, Some(TrapCause::EcallFromM));
        let (_, res) = rd(Inst::Ebreak, |_| {});
        assert!(res.halt);
        let (_, res) = rd(Inst::Fence, |_| {});
        assert_eq!(res.next_pc, 0x1004);
        assert!(res.trap.is_none());
        let (_, res) = rd(Inst::Illegal(0xFFFFFFFF), |_| {});
        assert_eq!(res.trap, Some(TrapCause::IllegalInstruction(0xFFFFFFFF)));

        let mut regs = [0; 32];
        let mut csr = CsrFile::new();
        csr.write(CSR_MEPC, 0x2000, Privilege::M);
        csr.write(CSR_MSTATUS, (3 << 11) | (1 << 7), Privilege::M);
        let res = run(Inst::Mret, &mut regs, &mut csr);
        assert_eq!(res.next_pc, 0x2000);

        let mut cpu = Cpu::new(0);
        cpu.priv_level = Privilege::U;
        let mut bus = Mmu::new(&[]);
        let res = execute(&mut cpu, &mut bus, Inst::Sret);
        assert_eq!(res.trap, Some(TrapCause::IllegalInstruction(0x10200073)));
    }
}
