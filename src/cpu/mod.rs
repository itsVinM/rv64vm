pub mod csr;
pub mod decoder;
pub mod executor;

use crate::cpu::csr::{
    CsrFile, Privilege, CSR_MCAUSE, CSR_MEPC, CSR_MSTATUS, CSR_MTVAL, CSR_MTVEC, CSR_SCAUSE,
    CSR_SEPC, CSR_STVAL, CSR_STVEC, MSTATUS_MIE, MSTATUS_MPIE, MSTATUS_MPP, MSTATUS_SIE,
    MSTATUS_SPIE, MSTATUS_SPP,
};
use crate::Bus;
use crate::traps::{pending_interrupt, TrapCause};
use decoder::decode;
use executor::execute;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StepResult {
    Ok,
    Halted,
    Trap(TrapCause),
}

pub struct Cpu {
    pub regs: [u64; 32],
    pub pc: u64,
    pub csr: CsrFile,
    pub halted: bool,
    pub priv_level: Privilege,
}

impl Cpu {
    pub fn new(entry: u64) -> Self {
        let mut regs = [0u64; 32];
        regs[2] = crate::mmu::DRAM_END;
        Self {
            regs,
            pc: entry,
            csr: CsrFile::new(),
            halted: false,
            priv_level: Privilege::M,
        }
    }

    pub fn step<B: Bus>(&mut self, bus: &mut B) -> StepResult {
        if self.halted {
            return StepResult::Halted;
        }

        bus.tick();
        self.csr.inc_cycle();

        // Check pending interrupts — both M and S level
        if let Some((irq, delegated)) = pending_interrupt(
            self.csr.mstatus(),
            self.csr.mie(),
            self.csr.mip(),
            self.priv_level,
            self.csr.mideleg(),
        ) {
            if delegated && self.priv_level < Privilege::M {
                // Delegate to S-mode
                self.take_s_trap(irq, self.pc, 0);
            } else {
                // Take in M-mode
                self.take_trap(irq, self.pc, 0);
            }
            return StepResult::Ok;
        }

        if self.pc & 3 != 0 {
            let cause = TrapCause::InstructionAddressMisaligned;
            self.take_trap(cause, self.pc, self.pc);
            return StepResult::Trap(cause);
        }

        // Fetch with virtual memory translation — one shared page walker
        // (executor::translate), raising the fault instead of aliasing VA→PA.
        let fetch_pa = match executor::fetch_phys(self, bus) {
            Ok(pa) => pa,
            Err(cause) => {
                self.take_trap(cause, self.pc, self.pc);
                return StepResult::Trap(cause);
            }
        };

        let raw = match bus.read32(fetch_pa) {
            Ok(v) => v,
            Err(_e) => {
                let cause = if self.priv_level >= Privilege::S {
                    TrapCause::InstructionAccessFault
                } else {
                    TrapCause::InstructionPageFault
                };
                self.take_trap(cause, self.pc, self.pc);
                return StepResult::Trap(cause);
            }
        };

        let result = execute(self, bus, decode(raw));
        self.csr.inc_instret();

        if result.halt {
            self.halted = true;
            return StepResult::Halted;
        }

        if let Some(trap) = result.trap {
            // Don't redirect pc for ecalls — caller (vm_tick) handles them directly
            match trap {
                TrapCause::EcallFromU | TrapCause::EcallFromS | TrapCause::EcallFromM => {}
                _ => {
                    let medeleg = self.csr.medeleg();
                    let exc_code = trap.exception_code();
                    let delegate =
                        self.priv_level <= Privilege::S && (medeleg & (1 << exc_code)) != 0;
                    if delegate && self.priv_level < Privilege::M {
                        self.take_s_trap(trap, self.pc, 0);
                    } else {
                        self.take_trap(trap, self.pc, 0);
                    }
                }
            }
            return StepResult::Trap(trap);
        }

        if let Some(new) = result.new_priv {
            self.priv_level = new;
        }
        self.regs[0] = 0;
        self.pc = result.next_pc;
        StepResult::Ok
    }

    fn take_trap(&mut self, cause: TrapCause, pc: u64, tval: u64) {
        let code = cause.code();
        self.csr.write(CSR_MEPC, pc, Privilege::M);
        self.csr.write(CSR_MCAUSE, code, Privilege::M);
        self.csr.write(CSR_MTVAL, tval, Privilege::M);

        let mut ms = self.csr.mstatus();
        let mie = (ms & MSTATUS_MIE) >> 3;
        ms &= !(MSTATUS_MIE | MSTATUS_MPIE | MSTATUS_MPP);
        ms |= mie << 7;
        ms |= self.priv_level.bits() << 11;
        self.csr.write(CSR_MSTATUS, ms, Privilege::M);

        let mtvec = self.csr.read(CSR_MTVEC, Privilege::M);
        self.pc = if cause.is_interrupt() && (mtvec & 1 == 1) {
            (mtvec & !3).wrapping_add(4 * (code & 0x7FFF_FFFF_FFFF_FFFF))
        } else {
            mtvec & !3
        };

        self.priv_level = Privilege::M;
    }

    fn take_s_trap(&mut self, cause: TrapCause, pc: u64, tval: u64) {
        let code = cause.code();
        self.csr.write(CSR_SEPC, pc, Privilege::S);
        self.csr.write(CSR_SCAUSE, code, Privilege::S);
        self.csr.write(CSR_STVAL, tval, Privilege::S);

        let mut ms = self.csr.mstatus();
        let sie = (ms & MSTATUS_SIE) >> 1;
        ms &= !(MSTATUS_SIE | MSTATUS_SPIE | MSTATUS_SPP);
        ms |= sie << 5;
        ms |= (self.priv_level.bits() & 1) << 8;
        self.csr.write(CSR_MSTATUS, ms, Privilege::M);

        let stvec = self.csr.read(CSR_STVEC, Privilege::S);
        self.pc = if cause.is_interrupt() && (stvec & 1 == 1) {
            (stvec & !3).wrapping_add(4 * (code & 0x7FFF_FFFF_FFFF_FFFF))
        } else {
            stvec & !3
        };

        self.priv_level = Privilege::S;
    }

    pub fn reg_name(r: usize) -> &'static str {
        const NAMES: [&str; 32] = [
            "zero", "ra", "sp", "gp", "tp", "t0", "t1", "t2", "s0", "s1", "a0", "a1", "a2", "a3",
            "a4", "a5", "a6", "a7", "s2", "s3", "s4", "s5", "s6", "s7", "s8", "s9", "s10", "s11",
            "t3", "t4", "t5", "t6",
        ];
        NAMES[r]
    }
}
