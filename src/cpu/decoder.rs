/// RISC-V decoder: `decode()` is a single table lookup into per-format
/// functions. `Inst` groups the ~70 opcodes into a handful of format-carrier
/// variants; each carrier holds a tiny op enum decoded from funct3/funct7, so
/// decoding stays exhaustive without a 70-arm match.

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BOp {
    Beq,
    Bne,
    Blt,
    Bge,
    Bltu,
    Bgeu,
}
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LOp {
    Lb,
    Lh,
    Lw,
    Ld,
    Lbu,
    Lhu,
    Lwu,
}
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SOp {
    Sb,
    Sh,
    Sw,
    Sd,
}
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum IOp {
    Addi,
    Slti,
    Sltiu,
    Xori,
    Ori,
    Andi,
    Slli,
    Srli,
    Srai,
    Addiw,
    Slliw,
    Srliw,
    Sraiw,
}
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ROp {
    Add,
    Sub,
    Sll,
    Slt,
    Sltu,
    Xor,
    Srl,
    Sra,
    Or,
    And,
    Addw,
    Subw,
    Sllw,
    Srlw,
    Sraw,
    Mul,
    Mulh,
    Mulhsu,
    Mulhu,
    Div,
    Divu,
    Rem,
    Remu,
    Mulw,
    Divw,
    Divuw,
    Remw,
    Remuw,
}
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AmoOp {
    AmoaddW,
    AmoswapW,
    LrW,
    ScW,
    LrD,
    ScD,
}
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum COp {
    Csrrw,
    Csrrs,
    Csrrc,
    Csrrwi,
    Csrrsi,
    Csrrci,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Inst {
    // U / J / I control flow.
    Lui { rd: u8, imm: i64 },
    Auipc { rd: u8, imm: i64 },
    Jal { rd: u8, imm: i64 },
    Jalr { rd: u8, rs1: u8, imm: i64 },
    // B-type; op selects the comparison.
    Branch { op: BOp, rs1: u8, rs2: u8, imm: i64 },
    // I-type loads / stores; op selects width + signedness.
    Load { op: LOp, rd: u8, rs1: u8, imm: i64 },
    Store { op: SOp, rs1: u8, rs2: u8, imm: i64 },
    // I-type ALU; shifts reuse `imm` to carry their shamt (bits 12..17).
    IAlu { op: IOp, rd: u8, rs1: u8, imm: i64 },
    // R-type: RV64I, RV64W and RV64M share one carrier.
    RR { op: ROp, rd: u8, rs1: u8, rs2: u8 },
    // A extension.
    Amo { op: AmoOp, rd: u8, rs1: u8, rs2: u8, aq: bool, rl: bool },
    // Zicsr; `src` is rs1 for *rw/rs/rc, a zero-extended uimm for *i.
    Csr { op: COp, rd: u8, src: u8, csr: u16 },
    Ecall,
    Ebreak,
    Fence,
    Mret,
    Sret,
    Illegal(u32),
}

// BIT FIELD HELPERS: extract bits from a raw instruction word (u32).
#[inline(always)]
fn bits(x: u32, lo: u32, hi: u32) -> u32 {
    (x >> lo) & ((1 << (hi - lo + 1)) - 1)
}
#[inline(always)]
fn bit(x: u32, n: u32) -> u32 {
    (x >> n) & 1
}

// Sign-extend a field from its sign-bit position up to i64.
#[inline(always)]
fn sign_ext(x: u32, bit_pos: u32) -> i64 {
    let shift = 63 - bit_pos;
    ((x as i64) << shift) >> shift
}

// IMMEDIATE DECODERS
fn imm_i(raw: u32) -> i64 {
    sign_ext(bits(raw, 20, 31), 11)
}
fn imm_s(raw: u32) -> i64 {
    sign_ext((bits(raw, 25, 31) << 5) | bits(raw, 7, 11), 11)
}
fn imm_b(raw: u32) -> i64 {
    sign_ext(
        (bit(raw, 31) << 12) | (bit(raw, 7) << 11) | (bits(raw, 25, 30) << 5) | (bits(raw, 8, 11) << 1),
        12,
    )
}
fn imm_u(raw: u32) -> i64 {
    sign_ext(raw & 0xFFFF_F000, 31)
}
fn imm_j(raw: u32) -> i64 {
    sign_ext(
        (bit(raw, 31) << 20) | (bits(raw, 12, 19) << 12) | (bit(raw, 20) << 11) | (bits(raw, 21, 30) << 1),
        20,
    )
}

// Shift-amount fields (SLLI is 6 bits, *W variants 5).
#[inline(always)]
fn shamt6(raw: u32) -> i64 {
    ((raw >> 20) & 0x3F) as i64
}
#[inline(always)]
fn shamt5(raw: u32) -> i64 {
    ((raw >> 20) & 0x1F) as i64
}

// DECODER — computed goto behind a function-pointer table.
pub struct BitField<const LO: u32, const HI: u32>;
impl<const LO: u32, const HI: u32> BitField<LO, HI> {
    #[inline(always)]
    pub fn get(word: u32) -> u32 {
        (word & Self::mask()) >> LO
    }
    #[inline(always)]
    const fn mask() -> u32 {
        let n = HI - LO + 1;
        if n == 32 {
            u32::MAX
        } else {
            ((1u32 << n) - 1) << LO
        }
    }
}

type Rd = BitField<7, 11>;
type Rs1 = BitField<15, 19>;
type Rs2 = BitField<20, 24>;
type Funct3 = BitField<12, 14>;
type Funct7 = BitField<25, 31>;
// RV64 64-bit shifts spend bit 25 on shamt[5], so their type field is only 6 bits.
type Funct6 = BitField<26, 31>;
type CsrBits = BitField<20, 31>;

fn decode_lui(raw: u32) -> Inst {
    Inst::Lui { rd: Rd::get(raw) as u8, imm: imm_u(raw) }
}
fn decode_auipc(raw: u32) -> Inst {
    Inst::Auipc { rd: Rd::get(raw) as u8, imm: imm_u(raw) }
}
fn decode_jal(raw: u32) -> Inst {
    Inst::Jal { rd: Rd::get(raw) as u8, imm: imm_j(raw) }
}
fn decode_jalr(raw: u32) -> Inst {
    Inst::Jalr { rd: Rd::get(raw) as u8, rs1: Rs1::get(raw) as u8, imm: imm_i(raw) }
}
fn decode_branch(raw: u32) -> Inst {
    let op = match Funct3::get(raw) {
        0 => BOp::Beq,
        1 => BOp::Bne,
        4 => BOp::Blt,
        5 => BOp::Bge,
        6 => BOp::Bltu,
        7 => BOp::Bgeu,
        _ => return Inst::Illegal(raw),
    };
    Inst::Branch { op, rs1: Rs1::get(raw) as u8, rs2: Rs2::get(raw) as u8, imm: imm_b(raw) }
}
fn decode_load(raw: u32) -> Inst {
    let op = match Funct3::get(raw) {
        0 => LOp::Lb,
        1 => LOp::Lh,
        2 => LOp::Lw,
        3 => LOp::Ld,
        4 => LOp::Lbu,
        5 => LOp::Lhu,
        6 => LOp::Lwu,
        _ => return Inst::Illegal(raw),
    };
    Inst::Load { op, rd: Rd::get(raw) as u8, rs1: Rs1::get(raw) as u8, imm: imm_i(raw) }
}
fn decode_store(raw: u32) -> Inst {
    let op = match Funct3::get(raw) {
        0 => SOp::Sb,
        1 => SOp::Sh,
        2 => SOp::Sw,
        3 => SOp::Sd,
        _ => return Inst::Illegal(raw),
    };
    Inst::Store { op, rs1: Rs1::get(raw) as u8, rs2: Rs2::get(raw) as u8, imm: imm_s(raw) }
}
fn decode_op_imm(raw: u32) -> Inst {
    let rd = Rd::get(raw) as u8;
    let rs1 = Rs1::get(raw) as u8;
    let imm = imm_i(raw);
    match Funct3::get(raw) {
        0 => Inst::IAlu { op: IOp::Addi, rd, rs1, imm },
        2 => Inst::IAlu { op: IOp::Slti, rd, rs1, imm },
        3 => Inst::IAlu { op: IOp::Sltiu, rd, rs1, imm },
        4 => Inst::IAlu { op: IOp::Xori, rd, rs1, imm },
        6 => Inst::IAlu { op: IOp::Ori, rd, rs1, imm },
        7 => Inst::IAlu { op: IOp::Andi, rd, rs1, imm },
        1 if Funct6::get(raw) == 0 => Inst::IAlu { op: IOp::Slli, rd, rs1, imm: shamt6(raw) },
        5 => match Funct6::get(raw) {
            0 => Inst::IAlu { op: IOp::Srli, rd, rs1, imm: shamt6(raw) },
            0x10 => Inst::IAlu { op: IOp::Srai, rd, rs1, imm: shamt6(raw) },
            _ => Inst::Illegal(raw),
        },
        _ => Inst::Illegal(raw),
    }
}
fn decode_op_imm32(raw: u32) -> Inst {
    let rd = Rd::get(raw) as u8;
    let rs1 = Rs1::get(raw) as u8;
    match Funct3::get(raw) {
        0 => Inst::IAlu { op: IOp::Addiw, rd, rs1, imm: imm_i(raw) },
        1 => Inst::IAlu { op: IOp::Slliw, rd, rs1, imm: shamt5(raw) },
        5 => match Funct7::get(raw) {
            0 => Inst::IAlu { op: IOp::Srliw, rd, rs1, imm: shamt5(raw) },
            0x20 => Inst::IAlu { op: IOp::Sraiw, rd, rs1, imm: shamt5(raw) },
            _ => Inst::Illegal(raw),
        },
        _ => Inst::Illegal(raw),
    }
}
fn decode_op(raw: u32) -> Inst {
    let rd = Rd::get(raw) as u8;
    let rs1 = Rs1::get(raw) as u8;
    let rs2 = Rs2::get(raw) as u8;
    let op = match (Funct7::get(raw), Funct3::get(raw)) {
        (0x00, 0) => ROp::Add,
        (0x20, 0) => ROp::Sub,
        (0x00, 1) => ROp::Sll,
        (0x00, 2) => ROp::Slt,
        (0x00, 3) => ROp::Sltu,
        (0x00, 4) => ROp::Xor,
        (0x00, 5) => ROp::Srl,
        (0x20, 5) => ROp::Sra,
        (0x00, 6) => ROp::Or,
        (0x00, 7) => ROp::And,
        (0x01, 0) => ROp::Mul,
        (0x01, 1) => ROp::Mulh,
        (0x01, 2) => ROp::Mulhsu,
        (0x01, 3) => ROp::Mulhu,
        (0x01, 4) => ROp::Div,
        (0x01, 5) => ROp::Divu,
        (0x01, 6) => ROp::Rem,
        (0x01, 7) => ROp::Remu,
        _ => return Inst::Illegal(raw),
    };
    Inst::RR { op, rd, rs1, rs2 }
}
fn decode_op32(raw: u32) -> Inst {
    let rd = Rd::get(raw) as u8;
    let rs1 = Rs1::get(raw) as u8;
    let rs2 = Rs2::get(raw) as u8;
    let op = match (Funct7::get(raw), Funct3::get(raw)) {
        (0x00, 0) => ROp::Addw,
        (0x20, 0) => ROp::Subw,
        (0x00, 1) => ROp::Sllw,
        (0x00, 5) => ROp::Srlw,
        (0x20, 5) => ROp::Sraw,
        (0x01, 0) => ROp::Mulw,
        (0x01, 4) => ROp::Divw,
        (0x01, 5) => ROp::Divuw,
        (0x01, 6) => ROp::Remw,
        (0x01, 7) => ROp::Remuw,
        _ => return Inst::Illegal(raw),
    };
    Inst::RR { op, rd, rs1, rs2 }
}
fn decode_amo(raw: u32) -> Inst {
    let op = match (raw >> 27, Funct3::get(raw)) {
        (0b00000, 2) => AmoOp::AmoaddW,
        (0b00001, 2) => AmoOp::AmoswapW,
        (0b00010, 2) => AmoOp::LrW,
        (0b00011, 2) => AmoOp::ScW,
        (0b00010, 3) => AmoOp::LrD,
        (0b00011, 3) => AmoOp::ScD,
        _ => return Inst::Illegal(raw),
    };
    Inst::Amo {
        op,
        rd: Rd::get(raw) as u8,
        rs1: Rs1::get(raw) as u8,
        rs2: Rs2::get(raw) as u8,
        aq: raw & (1 << 26) != 0,
        rl: raw & (1 << 25) != 0,
    }
}
fn decode_system(raw: u32) -> Inst {
    match raw {
        0x00000073 => return Inst::Ecall,
        0x00100073 => return Inst::Ebreak,
        0x30200073 => return Inst::Mret,
        0x10200073 => return Inst::Sret,
        _ => {}
    }
    let op = match Funct3::get(raw) {
        1 => COp::Csrrw,
        2 => COp::Csrrs,
        3 => COp::Csrrc,
        5 => COp::Csrrwi,
        6 => COp::Csrrsi,
        7 => COp::Csrrci,
        _ => return Inst::Fence, // wfi / sfence.vma / fence.i
    };
    Inst::Csr { op, rd: Rd::get(raw) as u8, src: Rs1::get(raw) as u8, csr: CsrBits::get(raw) as u16 }
}
fn decode_fence(_raw: u32) -> Inst {
    Inst::Fence
}
fn decode_illegal(raw: u32) -> Inst {
    Inst::Illegal(raw)
}

type Decoder = fn(u32) -> Inst;

const OP_TABLE: [Decoder; 128] = build_op_table();

const fn build_op_table() -> [Decoder; 128] {
    let mut t = [decode_illegal as Decoder; 128];
    t[0x37] = decode_lui;
    t[0x17] = decode_auipc;
    t[0x6F] = decode_jal;
    t[0x67] = decode_jalr;
    t[0x63] = decode_branch;
    t[0x03] = decode_load;
    t[0x23] = decode_store;
    t[0x13] = decode_op_imm;
    t[0x33] = decode_op;
    t[0x1B] = decode_op_imm32;
    t[0x3B] = decode_op32;
    t[0x2F] = decode_amo;
    t[0x73] = decode_system;
    t[0x0F] = decode_fence;
    t
}

#[inline(always)]
pub fn decode(raw: u32) -> Inst {
    OP_TABLE[(raw & 0x7F) as usize](raw)
}

#[cfg(test)]
mod tests {
    use super::*;

    // Test-only instruction encoders for each format.
    fn r(funct7: u32, rs2: u32, rs1: u32, funct3: u32, rd: u32) -> u32 {
        (funct7 << 25) | (rs2 << 20) | (rs1 << 15) | (funct3 << 12) | (rd << 7) | 0x33
    }
    fn rw(funct7: u32, rs2: u32, rs1: u32, funct3: u32, rd: u32) -> u32 {
        (funct7 << 25) | (rs2 << 20) | (rs1 << 15) | (funct3 << 12) | (rd << 7) | 0x3B
    }
    fn i(imm: u32, rs1: u32, funct3: u32, rd: u32, opcode: u32) -> u32 {
        (imm << 20) | (rs1 << 15) | (funct3 << 12) | (rd << 7) | opcode
    }
    fn iw(imm: u32, rs1: u32, funct3: u32, rd: u32) -> u32 {
        (imm << 20) | (rs1 << 15) | (funct3 << 12) | (rd << 7) | 0x1B
    }
    fn s(imm: u32, rs2: u32, rs1: u32, funct3: u32) -> u32 {
        ((imm >> 5 & 0x7F) << 25) | (rs2 << 20) | (rs1 << 15) | (funct3 << 12)
            | ((imm & 0x1F) << 7) | 0x23
    }
    fn b(imm: u32, rs2: u32, rs1: u32, funct3: u32) -> u32 {
        ((imm >> 12 & 1) << 31) | ((imm >> 5 & 0x3F) << 25) | (rs2 << 20) | (rs1 << 15)
            | (funct3 << 12) | ((imm >> 1 & 0xF) << 8) | ((imm >> 11 & 1) << 7) | 0x63
    }
    fn u(imm: u32, rd: u32, opcode: u32) -> u32 {
        imm & 0xFFFFF000 | (rd << 7) | opcode
    }
    fn j(imm: u32, rd: u32) -> u32 {
        ((imm >> 20 & 1) << 31) | ((imm >> 1 & 0x3FF) << 21) | ((imm >> 11 & 1) << 20)
            | ((imm >> 12 & 0xFF) << 12) | (rd << 7) | 0x6F
    }
    fn amos(imm: u32, rs2: u32, rs1: u32, funct3: u32, rd: u32) -> u32 {
        (imm << 27) | (rs2 << 20) | (rs1 << 15) | (funct3 << 12) | (rd << 7) | 0x2F
    }

    /// Assert every (encoded, expected) pair decodes exactly.
    fn chk<const N: usize>(cases: [(u32, Inst); N]) {
        for (raw, exp) in cases {
            assert_eq!(decode(raw), exp);
        }
    }

    #[test]
    fn rtype() {
        chk([
            (r(0x00, 3, 2, 0, 1), Inst::RR { op: ROp::Add, rd: 1, rs1: 2, rs2: 3 }),
            (r(0x20, 3, 2, 0, 1), Inst::RR { op: ROp::Sub, rd: 1, rs1: 2, rs2: 3 }),
            (r(0x00, 3, 2, 1, 1), Inst::RR { op: ROp::Sll, rd: 1, rs1: 2, rs2: 3 }),
            (r(0x00, 3, 2, 2, 1), Inst::RR { op: ROp::Slt, rd: 1, rs1: 2, rs2: 3 }),
            (r(0x00, 3, 2, 3, 1), Inst::RR { op: ROp::Sltu, rd: 1, rs1: 2, rs2: 3 }),
            (r(0x00, 3, 2, 4, 1), Inst::RR { op: ROp::Xor, rd: 1, rs1: 2, rs2: 3 }),
            (r(0x00, 3, 2, 5, 1), Inst::RR { op: ROp::Srl, rd: 1, rs1: 2, rs2: 3 }),
            (r(0x20, 3, 2, 5, 1), Inst::RR { op: ROp::Sra, rd: 1, rs1: 2, rs2: 3 }),
            (r(0x00, 3, 2, 6, 1), Inst::RR { op: ROp::Or, rd: 1, rs1: 2, rs2: 3 }),
            (r(0x00, 3, 2, 7, 1), Inst::RR { op: ROp::And, rd: 1, rs1: 2, rs2: 3 }),
            (r(0x00, 31, 31, 0, 31), Inst::RR { op: ROp::Add, rd: 31, rs1: 31, rs2: 31 }),
        ]);
    }

    #[test]
    fn rtype_m() {
        chk([
            (r(0x01, 3, 2, 0, 1), Inst::RR { op: ROp::Mul, rd: 1, rs1: 2, rs2: 3 }),
            (r(0x01, 3, 2, 1, 1), Inst::RR { op: ROp::Mulh, rd: 1, rs1: 2, rs2: 3 }),
            (r(0x01, 3, 2, 2, 1), Inst::RR { op: ROp::Mulhsu, rd: 1, rs1: 2, rs2: 3 }),
            (r(0x01, 3, 2, 3, 1), Inst::RR { op: ROp::Mulhu, rd: 1, rs1: 2, rs2: 3 }),
            (r(0x01, 3, 2, 4, 1), Inst::RR { op: ROp::Div, rd: 1, rs1: 2, rs2: 3 }),
            (r(0x01, 3, 2, 5, 1), Inst::RR { op: ROp::Divu, rd: 1, rs1: 2, rs2: 3 }),
            (r(0x01, 3, 2, 6, 1), Inst::RR { op: ROp::Rem, rd: 1, rs1: 2, rs2: 3 }),
            (r(0x01, 3, 2, 7, 1), Inst::RR { op: ROp::Remu, rd: 1, rs1: 2, rs2: 3 }),
        ]);
    }

    #[test]
    fn op_imm() {
        chk([
            (i(0xFFF, 2, 0, 1, 0x13), Inst::IAlu { op: IOp::Addi, rd: 1, rs1: 2, imm: -1 }),
            (i(0, 0, 0, 0, 0x13), Inst::IAlu { op: IOp::Addi, rd: 0, rs1: 0, imm: 0 }),
            (i(0x800, 0, 0, 1, 0x13), Inst::IAlu { op: IOp::Addi, rd: 1, rs1: 0, imm: -2048 }),
            (i(0x7FF, 0, 0, 1, 0x13), Inst::IAlu { op: IOp::Addi, rd: 1, rs1: 0, imm: 2047 }),
            (i(10, 2, 2, 1, 0x13), Inst::IAlu { op: IOp::Slti, rd: 1, rs1: 2, imm: 10 }),
            (i(10, 2, 3, 1, 0x13), Inst::IAlu { op: IOp::Sltiu, rd: 1, rs1: 2, imm: 10 }),
            (i(0xFF, 2, 4, 1, 0x13), Inst::IAlu { op: IOp::Xori, rd: 1, rs1: 2, imm: 0xFF }),
            (i(0xFF, 2, 6, 1, 0x13), Inst::IAlu { op: IOp::Ori, rd: 1, rs1: 2, imm: 0xFF }),
            (i(0xFF, 2, 7, 1, 0x13), Inst::IAlu { op: IOp::Andi, rd: 1, rs1: 2, imm: 0xFF }),
        ]);
    }

    #[test]
    fn shifts() {
        chk([
            ((5 << 20) | (2 << 15) | (1 << 12) | (1 << 7) | 0x13, Inst::IAlu { op: IOp::Slli, rd: 1, rs1: 2, imm: 5 }),
            ((5 << 20) | (2 << 15) | (5 << 12) | (1 << 7) | 0x13, Inst::IAlu { op: IOp::Srli, rd: 1, rs1: 2, imm: 5 }),
            ((0x10 << 26) | (5 << 20) | (2 << 15) | (5 << 12) | (1 << 7) | 0x13, Inst::IAlu { op: IOp::Srai, rd: 1, rs1: 2, imm: 5 }),
            // shamt[5] shares bit 25 with the type field: shifts >= 32 must still decode.
            ((32 << 20) | (2 << 15) | (1 << 12) | (1 << 7) | 0x13, Inst::IAlu { op: IOp::Slli, rd: 1, rs1: 2, imm: 32 }),
            ((32 << 20) | (2 << 15) | (5 << 12) | (1 << 7) | 0x13, Inst::IAlu { op: IOp::Srli, rd: 1, rs1: 2, imm: 32 }),
            ((0x10 << 26) | (32 << 20) | (2 << 15) | (5 << 12) | (1 << 7) | 0x13, Inst::IAlu { op: IOp::Srai, rd: 1, rs1: 2, imm: 32 }),
            ((0x10 << 26) | (63 << 20) | (2 << 15) | (5 << 12) | (1 << 7) | 0x13, Inst::IAlu { op: IOp::Srai, rd: 1, rs1: 2, imm: 63 }),
        ]);
    }

    #[test]
    fn u_type() {
        chk([
            (u(0x12345000, 1, 0x37), Inst::Lui { rd: 1, imm: 0x12345000 }),
            (u(0x12345000, 1, 0x17), Inst::Auipc { rd: 1, imm: 0x12345000 }),
        ]);
    }

    #[test]
    fn jumps() {
        chk([
            (j(0x100, 1), Inst::Jal { rd: 1, imm: 0x100 }),
            (j(0x1FFFFE, 1), Inst::Jal { rd: 1, imm: -2 }),
            (i(0x100, 2, 0, 1, 0x67), Inst::Jalr { rd: 1, rs1: 2, imm: 0x100 }),
        ]);
    }

    #[test]
    fn branches() {
        chk([
            (b(8, 3, 2, 0), Inst::Branch { op: BOp::Beq, rs1: 2, rs2: 3, imm: 8 }),
            (b(8, 3, 2, 1), Inst::Branch { op: BOp::Bne, rs1: 2, rs2: 3, imm: 8 }),
            (b(8, 3, 2, 4), Inst::Branch { op: BOp::Blt, rs1: 2, rs2: 3, imm: 8 }),
            (b(8, 3, 2, 5), Inst::Branch { op: BOp::Bge, rs1: 2, rs2: 3, imm: 8 }),
            (b(8, 3, 2, 6), Inst::Branch { op: BOp::Bltu, rs1: 2, rs2: 3, imm: 8 }),
            (b(8, 3, 2, 7), Inst::Branch { op: BOp::Bgeu, rs1: 2, rs2: 3, imm: 8 }),
        ]);
    }

    #[test]
    fn loads() {
        chk([
            (i(0x100, 2, 0, 1, 0x03), Inst::Load { op: LOp::Lb, rd: 1, rs1: 2, imm: 0x100 }),
            (i(0x100, 2, 1, 1, 0x03), Inst::Load { op: LOp::Lh, rd: 1, rs1: 2, imm: 0x100 }),
            (i(0x100, 2, 2, 1, 0x03), Inst::Load { op: LOp::Lw, rd: 1, rs1: 2, imm: 0x100 }),
            (i(0x100, 2, 3, 1, 0x03), Inst::Load { op: LOp::Ld, rd: 1, rs1: 2, imm: 0x100 }),
            (i(0x100, 2, 4, 1, 0x03), Inst::Load { op: LOp::Lbu, rd: 1, rs1: 2, imm: 0x100 }),
            (i(0x100, 2, 5, 1, 0x03), Inst::Load { op: LOp::Lhu, rd: 1, rs1: 2, imm: 0x100 }),
            (i(0x100, 2, 6, 1, 0x03), Inst::Load { op: LOp::Lwu, rd: 1, rs1: 2, imm: 0x100 }),
        ]);
    }

    #[test]
    fn stores() {
        chk([
            (s(0x108, 3, 2, 0), Inst::Store { op: SOp::Sb, rs1: 2, rs2: 3, imm: 0x108 }),
            (s(0x108, 3, 2, 1), Inst::Store { op: SOp::Sh, rs1: 2, rs2: 3, imm: 0x108 }),
            (s(0x108, 3, 2, 2), Inst::Store { op: SOp::Sw, rs1: 2, rs2: 3, imm: 0x108 }),
            (s(0x108, 3, 2, 3), Inst::Store { op: SOp::Sd, rs1: 2, rs2: 3, imm: 0x108 }),
        ]);
    }

    #[test]
    fn system() {
        chk([
            (0x00000073, Inst::Ecall),
            (0x00100073, Inst::Ebreak),
            (0x30200073, Inst::Mret),
            (0x10200073, Inst::Sret),
            (0x10500073, Inst::Fence), // wfi
            (0x0FF0000F, Inst::Fence),
            (0x12000073, Inst::Fence), // sfence.vma
            (0x12050073, Inst::Fence),
        ]);
    }

    #[test]
    fn csr_ops() {
        chk([
            (0x30051073, Inst::Csr { op: COp::Csrrw, rd: 0, src: 10, csr: 0x300 }),
            (0x30052073, Inst::Csr { op: COp::Csrrs, rd: 0, src: 10, csr: 0x300 }),
            (0x30053073, Inst::Csr { op: COp::Csrrc, rd: 0, src: 10, csr: 0x300 }),
            (0x30055073, Inst::Csr { op: COp::Csrrwi, rd: 0, src: 10, csr: 0x300 }),
            (0x30056073, Inst::Csr { op: COp::Csrrsi, rd: 0, src: 10, csr: 0x300 }),
            (0x30057073, Inst::Csr { op: COp::Csrrci, rd: 0, src: 10, csr: 0x300 }),
        ]);
    }

    #[test]
    fn word_alu() {
        chk([
            (iw(0x100, 2, 0, 1), Inst::IAlu { op: IOp::Addiw, rd: 1, rs1: 2, imm: 0x100 }),
            ((5 << 20) | (2 << 15) | (1 << 12) | (1 << 7) | 0x1B, Inst::IAlu { op: IOp::Slliw, rd: 1, rs1: 2, imm: 5 }),
            ((5 << 20) | (2 << 15) | (5 << 12) | (1 << 7) | 0x1B, Inst::IAlu { op: IOp::Srliw, rd: 1, rs1: 2, imm: 5 }),
            ((0x20 << 25) | (5 << 20) | (2 << 15) | (5 << 12) | (1 << 7) | 0x1B, Inst::IAlu { op: IOp::Sraiw, rd: 1, rs1: 2, imm: 5 }),
            (rw(0x00, 3, 2, 0, 1), Inst::RR { op: ROp::Addw, rd: 1, rs1: 2, rs2: 3 }),
            (rw(0x20, 3, 2, 0, 1), Inst::RR { op: ROp::Subw, rd: 1, rs1: 2, rs2: 3 }),
            (rw(0x00, 3, 2, 1, 1), Inst::RR { op: ROp::Sllw, rd: 1, rs1: 2, rs2: 3 }),
            (rw(0x00, 3, 2, 5, 1), Inst::RR { op: ROp::Srlw, rd: 1, rs1: 2, rs2: 3 }),
            (rw(0x20, 3, 2, 5, 1), Inst::RR { op: ROp::Sraw, rd: 1, rs1: 2, rs2: 3 }),
            (rw(0x01, 3, 2, 0, 1), Inst::RR { op: ROp::Mulw, rd: 1, rs1: 2, rs2: 3 }),
            (rw(0x01, 3, 2, 4, 1), Inst::RR { op: ROp::Divw, rd: 1, rs1: 2, rs2: 3 }),
            (rw(0x01, 3, 2, 5, 1), Inst::RR { op: ROp::Divuw, rd: 1, rs1: 2, rs2: 3 }),
            (rw(0x01, 3, 2, 6, 1), Inst::RR { op: ROp::Remw, rd: 1, rs1: 2, rs2: 3 }),
            (rw(0x01, 3, 2, 7, 1), Inst::RR { op: ROp::Remuw, rd: 1, rs1: 2, rs2: 3 }),
        ]);
    }

    #[test]
    fn atomics() {
        chk([
            (amos(0, 3, 2, 2, 1), Inst::Amo { op: AmoOp::AmoaddW, rd: 1, rs1: 2, rs2: 3, aq: false, rl: false }),
            (amos(1, 3, 2, 2, 1), Inst::Amo { op: AmoOp::AmoswapW, rd: 1, rs1: 2, rs2: 3, aq: false, rl: false }),
            (amos(2, 0, 2, 2, 1), Inst::Amo { op: AmoOp::LrW, rd: 1, rs1: 2, rs2: 0, aq: false, rl: false }),
            (amos(3, 3, 2, 2, 1), Inst::Amo { op: AmoOp::ScW, rd: 1, rs1: 2, rs2: 3, aq: false, rl: false }),
            (amos(2, 0, 2, 3, 1), Inst::Amo { op: AmoOp::LrD, rd: 1, rs1: 2, rs2: 0, aq: false, rl: false }),
            (amos(3, 3, 2, 3, 1), Inst::Amo { op: AmoOp::ScD, rd: 1, rs1: 2, rs2: 3, aq: false, rl: false }),
        ]);
    }

    #[test]
    fn illegal() {
        chk([
            (0xFFFFFFFF, Inst::Illegal(0xFFFFFFFF)),
            (i(0x100, 2, 7, 1, 0x03), Inst::Illegal(i(0x100, 2, 7, 1, 0x03))),
            (s(0x108, 3, 2, 7), Inst::Illegal(s(0x108, 3, 2, 7))),
        ]);
    }
}
