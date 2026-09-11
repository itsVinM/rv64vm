pub const CSR_MSTATUS: usize = 0x300;
pub const CSR_MISA: usize = 0x301;
pub const CSR_MEDELEG: usize = 0x302;
pub const CSR_MIDELEG: usize = 0x303;
pub const CSR_MIE: usize = 0x304;
pub const CSR_MTVEC: usize = 0x305;
pub const CSR_MENVCFG: usize = 0x30A;
pub const CSR_MEPC: usize = 0x341;
pub const CSR_MCAUSE: usize = 0x342;
pub const CSR_MTVAL: usize = 0x343;
pub const CSR_MIP: usize = 0x344;

pub const CSR_SSTATUS: usize = 0x100;
pub const CSR_SIE: usize = 0x104;
pub const CSR_STVEC: usize = 0x105;
pub const CSR_STIMECMP: usize = 0x14D;
pub const CSR_SEPC: usize = 0x141;
pub const CSR_SCAUSE: usize = 0x142;
pub const CSR_STVAL: usize = 0x143;
pub const CSR_SIP: usize = 0x144;
pub const CSR_SATP: usize = 0x180;

pub const CSR_CYCLE: usize = 0xC00;
pub const CSR_TIME: usize = 0xC01;
pub const CSR_INSTRET: usize = 0xC02;

const NUM_CSRS: usize = 4096;

pub const MENVCFG_STCE: u64 = 1 << 63;

pub const MSTATUS_MIE: u64 = 1 << 3;
pub const MSTATUS_MPIE: u64 = 1 << 7;
pub const MSTATUS_MPP: u64 = 0b11 << 11;
pub const MSTATUS_SPP: u64 = 1 << 8;
pub const MSTATUS_SPIE: u64 = 1 << 5;
pub const MSTATUS_SIE: u64 = 1 << 1;
pub const MSTATUS_SUM: u64 = 1 << 18;
pub const MSTATUS_MXR: u64 = 1 << 19;
pub const MSTATUS_FS: u64 = 0b11 << 13;
pub const MSTATUS_XS: u64 = 0b11 << 15;
pub const MSTATUS_VS: u64 = 0b11 << 9;

/// Bits of mstatus exposed to S-mode through sstatus (read & write gate).
const SSTATUS_MASK: u64 = MSTATUS_SPP
    | MSTATUS_SPIE
    | MSTATUS_SIE
    | MSTATUS_FS
    | MSTATUS_XS
    | MSTATUS_VS
    | MSTATUS_SUM
    | MSTATUS_MXR;

pub const MIP_SSIP: u64 = 1 << 1;
pub const MIP_STIP: u64 = 1 << 5;
pub const MIP_SEIP: u64 = 1 << 9;
pub const MIP_MSIP: u64 = 1 << 3;
pub const MIP_MTIP: u64 = 1 << 7;
pub const MIP_MEIP: u64 = 1 << 11;

pub const SATP_PPN: u64 = (1 << 44) - 1;
pub const SATP_MODE_BARE: u64 = 0;
pub const SATP_MODE_SV39: u64 = 8;

pub struct CsrFile {
    regs: [u64; NUM_CSRS],
    stimecmp: u64,
}

impl CsrFile {
    pub fn new() -> Self {
        let mut s = Self {
            regs: [0u64; NUM_CSRS],
            stimecmp: u64::MAX,
        };
        s.regs[CSR_MISA] = (2u64 << 62) | (1 << 0) | (1 << 8) | (1 << 12);                        // MXL=2 (RV64), A, I, M extensions
        s.regs[CSR_MEDELEG] = (1 << 3) | (1 << 8) | (1 << 9) | (1 << 12) | (1 << 13) | (1 << 15); // delegate exceptions to S-mode
        s.regs[CSR_MIDELEG] = MIP_SSIP | MIP_STIP | MIP_SEIP;
        s
    }

    pub fn read(&self, addr: usize, priv_level: Privilege) -> u64 {
        if addr >= NUM_CSRS {
            return 0;
        }
        match addr {
            CSR_SSTATUS => self.regs[CSR_MSTATUS] & SSTATUS_MASK,
            CSR_SIE => self.regs[CSR_MIE] & self.regs[CSR_MIDELEG],
            CSR_SIP => self.regs[CSR_MIP] & self.regs[CSR_MIDELEG],
            CSR_SATP if priv_level == Privilege::S => self.regs[CSR_SATP],
            CSR_TIME => self.regs[CSR_CYCLE], // time = cycle count
            CSR_CYCLE | CSR_INSTRET => self.regs[addr],
            CSR_STIMECMP if priv_level <= Privilege::S => {
                if priv_level == Privilege::S && !self.sstc_enabled() {
                    0
                } else {
                    self.stimecmp
                }
            }
            _ => {
                if priv_level < Privilege::M && (0x300..=0x3FF).contains(&addr) {
                    return 0;
                }
                self.regs[addr]
            }
        }
    }

    pub fn write(&mut self, addr: usize, val: u64, priv_level: Privilege) {
        if addr >= NUM_CSRS {
            return;
        }
        match addr {
            CSR_CYCLE | CSR_TIME | CSR_INSTRET => {}
            CSR_SSTATUS => {
                self.regs[CSR_MSTATUS] =
                    (self.regs[CSR_MSTATUS] & !SSTATUS_MASK) | (val & SSTATUS_MASK);
            }
            CSR_SIE => {
                let mask = self.regs[CSR_MIDELEG] & (MIP_SSIP | MIP_STIP | MIP_SEIP);
                let delegated = val & mask;
                self.regs[CSR_MIE] = (self.regs[CSR_MIE] & !mask) | delegated;
            }
            CSR_SIP => {
                let mask = self.regs[CSR_MIDELEG] & (MIP_SSIP | MIP_STIP | MIP_SEIP);
                let delegated = val & mask;
                self.regs[CSR_MIP] = (self.regs[CSR_MIP] & !mask) | delegated;
            }
            CSR_SATP if priv_level == Privilege::S => {
                self.regs[CSR_SATP] = val;
            }
            CSR_STIMECMP if priv_level <= Privilege::S => {
                if priv_level == Privilege::S && !self.sstc_enabled() {
                    return;
                }
                self.stimecmp = val;
            }
            _ => {
                if (0x300..=0x3FF).contains(&addr) && priv_level != Privilege::M {
                    return;
                }
                self.regs[addr] = val;
            }
        }
    }

    pub fn read_write(&mut self, addr: usize, val: u64, op: CsrOp, priv_level: Privilege) -> u64 {
        let old = self.read(addr, priv_level);
        let new = match op {
            CsrOp::Write => val,
            CsrOp::Set => old | val,
            CsrOp::Clear => old & !val,
        };
        self.write(addr, new, priv_level);
        old
    }

    pub fn mstatus(&self) -> u64 {
        self.regs[CSR_MSTATUS]
    }
    pub fn mie(&self) -> u64 {
        self.regs[CSR_MIE]
    }
    pub fn mip(&self) -> u64 {
        self.regs[CSR_MIP]
    }

    /// Set one or more MIP bits (raise an interrupt source).
    pub fn set_mip(&mut self, bits: u64) {
        self.regs[CSR_MIP] |= bits;
    }

    /// Clear one or more MIP bits (deassert an interrupt source).
    pub fn clear_mip(&mut self, bits: u64) {
        self.regs[CSR_MIP] &= !bits;
    }

    pub fn inc_cycle(&mut self) {
        self.regs[CSR_CYCLE] += 1;
    }
    pub fn inc_instret(&mut self) {
        self.regs[CSR_INSTRET] += 1;
    }

    pub fn medeleg(&self) -> u64 {
        self.regs[CSR_MEDELEG]
    }
    pub fn mideleg(&self) -> u64 {
        self.regs[CSR_MIDELEG]
    }

    pub fn sstc_enabled(&self) -> bool {
        (self.regs[CSR_MENVCFG] & MENVCFG_STCE) != 0
    }

    pub fn stimecmp(&self) -> u64 {
        self.stimecmp
    }
}

#[derive(Clone, Copy, PartialEq, Eq)]
pub enum CsrOp {
    Write,
    Set,
    Clear,
}

#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Debug)]
pub enum Privilege {
    U = 0,
    S = 1,
    M = 3,
}

impl Privilege {
    pub fn bits(self) -> u64 {
        match self {
            Privilege::U => 0,
            Privilege::S => 1,
            Privilege::M => 3,
        }
    }
}

pub fn csr_access_ok(addr: usize, priv_level: Privilege) -> bool {
    if priv_level == Privilege::M {
        return true;
    }
    match addr {
        0x100..=0x1FF => priv_level >= Privilege::S,
        0x300..=0x3FF => false,
        0xC00..=0xC1F => true,
        0xC80..=0xC9F => true,
        _ => false,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // ---- csr_access_ok ----
    #[test]
    fn test_m_can_read_anything() {
        assert!(csr_access_ok(0x300, Privilege::M));
        assert!(csr_access_ok(0x100, Privilege::M));
        assert!(csr_access_ok(0x000, Privilege::M));
        assert!(csr_access_ok(0xFFF, Privilege::M));
    }
    #[test]
    fn test_s_can_read_s_csrs() {
        assert!(csr_access_ok(0x100, Privilege::S)); // sstatus
        assert!(csr_access_ok(0x105, Privilege::S)); // stvec
        assert!(csr_access_ok(0x180, Privilege::S)); // satp
        assert!(csr_access_ok(0x1FF, Privilege::S)); // top of S range
    }
    #[test]
    fn test_s_cannot_read_m_csrs() {
        assert!(!csr_access_ok(0x300, Privilege::S));
        assert!(!csr_access_ok(0x341, Privilege::S));
        assert!(!csr_access_ok(0x3FF, Privilege::S));
    }
    #[test]
    fn test_s_can_read_counters() {
        assert!(csr_access_ok(0xC00, Privilege::S)); // cycle
        assert!(csr_access_ok(0xC01, Privilege::S)); // time
        assert!(csr_access_ok(0xC02, Privilege::S)); // instret
        assert!(csr_access_ok(0xC1F, Privilege::S));
        assert!(csr_access_ok(0xC80, Privilege::S));
        assert!(csr_access_ok(0xC9F, Privilege::S));
    }
    #[test]
    fn test_u_cannot_read_m_csrs() {
        assert!(!csr_access_ok(0x300, Privilege::U));
        assert!(!csr_access_ok(0x100, Privilege::U));
    }
    #[test]
    fn test_u_can_read_counters() {
        assert!(csr_access_ok(0xC00, Privilege::U));
        assert!(csr_access_ok(0xC80, Privilege::U));
    }

    // ---- CsrFile construction ----
    #[test]
    fn test_new_csr_misa() {
        let csr = CsrFile::new();
        let misa = csr.read(CSR_MISA, Privilege::M);
        assert!(misa & (1 << 0) != 0); // A extension
        assert!(misa & (1 << 8) != 0); // I extension
        assert!(misa & (1 << 12) != 0); // M extension
        assert!((misa >> 62) == 2); // MXL = 2 (RV64)
    }
    #[test]
    fn test_new_csr_deleg() {
        let csr = CsrFile::new();
        let medeleg = csr.read(CSR_MEDELEG, Privilege::M);
        assert_eq!(
            medeleg,
            (1 << 3) | (1 << 8) | (1 << 9) | (1 << 12) | (1 << 13) | (1 << 15)
        );
        let mideleg = csr.read(CSR_MIDELEG, Privilege::M);
        assert_eq!(mideleg, MIP_SSIP | MIP_STIP | MIP_SEIP);
    }

    // ---- CSR read/write ----
    #[test]
    fn test_read_write_mstatus() {
        let mut csr = CsrFile::new();
        csr.write(CSR_MSTATUS, 0xFFFFFFFFFFFFFFFF, Privilege::M);
        assert_eq!(csr.read(CSR_MSTATUS, Privilege::M), 0xFFFFFFFFFFFFFFFF);
    }
    #[test]
    fn test_sstatus_masks_mstatus() {
        let mut csr = CsrFile::new();
        csr.write(CSR_MSTATUS, 0xFFFFFFFFFFFFFFFF, Privilege::M);
        // sstatus only exposes SPP, SPIE, SIE, FS, XS, VS, SUM, MXR
        let mask = MSTATUS_SPP
            | MSTATUS_SPIE
            | MSTATUS_SIE
            | MSTATUS_FS
            | MSTATUS_XS
            | MSTATUS_VS
            | MSTATUS_SUM
            | MSTATUS_MXR;
        let sstatus_val = csr.read(CSR_SSTATUS, Privilege::S);
        assert_eq!(sstatus_val, mask);
        assert_eq!(sstatus_val & !mask, 0);
    }
    #[test]
    fn test_sstatus_write_masking() {
        let mut csr = CsrFile::new();
        csr.write(CSR_MSTATUS, 0, Privilege::M);
        let mask = MSTATUS_SPP
            | MSTATUS_SPIE
            | MSTATUS_SIE
            | MSTATUS_FS
            | MSTATUS_XS
            | MSTATUS_VS
            | MSTATUS_SUM
            | MSTATUS_MXR;
        csr.write(CSR_SSTATUS, 0xFFFFFFFFFFFFFFFF, Privilege::S);
        let mstatus = csr.read(CSR_MSTATUS, Privilege::M);
        assert_eq!(mstatus, mask);
        assert_eq!(mstatus & !mask, 0);
    }
    #[test]
    fn test_sie_masking_via_mideleg() {
        let mut csr = CsrFile::new();
        // CSRs SIE writes only affect delegated bits (SSIP, STIP, SEIP)
        csr.write(CSR_SIE, 0xFFFFFFFFFFFFFFFF, Privilege::S);
        let mie = csr.read(CSR_MIE, Privilege::M);
        assert_eq!(mie, MIP_SSIP | MIP_STIP | MIP_SEIP);
    }
    #[test]
    fn test_read_counter_in_any_mode() {
        let mut csr = CsrFile::new();
        for _ in 0..42 {
            csr.inc_cycle();
        }
        assert_eq!(csr.read(CSR_CYCLE, Privilege::S), 42);
        assert_eq!(csr.read(CSR_CYCLE, Privilege::U), 42);
    }
    #[test]
    fn test_write_counter_noop() {
        let mut csr = CsrFile::new();
        csr.write(CSR_CYCLE, 42, Privilege::M);
        assert_eq!(csr.read(CSR_CYCLE, Privilege::M), 0); // writes to cycle are ignored
    }
    
    // ---- MIP handling ----
    #[test]
    fn test_mip_timer() {
        let mut csr = CsrFile::new();
        assert_eq!(csr.mip() & MIP_MTIP, 0);
        csr.set_mip(MIP_MTIP);
        assert!(csr.mip() & MIP_MTIP != 0);
        csr.clear_mip(MIP_MTIP);
        assert_eq!(csr.mip() & MIP_MTIP, 0);
    }
    #[test]
    fn test_mip_ssip() {
        let mut csr = CsrFile::new();
        csr.set_mip(MIP_SSIP);
        assert!(csr.mip() & MIP_SSIP != 0);
        csr.clear_mip(MIP_SSIP);
        assert_eq!(csr.mip() & MIP_SSIP, 0);
    }
    #[test]
    fn test_inc_cycle_and_instret() {
        let mut csr = CsrFile::new();
        assert_eq!(csr.read(CSR_CYCLE, Privilege::M), 0);
        assert_eq!(csr.read(CSR_INSTRET, Privilege::M), 0);
        csr.inc_cycle();
        csr.inc_instret();
        csr.inc_instret();
        assert_eq!(csr.read(CSR_CYCLE, Privilege::M), 1);
        assert_eq!(csr.read(CSR_INSTRET, Privilege::M), 2);
    }

    // ---- SSTC ----
    #[test]
    fn test_sstc_disabled_by_default() {
        let csr = CsrFile::new();
        assert!(!csr.sstc_enabled());
    }
    #[test]
    fn test_sstc_enable() {
        let mut csr = CsrFile::new();
        csr.write(CSR_MENVCFG, MENVCFG_STCE, Privilege::M);
        assert!(csr.sstc_enabled());
    }
    #[test]
    fn test_stimecmp_access_when_sstc_disabled() {
        let mut csr = CsrFile::new();
        csr.write(CSR_STIMECMP, 100, Privilege::S);
        assert_eq!(csr.stimecmp(), u64::MAX); // S-mode write ignored when SSTC disabled
    }

    // ---- Privilege ----
    #[test]
    fn test_privilege_bits() {
        assert_eq!(Privilege::U.bits(), 0);
        assert_eq!(Privilege::S.bits(), 1);
        assert_eq!(Privilege::M.bits(), 3);
    }
    #[test]
    fn test_privilege_ordering() {
        assert!(Privilege::U < Privilege::S);
        assert!(Privilege::S < Privilege::M);
    }
}
