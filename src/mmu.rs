#[cfg(not(feature = "std"))]
use alloc::{vec, vec::Vec};

use crate::traps::TrapCause;
use crate::uart;
use crate::virtio::Virtio;

// Memory map
pub const DRAM_BASE: u64 = 0x8000_0000;
pub const DRAM_END: u64 = 0x8800_0000;
pub const CLINT_BASE: u64 = 0x0200_0000;
pub const CLINT_END: u64 = 0x020F_FFFF;
pub const PLIC_BASE: u64 = 0x0C00_0000;
pub const PLIC_END: u64 = 0x0FFF_FFFF;
pub const VIRTIO_BASE: u64 = 0x1000_1000;
pub const VIRTIO_END: u64 = 0x1000_1FFF;

const DRAM_SIZE: u64 = 128 * 1024 * 1024;

// DRAM
pub struct Memory {
    pub data: Vec<u8>,
}

impl Memory {
    fn new() -> Self {
        Self {
            data: vec![0u8; DRAM_SIZE as usize],
        }
    }

    fn load_binary(bytes: &[u8]) -> Self {
        let mut m = Self::new();
        m.data[..bytes.len().min(DRAM_SIZE as usize)]
            .copy_from_slice(&bytes[..bytes.len().min(DRAM_SIZE as usize)]);
        m
    }

    fn into_dram(mut data: Vec<u8>) -> Self {
        data.resize(DRAM_SIZE as usize, 0);
        Self { data }
    }

    #[inline(always)]
    fn off(&self, addr: u64) -> Option<usize> {
        (DRAM_BASE..DRAM_BASE + DRAM_SIZE)
            .contains(&addr)
            .then(|| (addr - DRAM_BASE) as usize)
    }

    fn read8(&self, addr: u64) -> Option<u8> {
        Some(self.data[self.off(addr)?])
    }
    fn read32(&self, addr: u64) -> Option<u32> {
        let i = self.off(addr)?;
        Some(u32::from_le_bytes(self.data[i..i + 4].try_into().ok()?))
    }
    fn read64(&self, addr: u64) -> Option<u64> {
        let i = self.off(addr)?;
        Some(u64::from_le_bytes(self.data[i..i + 8].try_into().ok()?))
    }
    fn write8(&mut self, addr: u64, v: u8) {
        if let Some(i) = self.off(addr) {
            self.data[i] = v;
        }
    }
    fn write32(&mut self, addr: u64, v: u32) {
        if let Some(i) = self.off(addr) {
            self.data[i..i + 4].copy_from_slice(&v.to_le_bytes());
        }
    }
    fn write64(&mut self, addr: u64, v: u64) {
        if let Some(i) = self.off(addr) {
            self.data[i..i + 8].copy_from_slice(&v.to_le_bytes());
        }
    }
}

// CLINT — timer and software interrupts
// mtime increments every tick; fires when mtime >= mtimecmp.
const MSIP_LO: u64 = 0x0000;
const MTIMECMP_LO: u64 = 0x4000;
const MTIMECMP_HI: u64 = 0x4004;
const MTIME_LO: u64 = 0xBFF8;
const MTIME_HI: u64 = 0xBFFC;

pub struct Clint {
    pub mtime: u64,
    pub mtimecmp: u64,
    msip: u64,
}

impl Clint {
    fn new() -> Self {
        Self {
            mtime: 0,
            mtimecmp: u64::MAX,
            msip: 0,
        }
    }
    pub fn timer_pending(&self) -> bool {
        self.mtime >= self.mtimecmp
    }
    pub fn softint_pending(&self) -> bool {
        (self.msip & 1) != 0
    }

    fn read(&self, offset: u64) -> u32 {
        match offset {
            MSIP_LO => self.msip as u32,
            MTIMECMP_LO => self.mtimecmp as u32,
            MTIMECMP_HI => (self.mtimecmp >> 32) as u32,
            MTIME_LO => self.mtime as u32,
            MTIME_HI => (self.mtime >> 32) as u32,
            _ => 0,
        }
    }

    fn read64(&self, offset: u64) -> u64 {
        match offset {
            MSIP_LO => self.msip,
            MTIMECMP_LO => self.mtimecmp,
            MTIME_LO => self.mtime,
            _ => 0,
        }
    }

    fn write(&mut self, offset: u64, val: u32) {
        match offset {
            MSIP_LO => self.msip = val as u64,
            MTIMECMP_LO => self.mtimecmp = (self.mtimecmp & 0xFFFF_FFFF_0000_0000) | val as u64,
            MTIMECMP_HI => {
                self.mtimecmp = (self.mtimecmp & 0x0000_0000_FFFF_FFFF) | ((val as u64) << 32)
            }
            MTIME_LO => self.mtime = (self.mtime & 0xFFFF_FFFF_0000_0000) | val as u64,
            MTIME_HI => self.mtime = (self.mtime & 0x0000_0000_FFFF_FFFF) | ((val as u64) << 32),
            _ => {}
        }
    }

    fn write64(&mut self, offset: u64, val: u64) {
        match offset {
            MSIP_LO => self.msip = val,
            MTIMECMP_LO => self.mtimecmp = val,
            MTIME_LO => self.mtime = val,
            _ => {}
        }
    }
}

// PLIC — external interrupt controller
struct Plic {
    priority: [u32; 53],
    pending: u64,
    m_enabled: u64,
    s_enabled: u64,
}

impl Plic {
    fn new() -> Self {
        Self {
            priority: [0; 53],
            pending: 0,
            m_enabled: 0,
            s_enabled: 0,
        }
    }

    fn claim(&self, context: u64) -> u32 {
        let en = if context == 0 {
            self.m_enabled
        } else {
            self.s_enabled
        };
        let active = self.pending & en;
        if active == 0 {
            0
        } else {
            active.trailing_zeros() + 1
        }
    }

    fn read(&self, offset: u64) -> u32 {
        match offset {
            0x000..=0x0D0 => {
                let i = (offset / 4) as usize;
                if i < 53 {
                    self.priority[i]
                } else {
                    0
                }
            }
            0x1000 => self.pending as u32,
            0x1004 => (self.pending >> 32) as u32,
            0x2000 => self.m_enabled as u32,
            0x2004 => (self.m_enabled >> 32) as u32,
            0x2080 => self.s_enabled as u32,
            0x2084 => (self.s_enabled >> 32) as u32,
            0x20_0000 => 0, // M-threshold (ignored)
            0x20_0004 => self.claim(0),
            0x201_000 => 0, // S-threshold (ignored)
            0x201_004 => self.claim(1),
            _ => 0,
        }
    }

    fn write(&mut self, offset: u64, val: u32) {
        match offset {
            0x000..=0x0D0 => {
                let i = (offset / 4) as usize;
                if i < 53 {
                    self.priority[i] = val;
                }
            }
            0x2000 => self.m_enabled = (self.m_enabled & 0xFFFF_FFFF_0000_0000) | val as u64,
            0x2004 => {
                self.m_enabled = (self.m_enabled & 0x0000_0000_FFFF_FFFF) | ((val as u64) << 32)
            }
            0x2080 => self.s_enabled = (self.s_enabled & 0xFFFF_FFFF_0000_0000) | val as u64,
            0x2084 => {
                self.s_enabled = (self.s_enabled & 0x0000_0000_FFFF_FFFF) | ((val as u64) << 32)
            }
            0x20_0004 if (1..64).contains(&val) => {
                self.pending &= !(1u64 << (val - 1));
            }
            0x201_004 if (1..64).contains(&val) => {
                self.pending &= !(1u64 << (val - 1));
            }
            _ => {}
        }
    }

    fn has_pending_s(&self) -> bool {
        (self.pending & self.s_enabled) != 0
    }

    fn set_pending(&mut self, irq: u32) {
        self.pending |= 1u64 << irq;
    }
}

// MMU / Bus
pub struct Mmu {
    dram: Memory,
    pub clint: Clint,
    plic: Plic,
    pub uart: uart::Uart,
    pub virtio: Virtio,
}

impl Mmu {
    pub fn new(binary: &[u8]) -> Self {
        Self {
            dram: Memory::load_binary(binary),
            clint: Clint::new(),
            plic: Plic::new(),
            uart: uart::Uart::new(),
            virtio: Virtio::new(),
        }
    }

    pub fn from_dram(data: Vec<u8>) -> Self {
        Self {
            dram: Memory::into_dram(data),
            clint: Clint::new(),
            plic: Plic::new(),
            uart: uart::Uart::new(),
            virtio: Virtio::new(),
        }
    }

    /// Load binary at a specific address (used for S-mode kernel at KERNEL_BASE)
    pub fn new_at(data: Vec<u8>, base: u64) -> Self {
        let mut m = Self {
            dram: Memory::new(),
            clint: Clint::new(),
            plic: Plic::new(),
            uart: uart::Uart::new(),
            virtio: Virtio::new(),
        };
        let offset = (base - DRAM_BASE) as usize;
        let end = (offset + data.len()).min(DRAM_SIZE as usize);
        m.dram.data[offset..end].copy_from_slice(&data[..end - offset]);
        m
    }

    pub fn has_pending_plic_s(&self) -> bool {
        self.plic.has_pending_s()
    }

    pub fn dram_mut(&mut self) -> &mut Memory {
        &mut self.dram
    }

    pub fn tick(&mut self) {
        self.clint.mtime = self.clint.mtime.wrapping_add(1);
    }

    pub fn read8(&self, addr: u64) -> Result<u8, TrapCause> {
        match addr {
            DRAM_BASE..=DRAM_END => self.dram.read8(addr).ok_or(TrapCause::LoadAccessFault),
            CLINT_BASE..=CLINT_END => Ok(self.clint.read(addr - CLINT_BASE) as u8),
            PLIC_BASE..=PLIC_END => Ok(self.plic.read(addr - PLIC_BASE) as u8),
            uart::UART_BASE..=uart::UART_END => self.uart.read8(addr - uart::UART_BASE),
            VIRTIO_BASE..=VIRTIO_END => Ok(self.virtio.read32(addr - VIRTIO_BASE) as u8),
            _ => Err(TrapCause::LoadAccessFault),
        }
    }

    pub fn read16(&self, addr: u64) -> Result<u16, TrapCause> {
        Ok(self.read8(addr)? as u16 | ((self.read8(addr + 1)? as u16) << 8))
    }

    pub fn read32(&self, addr: u64) -> Result<u32, TrapCause> {
        match addr {
            DRAM_BASE..=DRAM_END => self.dram.read32(addr).ok_or(TrapCause::LoadAccessFault),
            CLINT_BASE..=CLINT_END => Ok(self.clint.read(addr - CLINT_BASE)),
            PLIC_BASE..=PLIC_END => Ok(self.plic.read(addr - PLIC_BASE)),
            uart::UART_BASE..=uart::UART_END => Ok(self.uart.read8(addr - uart::UART_BASE)? as u32),
            VIRTIO_BASE..=VIRTIO_END => Ok(self.virtio.read32(addr - VIRTIO_BASE)),
            _ => Err(TrapCause::LoadAccessFault),
        }
    }

    pub fn read64(&self, addr: u64) -> Result<u64, TrapCause> {
        match addr {
            DRAM_BASE..=DRAM_END => self.dram.read64(addr).ok_or(TrapCause::LoadAccessFault),
            CLINT_BASE..=CLINT_END => Ok(self.clint.read64(addr - CLINT_BASE)),
            _ => Ok(self.read32(addr)? as u64 | ((self.read32(addr + 4)? as u64) << 32)),
        }
    }

    pub fn write8(&mut self, addr: u64, val: u8) -> Result<(), TrapCause> {
        match addr {
            DRAM_BASE..=DRAM_END => {
                self.dram.write8(addr, val);
                Ok(())
            }
            CLINT_BASE..=CLINT_END => {
                self.clint.write(addr - CLINT_BASE, val as u32);
                Ok(())
            }
            PLIC_BASE..=PLIC_END => {
                self.plic.write(addr - PLIC_BASE, val as u32);
                Ok(())
            }
            uart::UART_BASE..=uart::UART_END => self.uart.write8(addr - uart::UART_BASE, val),
            VIRTIO_BASE..=VIRTIO_END => Ok(()),
            _ => Err(TrapCause::StoreAccessFault),
        }
    }

    pub fn write16(&mut self, addr: u64, val: u16) -> Result<(), TrapCause> {
        self.write8(addr, val as u8)?;
        self.write8(addr + 1, (val >> 8) as u8)
    }

    pub fn write32(&mut self, addr: u64, val: u32) -> Result<(), TrapCause> {
        match addr {
            DRAM_BASE..=DRAM_END => {
                self.dram.write32(addr, val);
                Ok(())
            }
            CLINT_BASE..=CLINT_END => {
                let off = addr - CLINT_BASE;
                #[cfg(feature = "std")]
                let old = self.clint.mtimecmp;
                self.clint.write(off, val);
                #[cfg(feature = "std")]
                if off == 0x4000 || off == 0x4004 {
                    eprintln!(
                        "CLINT write32 mtimecmp off={:#x}: {:#x} -> {:#x} (old={:#x}) mtime={:#x}",
                        off, old, self.clint.mtimecmp, old, self.clint.mtime
                    );
                }
                Ok(())
            }
            PLIC_BASE..=PLIC_END => {
                self.plic.write(addr - PLIC_BASE, val);
                Ok(())
            }
            VIRTIO_BASE..=VIRTIO_END => {
                let off = addr - VIRTIO_BASE;
                self.virtio.write32(off, val);
                if off == 0x050 {
                    self.virtio.handle_notify(&mut self.dram.data);
                    self.plic.set_pending(2);
                }
                Ok(())
            }
            _ => Err(TrapCause::StoreAccessFault),
        }
    }

    /// Physical memory read — bypasses translation, used by page table walker
    pub fn read_phys64(&self, addr: u64) -> Result<u64, TrapCause> {
        match addr {
            DRAM_BASE..=DRAM_END => self.dram.read64(addr).ok_or(TrapCause::LoadAccessFault),
            CLINT_BASE..=CLINT_END => Ok(self.clint.read64(addr - CLINT_BASE)),
            _ => Ok(self.read32(addr)? as u64 | ((self.read32(addr + 4)? as u64) << 32)),
        }
    }

    pub fn write64(&mut self, addr: u64, val: u64) -> Result<(), TrapCause> {
        match addr {
            DRAM_BASE..=DRAM_END => {
                self.dram.write64(addr, val);
                Ok(())
            }
            CLINT_BASE..=CLINT_END => {
                let off = addr - CLINT_BASE;
                #[cfg(feature = "std")]
                let old = self.clint.mtimecmp;
                self.clint.write64(off, val);
                #[cfg(feature = "std")]
                if off == 0x4000 {
                    eprintln!(
                        "CLINT write64 mtimecmp: {:#x} -> {:#x} (old={:#x}) mtime={:#x}",
                        old, self.clint.mtimecmp, old, self.clint.mtime
                    );
                }
                Ok(())
            }
            _ => {
                self.write32(addr, val as u32)?;
                self.write32(addr + 4, (val >> 32) as u32)
            }
        }
    }
}

impl crate::Bus for Mmu {
    fn read8(&self, addr: u64) -> Result<u8, TrapCause> {
        self.read8(addr)
    }
    fn read16(&self, addr: u64) -> Result<u16, TrapCause> {
        self.read16(addr)
    }
    fn read32(&self, addr: u64) -> Result<u32, TrapCause> {
        self.read32(addr)
    }
    fn read64(&self, addr: u64) -> Result<u64, TrapCause> {
        self.read64(addr)
    }
    fn write8(&mut self, addr: u64, val: u8) -> Result<(), TrapCause> {
        self.write8(addr, val)
    }
    fn write16(&mut self, addr: u64, val: u16) -> Result<(), TrapCause> {
        self.write16(addr, val)
    }
    fn write32(&mut self, addr: u64, val: u32) -> Result<(), TrapCause> {
        self.write32(addr, val)
    }
    fn write64(&mut self, addr: u64, val: u64) -> Result<(), TrapCause> {
        self.write64(addr, val)
    }
    fn read_phys64(&self, addr: u64) -> Result<u64, TrapCause> {
        self.read_phys64(addr)
    }
    fn tick(&mut self) {
        self.tick()
    }
    fn timer_pending(&self) -> bool {
        self.clint.timer_pending()
    }
    fn softint_pending(&self) -> bool {
        self.clint.softint_pending()
    }
    fn mtime(&self) -> u64 {
        self.clint.mtime
    }
    fn set_mtimecmp(&mut self, v: u64) {
        self.clint.mtimecmp = v;
    }
    fn has_pending_plic_s(&self) -> bool {
        self.has_pending_plic_s()
    }
    fn read_uart(&mut self) -> &str {
        self.uart.flush()
    }
}