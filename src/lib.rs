#![cfg_attr(not(feature = "std"), no_std)]

extern crate alloc;

#[cfg(not(feature = "std"))]
use alloc::{vec, vec::Vec};

pub mod cpu;
pub mod dtb;
pub mod mmu;
pub mod sbi;
pub mod traps;
pub mod uart;
pub mod virtio;

use crate::cpu::csr::{MIP_MTIP, MIP_SEIP, MIP_SSIP, MIP_STIP};
use crate::cpu::{Cpu, StepResult};
use crate::mmu::{Mmu, DRAM_BASE, DRAM_END};
use crate::traps::TrapCause;

#[cfg(feature = "std")]
use std::collections::HashSet;

#[cfg(not(feature = "std"))]
use hashbrown::HashSet;

pub const KERNEL_BASE: u64 = 0x8020_0000;
pub const DTB_ADDR: u64 = 0x8600_0000;
pub const SYS_EXIT: u64 = 93;
pub const SBI_SHUTDOWN: u64 = 8;

/// A memory bus the CPU runs against: RAM, MMIO devices, timers, and UART.
///
/// Implemented by the concrete [`Mmu`]; kept generic so tests and embedders
/// can substitute their own bus without touching the CPU core.
pub trait Bus {
    fn read8(&self, addr: u64) -> Result<u8, TrapCause>;
    fn read16(&self, addr: u64) -> Result<u16, TrapCause>;
    fn read32(&self, addr: u64) -> Result<u32, TrapCause>;
    fn read64(&self, addr: u64) -> Result<u64, TrapCause>;
    fn write8(&mut self, addr: u64, val: u8) -> Result<(), TrapCause>;
    fn write16(&mut self, addr: u64, val: u16) -> Result<(), TrapCause>;
    fn write32(&mut self, addr: u64, val: u32) -> Result<(), TrapCause>;
    fn write64(&mut self, addr: u64, val: u64) -> Result<(), TrapCause>;
    fn read_phys64(&self, addr: u64) -> Result<u64, TrapCause>;
    fn tick(&mut self);
    fn timer_pending(&self) -> bool;
    fn softint_pending(&self) -> bool;
    fn mtime(&self) -> u64;
    fn set_mtimecmp(&mut self, v: u64);
    fn has_pending_plic_s(&self) -> bool;
    fn read_uart(&mut self) -> &str;
}

/// Core VM state that can run in no_std environments.
pub struct Vm<'a, B: Bus = Mmu> {
    pub cpu: Cpu,
    pub bus: &'a mut B,
    pub breakpoints: HashSet<u64>,
    pub halted: bool,
}

impl<'a, B: Bus> Vm<'a, B> {
    pub fn new(bus: &'a mut B, entry: u64) -> Self {
        let cpu = Cpu::new(entry);
        Self {
            cpu,
            bus,
            breakpoints: HashSet::new(),
            halted: false,
        }
    }

    /// Single VM tick - call repeatedly from executor
    pub fn tick(&mut self) -> StepResult {
        self.vm_tick()
    }

    /// Run until halted, breakpoint, or max instructions
    pub fn run(&mut self, max_instructions: Option<u64>) -> StepResult {
        let max = max_instructions.unwrap_or(u64::MAX);
        for _ in 0..max {
            let r = self.vm_tick();
            if !matches!(r, StepResult::Ok) {
                return r;
            }
            if self.breakpoints.contains(&self.cpu.pc) {
                return StepResult::Ok;
            }
        }
        StepResult::Ok
    }

    fn vm_tick(&mut self) -> StepResult {
        if self.bus.timer_pending() {
            self.cpu.csr.set_mip(MIP_MTIP);
        } else {
            self.cpu.csr.clear_mip(MIP_MTIP);
        }
        if self.cpu.csr.sstc_enabled() && self.bus.mtime() >= self.cpu.csr.stimecmp() {
            self.cpu.csr.set_mip(MIP_STIP);
        } else {
            self.cpu.csr.clear_mip(MIP_STIP);
        }
        if self.bus.has_pending_plic_s() {
            self.cpu.csr.set_mip(MIP_SEIP);
        } else {
            self.cpu.csr.clear_mip(MIP_SEIP);
        }
        if self.bus.softint_pending() {
            self.cpu.csr.set_mip(MIP_SSIP);
        } else {
            self.cpu.csr.clear_mip(MIP_SSIP);
        }

        let result_step = self.cpu.step(self.bus);

        match result_step {
            StepResult::Trap(TrapCause::EcallFromM) => {
                if self.cpu.regs[17] == SYS_EXIT {
                    self.halted = true;
                    return StepResult::Halted;
                }
                self.cpu.pc = self.cpu.pc.wrapping_add(4);
            }
            StepResult::Trap(TrapCause::EcallFromS) => {
                let a7 = self.cpu.regs[17];
                let a0 = self.cpu.regs[10];
                let a1 = self.cpu.regs[11];
                let a2 = self.cpu.regs[12];
                let a6 = self.cpu.regs[16];
                let res = crate::sbi::handle_sbi(a7, a0, a1, a2, a6, self.bus);
                self.cpu.regs[10] = res.a0;
                if res.halt {
                    self.halted = true;
                    return StepResult::Halted;
                }
                self.cpu.pc = self.cpu.pc.wrapping_add(4);
            }
            _ => {}
        }
        result_step
    }

    pub fn add_breakpoint(&mut self, addr: u64) {
        self.breakpoints.insert(addr);
    }

    pub fn remove_breakpoint(&mut self, addr: u64) -> bool {
        self.breakpoints.remove(&addr)
    }

    pub fn read_uart(&mut self) -> Option<&str> {
        let out = self.bus.read_uart();
        if out.is_empty() { None } else { Some(out) }
    }
}

/// Load binary (ELF or raw) into memory
pub fn load_binary(bytes: &[u8]) -> (Mmu, u64) {
    if let Some((flat, entry)) = load_elf(bytes) {
        (Mmu::from_dram(flat), entry)
    } else {
        (Mmu::new(bytes), DRAM_BASE)
    }
}

fn load_elf(bytes: &[u8]) -> Option<(Vec<u8>, u64)> {
    if bytes.len() < 64 || bytes.get(0..4)? != [0x7f, b'E', b'L', b'F'] {
        return None;
    }
    if *bytes.get(4)? != 2 { return None; }
    if *bytes.get(5)? != 1 { return None; }
    if *bytes.get(6)? != 1 { return None; }
    let machine = u16::from_le_bytes(bytes.get(18..20)?.try_into().ok()?);
    if machine != 0xF3 { return None; }

    let entry = u64::from_le_bytes(bytes.get(24..32)?.try_into().ok()?);
    let phoff = u64::from_le_bytes(bytes.get(32..40)?.try_into().ok()?) as usize;
    let phentsize = u16::from_le_bytes(bytes.get(54..56)?.try_into().ok()?) as usize;
    let phnum = u16::from_le_bytes(bytes.get(56..58)?.try_into().ok()?) as usize;
    let dram_size = (DRAM_END - DRAM_BASE) as usize;
    let mut flat = vec![0u8; dram_size];

    for idx in 0..phnum {
        let off = phoff + idx * phentsize;
        let p_type = u32::from_le_bytes(bytes.get(off..off + 4)?.try_into().ok()?);
        if p_type != 1 { continue; }
        let p_offset = u64::from_le_bytes(bytes.get(off + 8..off + 16)?.try_into().ok()?) as usize;
        let p_vaddr = u64::from_le_bytes(bytes.get(off + 16..off + 24)?.try_into().ok()?) as usize;
        let p_filesz = u64::from_le_bytes(bytes.get(off + 32..off + 40)?.try_into().ok()?) as usize;
        let p_memsz = u64::from_le_bytes(bytes.get(off + 40..off + 48)?.try_into().ok()?) as usize;
        if p_vaddr < DRAM_BASE as usize || p_vaddr + p_memsz > DRAM_END as usize {
            return None;
        }
        flat[p_vaddr - DRAM_BASE as usize..p_vaddr - DRAM_BASE as usize + p_filesz]
            .copy_from_slice(&bytes[p_offset..p_offset + p_filesz]);
    }
    Some((flat, entry))
}