use rv64vm_core::cpu::StepResult;
use rv64vm_core::mmu::{Mmu, DRAM_BASE};
use rv64vm_core::uart::UART_BASE;
use rv64vm_core::{load_binary, Vm};

// addi a0, x0, 42 ; addi a7, x0, 93 (SYS_EXIT) ; ecall
const PROGRAM: &[u8] = &[
    0x13, 0x05, 0xA0, 0x02, // addi a0, x0, 42
    0x93, 0x08, 0xD0, 0x05, // addi a7, x0, 93
    0x73, 0x00, 0x00, 0x00, // ecall
];

#[test]
fn vm_runs_program_to_halt() {
    let (mut bus, entry) = load_binary(PROGRAM);
    let mut vm = Vm::new(&mut bus, entry);
    assert_eq!(vm.run(None), StepResult::Halted);
    assert!(vm.halted);
    assert_eq!(vm.cpu.pc, DRAM_BASE + 8);
    assert_eq!(vm.cpu.regs[10], 42);
}

#[test]
fn vm_breakpoint_stops_then_resumes() {
    let (mut bus, entry) = load_binary(PROGRAM);
    let mut vm = Vm::new(&mut bus, entry);
    vm.add_breakpoint(DRAM_BASE + 4);
    assert_eq!(vm.run(None), StepResult::Ok); // stopped at breakpoint
    assert_eq!(vm.cpu.pc, DRAM_BASE + 4);
    assert!(!vm.halted);
    assert!(vm.remove_breakpoint(DRAM_BASE + 4));
    assert_eq!(vm.run(None), StepResult::Halted);
    assert!(vm.halted);
}

#[test]
fn vm_uart_output_flushes_once() {
    let mut bus = Mmu::new(&[]);
    bus.write8(UART_BASE, b'H').unwrap();
    bus.write8(UART_BASE, b'i').unwrap();
    let mut vm = Vm::new(&mut bus, DRAM_BASE);
    assert_eq!(vm.read_uart(), Some("Hi"));
    assert_eq!(vm.read_uart(), None);
}

#[test]
fn vm_plain_text_load() {
    let (bus, entry) = load_binary(PROGRAM);
    assert_eq!(entry, DRAM_BASE);
    assert_eq!(bus.read32(DRAM_BASE).unwrap(), 0x02A0_0513);
}