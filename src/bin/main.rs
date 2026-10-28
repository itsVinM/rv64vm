use rv64vm_core::cpu::csr::{
    Privilege, CSR_CYCLE, CSR_INSTRET, CSR_MEDELEG, CSR_MEPC, CSR_MSTATUS, MSTATUS_SIE,
    MSTATUS_SPIE, MSTATUS_SPP,
};
use rv64vm_core::cpu::{Cpu, StepResult};
use rv64vm_core::mmu::{Mmu, DRAM_BASE};
use rv64vm_core::traps::TrapCause;
use rv64vm_core::{load_binary, Bus, Vm, DTB_ADDR, KERNEL_BASE, SBI_SHUTDOWN};
use std::{
    collections::HashSet,
    env, fs,
    io::{self, BufRead, Write},
};

const DEMO: &[u32] = &[
    0x0000_0113, 0x0010_0193, 0x0000_0293, 0x00A0_0313, 0x0062_8C63, 0x0031_0233,
    0x0001_8113, 0x0002_0193, 0x0012_8293, 0xFEDF_F06F, 0x0001_0513, 0x0010_0073,
];

fn build_smode_demo() -> Vec<u8> {
    let mut code = Vec::new();
    for &ch in b"Hello from S-mode!\n" {
        let addi_a0: u32 = ((ch as u32) << 20) | (10 << 7) | 0x13;
        code.extend_from_slice(&addi_a0.to_le_bytes());
        let li_a7: u32 = (1 << 20) | (17 << 7) | 0x13;
        code.extend_from_slice(&li_a7.to_le_bytes());
        code.extend_from_slice(&0x0000_0073u32.to_le_bytes());
    }
    let li_a7_8: u32 = ((SBI_SHUTDOWN as u32) << 20) | (17 << 7) | 0x13;
    code.extend_from_slice(&li_a7_8.to_le_bytes());
    code.extend_from_slice(&0x0000_0073u32.to_le_bytes());
    code
}

fn write_dtb(bus: &mut Mmu) {
    let dtb = rv64vm_core::dtb::generate_dtb();
    let off = (DTB_ADDR - DRAM_BASE) as usize;
    let dram = &mut bus.dram_mut().data;
    let end = (off + dtb.len()).min(dram.len());
    dram[off..end].copy_from_slice(&dtb[..end - off]);
}

fn boot_smode(cpu: &mut Cpu, bus: &mut Mmu, entry: u64) {
    write_dtb(bus);
    cpu.regs[10] = 0;
    cpu.regs[11] = DTB_ADDR;
    let ms = cpu.csr.mstatus() & !(MSTATUS_SPP | MSTATUS_SPIE | MSTATUS_SIE);
    cpu.csr.write(CSR_MSTATUS, ms, Privilege::M);
    cpu.csr.write(CSR_MEDELEG, u64::MAX, Privilege::M);
    cpu.csr.write(CSR_MEPC, entry, Privilege::M);
    cpu.priv_level = Privilege::S;
    cpu.pc = entry;
}

const DIM: &str = "\x1b[2m";
const RST: &str = "\x1b[0m";
const YLW: &str = "\x1b[33m";
const GRN: &str = "\x1b[32m";
const RED: &str = "\x1b[31m";
const CYN: &str = "\x1b[36m";

fn print_state(cpu: &Cpu, bus: &Mmu, bps: &HashSet<u64>) {
    println!("{CYN}-- registers ------------------------------------------------------{RST}");
    for i in 0..32 {
        let v = cpu.regs[i];
        let c = if v != 0 { GRN } else { DIM };
        print!("{c}{:>4}:{RST} {:#018x}  ", Cpu::reg_name(i), v);
        if i % 4 == 3 {
            println!();
        }
    }
    println!("\n{CYN}-- instructions ---------------------------------------------------{RST}");
    for offset in -3i64..=3 {
        let addr = cpu.pc.wrapping_add((offset * 4) as u64);
        let raw = bus.read32(addr).unwrap_or(0);
        let (arrow, color) = match (offset == 0, bps.contains(&addr)) {
            (true, true) => (">*", YLW),
            (true, false) => ("> ", YLW),
            (false, true) => (" *", RED),
            _ => ("  ", DIM),
        };
        println!("{color}{arrow} {addr:#010x}: {raw:08x}{RST}");
    }
    println!("\n{CYN}-- status ---------------------------------------------------------{RST}");
    println!(
        "  halted={:<5}  mode={:?}  bp={}  cycle={}  instret={}",
        cpu.halted,
        cpu.priv_level,
        bps.len(),
        cpu.csr.read(CSR_CYCLE, Privilege::M),
        cpu.csr.read(CSR_INSTRET, Privilege::M)
    );
}

fn prompt(bps: &HashSet<u64>) -> String {
    let marker = if bps.is_empty() { "" } else { " [bp]" };
    print!("\n{YLW}dbg{marker}>{RST} ");
    io::stdout().flush().ok();
    let mut line = String::new();
    let _ = io::stdin().lock().read_line(&mut line);
    line.trim().to_string()
}

fn parse_addr(s: &str) -> Option<u64> {
    u64::from_str_radix(s.trim_start_matches("0x"), 16).ok()
}

fn run_debugger(vm: &mut Vm<Mmu>) {
    let mut bps: HashSet<u64> = HashSet::new();
    print_help();
    loop {
        print_state(&vm.cpu, vm.bus, &bps);
        if vm.halted {
            println!("{YLW}VM halted. Use 'reset' to restart.{RST}");
            break;
        }
        let cmd = prompt(&bps);
        let mut parts = cmd.splitn(2, ' ');
        let verb = parts.next().unwrap_or("");
        let rest = parts.next().unwrap_or("").trim();
        match verb {
            "s" | "" => {
                let _ = vm.tick();
            }
            "r" | "c" => {
                let n = if verb == "c" { None } else { Some(rest.parse().unwrap_or(100)) };
                if matches!(vm.run(n), StepResult::Halted) {
                    vm.halted = true;
                }
            }
            "b" => {
                let addr = parse_addr(rest).unwrap_or(vm.cpu.pc);
                if bps.remove(&addr) {
                    println!("  breakpoint removed at {addr:#010x}");
                } else {
                    bps.insert(addr);
                    println!("  breakpoint set at {addr:#010x}");
                }
            }
            "reg" => {
                let mut args = rest.splitn(2, ' ');
                let name = args.next().unwrap_or("");
                if name.is_empty() {
                    for i in 0..32 {
                        println!("  {:>4}: {:#018x}", Cpu::reg_name(i), vm.cpu.regs[i]);
                    }
                } else if let Some(i) = (0..32)
                    .find(|&i| Cpu::reg_name(i) == name || format!("x{i}") == name || i.to_string() == name)
                {
                    let val = args.next().unwrap_or("").trim();
                    if val.is_empty() {
                        println!("  {} = {:#018x}", Cpu::reg_name(i), vm.cpu.regs[i]);
                    } else if let Some(v) = parse_addr(val).or_else(|| val.parse().ok()) {
                        vm.cpu.regs[i] = v;
                        println!("  {} := {:#018x}", Cpu::reg_name(i), v);
                    } else {
                        println!("{DIM}  bad value{RST}");
                    }
                } else {
                    println!("{DIM}  unknown register '{name}'{RST}");
                }
            }
            "mem" => {
                let mut args = rest.splitn(2, ' ');
                match args.next().and_then(parse_addr) {
                    Some(addr) => {
                        let count: usize = args.next().unwrap_or("8").trim().parse().unwrap_or(8);
                        for i in 0..count {
                            let a = addr.wrapping_add((i * 4) as u64);
                            if i % 4 == 0 {
                                print!("  {a:#010x}: ");
                            }
                            print!("{:08x} ", vm.bus.read32(a).unwrap_or(0));
                            if i % 4 == 3 {
                                println!();
                            }
                        }
                        if count % 4 != 0 {
                            println!();
                        }
                    }
                    None => println!("{DIM}  usage: mem addr [n]  or mem8/16/32/64 addr{RST}"),
                }
            }
            "mem8" | "mem16" | "mem32" | "mem64" => match parse_addr(rest) {
                Some(addr) => {
                    let (v, w) = match verb {
                        "mem8" => (vm.bus.read8(addr).unwrap_or(0) as u64, 4),
                        "mem16" => (vm.bus.read16(addr).unwrap_or(0) as u64, 6),
                        "mem32" => (vm.bus.read32(addr).unwrap_or(0) as u64, 10),
                        _ => (vm.bus.read64(addr).unwrap_or(0), 18),
                    };
                    println!("  [{addr:#010x}]: {v:#0w$x}");
                }
                None => println!("{DIM}  bad address{RST}"),
            },
            "csr" => match parse_addr(rest) {
                Some(addr) => println!(
                    "  CSR[{:#06x}] = {:#018x}",
                    addr,
                    vm.cpu.csr.read(addr as usize, Privilege::M)
                ),
                None => println!("{DIM}  usage: csr <hex-addr>{RST}"),
            },
            "reset" => {
                let pc = vm.cpu.pc;
                vm.cpu = Cpu::new(pc);
                println!("  CPU reset to pc={pc:#010x}");
            }
            "h" | "help" => print_help(),
            "q" | "quit" => break,
            other if !other.is_empty() => println!("{DIM}  unknown '{other}'. try h for help{RST}"),
            _ => {}
        }
    }
}

fn drain_uart(bus: &mut Mmu) {
    let out = bus.read_uart();
    if !out.is_empty() {
        print!("{out}");
    }
}

fn run_headless(vm: &mut Vm<Mmu>, max: Option<u64>) {
    let mut left = max.unwrap_or(u64::MAX);
    loop {
        match vm.tick() {
            StepResult::Halted => {
                drain_uart(vm.bus);
                let _ = io::stdout().flush();
                println!(
                    "\nhalted  pc={:#010x}  a0={}  cycles={}",
                    vm.cpu.pc,
                    vm.cpu.regs[10] as i64,
                    vm.cpu.csr.read(CSR_CYCLE, Privilege::M)
                );
                break;
            }
            StepResult::Trap(t) if !matches!(t, TrapCause::EcallFromM | TrapCause::EcallFromS) => {
                drain_uart(vm.bus);
                let _ = io::stdout().flush();
                eprintln!("trap {:?}  pc={:#010x}", t, vm.cpu.pc);
                break;
            }
            _ => drain_uart(vm.bus),
        }
        if left == 0 {
            let _ = io::stdout().flush();
            break;
        }
        left -= 1;
    }
    let _ = io::stdout().flush();
}

fn print_help() {
    println!("{CYN}RISC-V64 live debugger{RST}");
    println!("  {GRN}s{RST}           step one instruction");
    println!("  {GRN}r [n]{RST}       run n instructions (default 100)");
    println!("  {GRN}c{RST}           continue until breakpoint or halt");
    println!("  {GRN}b [addr]{RST}    toggle breakpoint at addr (hex, default PC)");
    println!("  {GRN}reg [r] [v]{RST} read/write register by name/number; no args = show all");
    println!("  {GRN}mem addr [n]{RST} dump n x 32-bit words (default 8) at addr");
    println!("  {GRN}mem8/16/32/64 addr{RST}  read 1/2/4/8 bytes");
    println!("  {GRN}csr addr{RST}    read a CSR (hex address, e.g. 0x300 for mstatus)");
    println!("  {GRN}reset{RST}       reset CPU to initial state (preserves memory)");
    println!("  {GRN}h{RST}           this help");
    println!("  {GRN}q{RST}           quit");
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args: Vec<String> = env::args().collect();
    let has = |a: &str| args.iter().any(|x| x == a);
    let debug = has("--debug") || has("-d");
    let smode = has("--sbi") || has("-s");
    let max = args
        .iter()
        .position(|a| a == "--max" || a == "--steps")
        .and_then(|i| args.get(i + 1))
        .and_then(|v| v.parse().ok());

    let mut skip = false;
    let path = args.iter().skip(1).find(|a| {
        if skip {
            skip = false;
            return false;
        }
        if *a == "--max" || *a == "--steps" {
            skip = true;
            return false;
        }
        !a.starts_with('-')
    });

    let (mut bus, entry) = match path {
        Some(p) => load_binary(&fs::read(p)?),
        None if smode => {
            println!("no kernel supplied -- running built-in S-mode demo");
            (Mmu::new_at(build_smode_demo(), KERNEL_BASE), KERNEL_BASE)
        }
        None => {
            println!("no binary supplied -- running built-in demo: fib(10)");
            let code: Vec<u8> = DEMO.iter().flat_map(|w| w.to_le_bytes()).collect();
            (Mmu::new(&code), DRAM_BASE)
        }
    };

    let mut vm = Vm::new(&mut bus, entry);
    if smode {
        boot_smode(&mut vm.cpu, vm.bus, entry);
    }
    if debug {
        run_debugger(&mut vm);
    } else {
        run_headless(&mut vm, max);
    }
    Ok(())
}
