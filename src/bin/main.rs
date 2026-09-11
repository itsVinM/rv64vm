use rv64vm_core::{load_binary, KERNEL_BASE, DTB_ADDR, SYS_EXIT, SBI_SHUTDOWN};
use rv64vm_core::cpu::csr::{MIP_MTIP, MIP_SEIP, MIP_SSIP, MIP_STIP, Privilege, CSR_CYCLE, CSR_INSTRET, CSR_MEDELEG, CSR_MEPC, CSR_MSTATUS, MSTATUS_SIE, MSTATUS_SPIE, MSTATUS_SPP};
use rv64vm_core::cpu::{Cpu, StepResult};
use rv64vm_core::mmu::{Mmu, DRAM_BASE};
use rv64vm_core::traps::TrapCause;
use rv64vm_core::sbi;
use std::{
    collections::HashSet,
    env, fs,
    io::{self, BufRead, Write},
};

const DEMO: &[u32] = &[
    0x0000_0113,
    0x0010_0193,
    0x0000_0293,
    0x00A0_0313,
    0x0062_8C63,
    0x0031_0233,
    0x0001_8113,
    0x0002_0193,
    0x0012_8293,
    0xFEDF_F06F,
    0x0001_0513,
    0x0010_0073,
];

fn build_smode_demo() -> Vec<u8> {
    let mut code = Vec::new();
    let msg = b"Hello from S-mode!\n";
    for &ch in msg.iter() {
        let imm = ch as u32;
        let addi_a0 = (imm << 20) | (10 << 7) | 0x13;
        code.extend_from_slice(&addi_a0.to_le_bytes());
        let li_a7: u32 = (1 << 20) | (17 << 7) | 0x13;
        code.extend_from_slice(&li_a7.to_le_bytes());
        code.extend_from_slice(&0x00000073u32.to_le_bytes());
    }
    let li_a7_8: u32 = (SBI_SHUTDOWN as u32) << 20 | (17 << 7) | 0x13;
    code.extend_from_slice(&li_a7_8.to_le_bytes());
    code.extend_from_slice(&0x00000073u32.to_le_bytes());
    code
}

fn write_dtb(bus: &mut Mmu, addr: u64) -> Result<(), Box<dyn std::error::Error>> {
    let dtb = rv64vm_core::dtb::generate_dtb();
    let off = (addr - DRAM_BASE) as usize;
    let dram = &mut bus.dram_mut().data;
    let end = (off + dtb.len()).min(dram.len());
    dram[off..end].copy_from_slice(&dtb[..end - off]);
    Ok(())
}

fn boot_smode(cpu: &mut Cpu, bus: &mut Mmu, entry: u64) -> Result<(), Box<dyn std::error::Error>> {
    let _ = write_dtb(bus, DTB_ADDR);
    cpu.regs[10] = 0;
    cpu.regs[11] = DTB_ADDR;
    let ms = cpu.csr.mstatus() & !(MSTATUS_SPP | MSTATUS_SPIE | MSTATUS_SIE);
    cpu.csr.write(CSR_MSTATUS, ms, Privilege::M);
    cpu.csr.write(CSR_MEDELEG, u64::MAX, Privilege::M);
    cpu.csr.write(CSR_MEPC, entry, Privilege::M);
    cpu.priv_level = Privilege::S;
    cpu.pc = entry;
    Ok(())
}

const DIM: &str = "\x1b[2m";
const RST: &str = "\x1b[0m";
const YLW: &str = "\x1b[33m";
const GRN: &str = "\x1b[32m";
const RED: &str = "\x1b[31m";
const CYN: &str = "\x1b[36m";

fn print_state(cpu: &Cpu, bus: &Mmu, bps: &HashSet<u64>) {
    println!("{CYN}── registers ────────────────────────────────────────────────────────{RST}");
    for i in 0..32usize {
        let v = cpu.regs[i];
        let color = if v != 0 { GRN } else { DIM };
        print!("{color}{:>4}:{RST} {:#018x}  ", Cpu::reg_name(i), v);
        if i % 4 == 3 { println!(); }
    }
    println!("\n{CYN}── instructions ────────────────────────────────────────────────────{RST}");
    for offset in -3i64..=3 {
        let addr = cpu.pc.wrapping_add((offset * 4) as u64);
        let raw = bus.read32(addr).unwrap_or(0);
        let is_pc = offset == 0;
        let is_bp = bps.contains(&addr);
        let (arrow, color) = match (is_pc, is_bp) {
            (true, true) => ("►●", YLW),
            (true, false) => ("► ", YLW),
            (false, true) => (" ●", RED),
            (false, false) => ("  ", DIM),
        };
        println!("{color}{arrow} {addr:#010x}: {raw:08x}{RST}");
    }
    let cycle = cpu.csr.read(CSR_CYCLE, Privilege::M);
    let instret = cpu.csr.read(CSR_INSTRET, Privilege::M);
    println!("\n{CYN}── status ───────────────────────────────────────────────────────────{RST}");
    println!("  halted={:<5}  mode={:?}  breakpoints={}  cycle={}  instret={}",
        cpu.halted, cpu.priv_level, bps.len(), cycle, instret);
    if !bps.is_empty() {
        let mut sorted: Vec<u64> = bps.iter().copied().collect();
        sorted.sort_unstable();
        let bp_list: Vec<String> = sorted.iter().map(|a| format!("{a:#010x}")).collect();
        println!("  breakpoints: {}", bp_list.join("  "));
    }
}

fn prompt(bps: &HashSet<u64>) -> String {
    let marker = if bps.is_empty() { "" } else { " [bp]" };
    print!("\n{YLW}dbg{marker}>{RST} ");
    io::stdout().flush().ok();
    let mut line = String::new();
    io::stdin().lock().read_line(&mut line).ok();
    line.trim().to_string()
}

fn run_debugger(cpu: &mut Cpu, bus: &mut Mmu, _initial_entry: u64) {
    let mut bps: HashSet<u64> = HashSet::new();
    print_help();
    loop {
        print_state(cpu, bus, &bps);
        if cpu.halted {
            println!("{YLW}VM halted. Use 'reset' to restart.{RST}");
            break;
        }
        let cmd = prompt(&bps);
        let mut parts = cmd.splitn(2, ' ');
        let verb = parts.next().unwrap_or("");
        let rest = parts.next().unwrap_or("").trim().to_string();
        match verb {
            "s" | "" => { vm_tick(cpu, bus); }
            "r" => {
                let n: u64 = rest.parse().unwrap_or(100);
                let r = run_until(cpu, bus, &bps, Some(n));
                if matches!(r, StepResult::Halted) { cpu.halted = true; }
            }
            "c" => {
                let r = run_until(cpu, bus, &bps, None);
                if matches!(r, StepResult::Halted) { cpu.halted = true; }
            }
            "b" => {
                let addr = if rest.is_empty() { cpu.pc }
                    else { u64::from_str_radix(rest.trim_start_matches("0x"), 16).unwrap_or(cpu.pc) };
                if bps.remove(&addr) { println!("  breakpoint removed at {addr:#010x}"); }
                else { bps.insert(addr); println!("  breakpoint set at {addr:#010x}"); }
            }
            "reg" => {
                let mut args = rest.splitn(2, ' ');
                let rname = args.next().unwrap_or("");
                if rname.is_empty() {
                    for i in 0..32usize { println!("  {:>4}: {:#018x}", Cpu::reg_name(i), cpu.regs[i]); }
                } else {
                    let idx = (0..32).find(|&i| Cpu::reg_name(i) == rname || format!("x{i}") == rname
                        || (rname.starts_with('x') && rname[1..].parse::<usize>().ok() == Some(i))
                        || i.to_string() == rname);
                    match idx {
                        Some(i) => {
                            let val_str = args.next().unwrap_or("").trim();
                            if val_str.is_empty() { println!("  {} = {:#018x}", Cpu::reg_name(i), cpu.regs[i]); }
                            else if let Ok(v) = if val_str.starts_with("0x") || val_str.starts_with("0X") {
                                u64::from_str_radix(&val_str[2..], 16)
                            } else { val_str.parse::<u64>() } {
                                cpu.regs[i] = v;
                                println!("  {} := {:#018x}", Cpu::reg_name(i), v);
                            } else { println!("{DIM}  bad value{RST}"); }
                        }
                        None => println!("{DIM}  unknown register '{rname}'{RST}"),
                    }
                }
            }
            "mem" => {
                let mut args = rest.splitn(2, ' ');
                let addr_str = args.next().unwrap_or("");
                if addr_str.is_empty() { println!("{DIM}  usage: mem addr [n]  or mem8/16/32/64 addr{RST}"); }
                else if let Ok(addr) = u64::from_str_radix(addr_str.trim_start_matches("0x"), 16) {
                    let count: usize = args.next().unwrap_or("8").parse().unwrap_or(8);
                    for i in 0..count {
                        let a = addr.wrapping_add((i * 4) as u64);
                        let v = bus.read32(a).unwrap_or(0);
                        if i % 4 == 0 { print!("  {a:#010x}: "); }
                        print!("{v:08x} ");
                        if i % 4 == 3 { println!(); }
                    }
                    if !count.is_multiple_of(4) { println!(); }
                } else { println!("{DIM}  bad address{RST}"); }
            }
            "mem8" | "mem16" | "mem32" | "mem64" => {
                let size = match verb { "mem8" => 1, "mem16" => 2, "mem32" => 4, _ => 8 };
                if let Ok(addr) = u64::from_str_radix(rest.trim_start_matches("0x"), 16) {
                    let v = match size {
                        1 => bus.read8(addr).unwrap_or(0) as u64,
                        2 => bus.read16(addr).unwrap_or(0) as u64,
                        4 => bus.read32(addr).unwrap_or(0) as u64,
                        _ => bus.read64(addr).unwrap_or(0),
                    };
                    println!("  [{:#010x}]: {:#0width$x}", addr, v, width = 2 + size * 2);
                } else { println!("{DIM}  bad address{RST}"); }
            }
            "csr" => {
                if let Ok(addr) = u64::from_str_radix(rest.trim_start_matches("0x"), 16) {
                    let v = cpu.csr.read(addr as usize, Privilege::M);
                    println!("  CSR[{:#06x}] = {:#018x}", addr, v);
                } else { println!("{DIM}  usage: csr <hex-addr>{RST}"); }
            }
            "reset" => { *cpu = Cpu::new(cpu.pc); println!("  CPU reset to pc={:#010x}", cpu.pc); }
            "h" | "help" => print_help(),
            "q" | "quit" => break,
            other if !other.is_empty() => println!("{DIM}  unknown '{other}'. try h for help{RST}"),
            _ => {}
        }
    }
}

fn vm_tick(cpu: &mut Cpu, bus: &mut Mmu) -> StepResult {
    if bus.clint.timer_pending() { cpu.csr.set_mip(MIP_MTIP); } else { cpu.csr.clear_mip(MIP_MTIP); }
    if cpu.csr.sstc_enabled() && bus.clint.mtime >= cpu.csr.stimecmp() { cpu.csr.set_mip(MIP_STIP); } else { cpu.csr.clear_mip(MIP_STIP); }
    if bus.has_pending_plic_s() { cpu.csr.set_mip(MIP_SEIP); } else { cpu.csr.clear_mip(MIP_SEIP); }
    if bus.clint.softint_pending() { cpu.csr.set_mip(MIP_SSIP); } else { cpu.csr.clear_mip(MIP_SSIP); }
    let result_step = cpu.step(bus);
    match result_step {
        StepResult::Trap(TrapCause::EcallFromM) => {
            if cpu.regs[17] == SYS_EXIT { return StepResult::Halted; }
            cpu.pc = cpu.pc.wrapping_add(4);
        }
        StepResult::Trap(TrapCause::EcallFromS) => {
            let a7 = cpu.regs[17]; let a0 = cpu.regs[10]; let a1 = cpu.regs[11]; let a2 = cpu.regs[12]; let a6 = cpu.regs[16];
            let res = sbi::handle_sbi(a7, a0, a1, a2, a6, bus);
            cpu.regs[10] = res.a0;
            if res.halt { return StepResult::Halted; }
            cpu.pc = cpu.pc.wrapping_add(4);
        }
        _ => {}
    }
    result_step
}

fn run_until(cpu: &mut Cpu, bus: &mut Mmu, bps: &HashSet<u64>, number: Option<u64>) -> StepResult {
    let max = number.unwrap_or(u64::MAX);
    for _ in 0..max {
        let r = vm_tick(cpu, bus);
        if !matches!(r, StepResult::Ok) { return r; }
        if bps.contains(&cpu.pc) { return StepResult::Ok; }
    }
    StepResult::Ok
}

fn drain_uart(bus: &mut Mmu) {
    let out = bus.uart.flush();
    if !out.is_empty() { print!("{out}"); }
}

fn run_headless(cpu: &mut Cpu, bus: &mut Mmu) {
    loop {
        match vm_tick(cpu, bus) {
            StepResult::Halted => { drain_uart(bus); let _ = io::stdout().flush();
                println!("halted  pc={:#010x}  a0={}  cycles={}", cpu.pc, cpu.regs[10] as i64, cpu.csr.read(CSR_CYCLE, Privilege::M)); break; }
            StepResult::Trap(t) if !matches!(t, TrapCause::EcallFromM | TrapCause::EcallFromS) => { drain_uart(bus); let _ = io::stdout().flush(); eprintln!("trap {:?}  pc={:#010x}", t, cpu.pc); break; }
            _ => { drain_uart(bus); }
        }
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
    println!("  {GRN}mem addr [n]{RST} dump n×32-bit words (default 8) at addr");
    println!("  {GRN}mem8/16/32/64 addr{RST}  read 1/2/4/8 bytes");
    println!("  {GRN}csr addr{RST}    read a CSR (hex address, e.g. 0x300 for mstatus)");
    println!("  {GRN}reset{RST}       reset CPU to initial state (preserves memory)");
    println!("  {GRN}h{RST}           this help");
    println!("  {GRN}q{RST}           quit");
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args: Vec<String> = env::args().collect();
    let debug = args.contains(&"--debug".to_string()) || args.contains(&"-d".to_string());
    let smode = args.contains(&"--sbi".to_string()) || args.contains(&"-s".to_string());
    let path = args.iter().skip(1).find(|a| !a.starts_with('-'));

    if smode {
        let (bin, entry) = match path {
            Some(p) => { let bytes = fs::read(p).unwrap_or_else(|e| { eprintln!("error: {e}"); std::process::exit(1); }); load_file(&bytes) }
            None => { println!("no kernel supplied — running built-in S-mode demo"); (Mmu::new_at(build_smode_demo(), KERNEL_BASE), KERNEL_BASE) }
        };
        let mut bus = bin;
        let mut cpu = Cpu::new(entry);
        let _ = boot_smode(&mut cpu, &mut bus, entry);
        if debug { run_debugger(&mut cpu, &mut bus, entry); } else { run_headless(&mut cpu, &mut bus); }
        return Ok(());
    }

    let demo_mcode: Vec<u8> = DEMO.iter().flat_map(|w| w.to_le_bytes()).collect();
    let (mut bus, entry) = match path {
        Some(p) => { let bytes = fs::read(p).unwrap_or_else(|e| { eprintln!("error: {e}"); std::process::exit(1); }); load_file(&bytes) }
        None => { println!("no binary supplied — running built-in demo: fib(10)"); (Mmu::new(&demo_mcode), DRAM_BASE) }
    };
    let mut cpu = Cpu::new(entry);
    if debug { run_debugger(&mut cpu, &mut bus, entry); } else { run_headless(&mut cpu, &mut bus); }
    Ok(())
}

fn load_file(bytes: &[u8]) -> (Mmu, u64) {
    load_binary(bytes)
}