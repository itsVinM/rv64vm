# rv64vm

Minimal **RV64IMA** RISC-V virtual machine in pure Rust, a freestanding **C++20** bare-metal
kernel, and Python host tooling. The VM boots S-mode kernels (xv6, Linux, `kernel/`) via an
in-hypervisor SBI, or plain M-mode binaries.

```
rv64vm/
├── src/            # Rust VM: CPU core, MMU, devices, debugger
├── kernel/         # Freestanding C++20 kernel (no assembly, no libc/libstdc++)
│   ├── include/    #   headers (.hpp + minimal freestanding <array>/<optional>/...)
│   ├── src/        #   sources (.cpp)
│   └── linker.ld   #   linked at 0x80200000
└── tools/          # Python host tooling (ELF inspection)
```

## Virtual machine

```sh
cargo run                                     # fib(10) demo
cargo run -- program.bin                      # M-mode binary
cargo run -- --sbi kernel/build/kernel.elf    # S-mode kernel (the C++20 kernel)
cargo run -- --sbi --debug kernel/build/kernel.elf
```

| Flag | Description |
|------|-------------|
| `--sbi` / `-s` | Boot S-mode with DTB + SBI |
| `--debug` / `-d` | Interactive debugger |
| `--max` / `--steps` `<n>` | Headless run capped at n instructions |

Boot contract for S-mode kernels: PC starts at `0x80200000` in S-mode with
`a0 = hartid`, `a1 = DTB physical address`.

### Debugger

Each step shows one line: `pc  raw_instruction_hex`

| Command | Description |
|---------|-------------|
| `s` | Step one instruction |
| `r [n]` | Run n steps (default 100) |
| `c` | Continue until breakpoint/halt |
| `b [addr]` | Toggle breakpoint (default PC) |
| `reg [name] [val]` | Read/write register |
| `mem addr [n]` | Dump n×32-bit words |
| `mem8/16/32/64 addr` | Read 1/2/4/8 bytes |
| `csr addr` | Read a CSR |
| `reset` | Reset CPU |
| `h` | Help |

### Memory map

| Region | Base | Size |
|--------|------|------|
| CLINT | `0x02000000` | 1 MB |
| PLIC | `0x0C000000` | 64 MB |
| UART | `0x10000000` | 256 B |
| VIRTIO | `0x10001000` | 4 KB |
| DTB | `0x86000000` | — |
| DRAM | `0x80000000` | 128 MB |

### SBI support

Legacy v0.1 (putchar, getchar, shutdown), Base, Timer, IPI, RFENCE, HSM, SRST, DBCN.

## Kernel (`kernel/`, C++20)

Freestanding RV64IMA_Zicsr kernel written in C++20 (`-fno-exceptions -fno-rtti`,
no runtime, no assembly files — inline asm only). On boot it:

1. Clears BSS, installs the stack canary and trap vector
2. Finds the UART through the device tree (`/chosen` → `stdout-path` → `reg`)
3. Enables Sv39 paging with a flat identity map (MMIO + DRAM gigapage leaves)
4. Runs self-tests (heap allocator, DRAM readback, string ops) over UART

Subsystems live under `namespace kernel::`: typed CSR access templates
(`arch::rv64vm`), Sv39 MMU driver (`rv64vm::Mmu`), first-fit heap allocator,
device-tree parser (`devicetree`), 8250 UART, printf-style formatting, panic,
trap dispatch, and SBI call wrappers.

Build (requires CMake ≥ 3.16 and a RISC-V gcc/g++ cross-toolchain,
e.g. `brew install riscv64-elf-gcc`):

```sh
cd kernel
cmake -B build        # configure — auto-detects the cross g++
cmake --build build   # produces build/kernel.elf
```

## Host tooling (`tools/`, Python 3)

```sh
python3 tools/elfdump.py kernel/build/kernel.elf           # dump header/phdrs/sections
python3 tools/elfdump.py --check kernel/build/kernel.elf   # + verify it fits VM DRAM
python3 -m unittest discover -s tools                      # unit tests
```

`tools/rv64elf.py` is a small stdlib-only ELF64 reader used by `elfdump`.
