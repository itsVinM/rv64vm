// Minimal Flattened Device Tree generator — builds a valid DTB blob in memory.

#[cfg(not(feature = "std"))]
use alloc::vec::Vec;

const FDT_MAGIC: u32 = 0xD00DFEED;
const FDT_VERSION: u32 = 17;
const FDT_COMPAT: u32 = 16;

// FDT tokens
const FDT_BEGIN_NODE: u32 = 0x00000001;
const FDT_END_NODE: u32 = 0x00000002;
const FDT_PROP: u32 = 0x00000003;
const FDT_END: u32 = 0x00000009;

fn u32_to_be(v: u32) -> [u8; 4] {
    v.to_be_bytes()
}
fn u64_to_be(v: u64) -> [u8; 8] {
    v.to_be_bytes()
}

struct FdtWriter {
    buf: Vec<u8>,
    str_off: u32,
    strtab: Vec<u8>,
}

impl FdtWriter {
    fn new() -> Self {
        Self {
            buf: Vec::new(),
            str_off: 0,
            strtab: Vec::new(),
        }
    }

    fn u32(&mut self, v: u32) {
        self.buf.extend_from_slice(&u32_to_be(v));
    }
    fn bytes(&mut self, b: &[u8]) {
        self.buf.extend_from_slice(b);
    }

    fn align4(&mut self) {
        while !self.buf.len().is_multiple_of(4) {
            self.buf.push(0);
        }
    }

    fn add_string(&mut self, s: &str) -> u32 {
        let off = self.str_off;
        self.strtab.extend_from_slice(s.as_bytes());
        self.strtab.push(0);
        self.str_off += s.len() as u32 + 1;
        off
    }

    fn begin_node(&mut self, name: &str) {
        self.u32(FDT_BEGIN_NODE);
        self.bytes(name.as_bytes());
        self.bytes(&[0]);
        self.align4();
    }

    fn end_node(&mut self) {
        self.u32(FDT_END_NODE);
    }

    fn prop(&mut self, name: &str, val: &[u8]) {
        let name_off = self.add_string(name);
        self.u32(FDT_PROP);
        self.u32(val.len() as u32);
        self.u32(name_off);
        self.bytes(val);
        self.align4();
    }

    fn prop_u32(&mut self, name: &str, v: u32) {
        self.prop(name, &u32_to_be(v));
    }

    fn prop_cells(&mut self, name: &str, vals: &[u32]) {
        let mut data = Vec::new();
        for &v in vals {
            data.extend_from_slice(&u32_to_be(v));
        }
        self.prop(name, &data);
    }

    fn prop_str(&mut self, name: &str, val: &str) {
        self.prop(name, val.as_bytes());
    }

    fn prop_empty(&mut self, name: &str) {
        self.prop(name, &[]);
    }

    fn finish(mut self) -> Vec<u8> {
        self.u32(FDT_END);

        let struct_len = self.buf.len() as u32;
        let strtab_len = self.strtab.len() as u32;
        let total = 40 + 16 + struct_len + strtab_len;

        // Build header
        let mut hdr = Vec::with_capacity(total as usize);
        hdr.extend_from_slice(&u32_to_be(FDT_MAGIC));
        hdr.extend_from_slice(&u32_to_be(total));
        hdr.extend_from_slice(&u32_to_be(40 + 16)); // off_dt_struct
        hdr.extend_from_slice(&u32_to_be(40 + 16 + struct_len)); // off_dt_strings
        hdr.extend_from_slice(&u32_to_be(40)); // off_mem_rsvmap
        hdr.extend_from_slice(&u32_to_be(FDT_VERSION));
        hdr.extend_from_slice(&u32_to_be(FDT_COMPAT));
        hdr.extend_from_slice(&u32_to_be(0)); // boot_cpuid_phys
        hdr.extend_from_slice(&u32_to_be(strtab_len));
        hdr.extend_from_slice(&u32_to_be(struct_len));

        // Memory reservation block (empty)
        hdr.extend_from_slice(&u64_to_be(0));
        hdr.extend_from_slice(&u64_to_be(0));

        hdr.extend_from_slice(&self.buf);
        hdr.extend_from_slice(&self.strtab);

        hdr
    }
}

pub fn generate_dtb() -> Vec<u8> {
    let mut f = FdtWriter::new();

    // Root node
    f.begin_node("");
    f.prop_cells("#address-cells", &[2]);
    f.prop_cells("#size-cells", &[2]);
    f.prop_str("compatible", "riscv-virtio");
    f.prop_str("model", "riscv-vm");

    // Chosen
    f.begin_node("chosen");
    f.prop_str(
        "bootargs",
        "console=ttyS0 earlycon=uart8250,mmio,0x10000000 root=/dev/vda1 rw",
    );
    f.prop_str("stdout-path", "/soc/uart@10000000");
    f.end_node();

    // Memory
    f.begin_node("memory@80000000");
    f.prop_str("device_type", "memory");
    f.prop_cells("reg", &[0, 0x80000000, 0, 0x08000000]); // 128MB
    f.end_node();

    // Cpus
    f.begin_node("cpus");
    f.prop_cells("#address-cells", &[1]);
    f.prop_cells("#size-cells", &[0]);
    f.prop_u32("timebase-frequency", 10_000_000);

    f.begin_node("cpu@0");
    f.prop_str("device_type", "cpu");
    f.prop_u32("reg", 0);
    f.prop_str("compatible", "riscv");
    f.prop_str("riscv,isa", "rv64imafd");
    f.prop_str("mmu-type", "riscv,sv39");
    f.prop_u32("clock-frequency", 100_000_000);
    f.prop_str("status", "okay");

    f.begin_node("interrupt-controller");
    f.prop_u32("phandle", 1);
    f.prop_cells("#interrupt-cells", &[1]);
    f.prop_empty("interrupt-controller");
    f.prop_str("compatible", "riscv,cpu-intc");
    f.end_node(); // interrupt-controller
    f.end_node(); // cpu@0
    f.end_node(); // cpus

    // Soc
    f.begin_node("soc");
    f.prop_cells("#address-cells", &[2]);
    f.prop_cells("#size-cells", &[2]);
    f.prop_str("compatible", "simple-bus");
    f.prop_empty("ranges");

    // CLINT
    f.begin_node("clint@2000000");
    f.prop_str("compatible", "riscv,clint0");
    f.prop_cells("reg", &[0, 0x2000000, 0, 0x10000]);
    f.prop_cells("interrupts-extended", &[0x01, 3, 0x01, 7]);
    f.end_node();

    // PLIC
    f.begin_node("plic@c000000");
    f.prop_u32("phandle", 2);
    f.prop_str("compatible", "riscv,plic0");
    f.prop_cells("reg", &[0, 0xC000000, 0, 0x4000000]);
    f.prop_cells("interrupts-extended", &[0x01, 11, 0x01, 9]);
    f.prop_u32("riscv,ndev", 53);
    f.end_node();

    // UART 16550
    f.begin_node("uart@10000000");
    f.prop_str("compatible", "ns16550a");
    f.prop_cells("reg", &[0, 0x10000000, 0, 0x100]);
    f.prop_u32("clock-frequency", 0x384000);
    f.prop_u32("interrupt-parent", 0x02);
    f.prop_u32("interrupts", 10);
    f.end_node();

    f.end_node(); // soc
    f.end_node(); // root

    f.finish()
}