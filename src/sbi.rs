use crate::uart;
use crate::Bus;

// SBI extension IDs (legacy)
pub const SBI_SET_TIMER: u64 = 0;
pub const SBI_CONSOLE_PUTCHAR: u64 = 1;
pub const SBI_CONSOLE_GETCHAR: u64 = 2;
pub const SBI_CLEAR_IPI: u64 = 3;
pub const SBI_SEND_IPI: u64 = 4;
pub const SBI_REMOTE_FENCE_I: u64 = 5;
pub const SBI_REMOTE_SFENCE_VMA: u64 = 6;
pub const SBI_REMOTE_SFENCE_VMA_ASID: u64 = 7;
pub const SBI_SHUTDOWN: u64 = 8;

// SBI v0.2+ extension IDs
pub const SBI_EXT_BASE: u64 = 0x10;
pub const SBI_EXT_TIME: u64 = 0x54494D45;
pub const SBI_EXT_IPI: u64 = 0x735049;
pub const SBI_EXT_RFENCE: u64 = 0x52464E43;
pub const SBI_EXT_HSM: u64 = 0x48534D;
pub const SBI_EXT_SRST: u64 = 0x53525354;
pub const SBI_EXT_DBCN: u64 = 0x4442434E;

// SBI v0.2 function IDs
pub const SBI_BASE_GET_SPEC_VERSION: u64 = 0;
pub const SBI_BASE_GET_IMP_ID: u64 = 1;
pub const SBI_BASE_GET_IMP_VERSION: u64 = 2;
pub const SBI_BASE_PROBE_EXT: u64 = 3;
pub const SBI_BASE_GET_MVENDORID: u64 = 4;
pub const SBI_BASE_GET_MARCHID: u64 = 5;
pub const SBI_BASE_GET_MIMPID: u64 = 6;
pub const SBI_TIME_SET_TIMER: u64 = 0;
pub const SBI_HSM_HART_STOP: u64 = 1;
pub const SBI_SRST_SYSTEM_RESET: u64 = 0;

// DBCN function IDs
pub const SBI_DBCN_CONSOLE_WRITE: u64 = 0;
pub const SBI_DBCN_CONSOLE_READ: u64 = 1;
pub const SBI_DBCN_CONSOLE_WRITE_BYTE: u64 = 2;

// SBI return values
pub const SBI_SUCCESS: u64 = 0;
pub const SBI_ERR_NOT_SUPPORTED: u64 = -2i64 as u64;

#[derive(Debug, Clone, Copy)]
pub struct SbiResult {
    pub a0: u64,
    pub halt: bool,
}

impl SbiResult {
    pub const fn ok() -> Self {
        Self {
            a0: SBI_SUCCESS,
            halt: false,
        }
    }
    pub const fn value(v: u64) -> Self {
        Self { a0: v, halt: false }
    }
    pub const fn not_supported() -> Self {
        Self {
            a0: SBI_ERR_NOT_SUPPORTED,
            halt: false,
        }
    }
    pub const fn halt() -> Self {
        Self { a0: 0, halt: true }
    }
}

pub fn handle_sbi<B: Bus>(
    a7: u64,
    a0: u64,
    _a1: u64,
    _a2: u64,
    a6: u64,
    bus: &mut B,
) -> SbiResult {
    match a7 {
        // Legacy extensions
        SBI_SET_TIMER => {
            bus.set_mtimecmp(a0);
            SbiResult::ok()
        }
        SBI_CONSOLE_PUTCHAR => {
            let _ = bus.write8(uart::UART_BASE, a0 as u8);
            SbiResult::ok()
        }
        SBI_CONSOLE_GETCHAR
        | SBI_CLEAR_IPI
        | SBI_SEND_IPI
        | SBI_REMOTE_FENCE_I
        | SBI_REMOTE_SFENCE_VMA
        | SBI_REMOTE_SFENCE_VMA_ASID => SbiResult::ok(),
        SBI_SHUTDOWN => SbiResult::halt(),
        // v0.2 extensions
        SBI_EXT_BASE => handle_sbi_base(a6, a0),
        SBI_EXT_TIME => match a6 {
            SBI_TIME_SET_TIMER => {
                bus.set_mtimecmp(a0);
                SbiResult::ok()
            }
            _ => SbiResult::not_supported(),
        },
        SBI_EXT_IPI | SBI_EXT_RFENCE => SbiResult::ok(),
        SBI_EXT_HSM | SBI_EXT_SRST => match a6 {
            SBI_HSM_HART_STOP | SBI_SRST_SYSTEM_RESET => SbiResult::halt(),
            _ => SbiResult::not_supported(),
        },
        SBI_EXT_DBCN => handle_sbi_dbcn(a6, a0, _a1, _a2, bus),
        _ => SbiResult::not_supported(),
    }
}

fn handle_sbi_dbcn<B: Bus>(
    func_id: u64,
    buf_addr: u64,
    buf_len: u64,
    _out_len_addr: u64,
    bus: &mut B,
) -> SbiResult {
    match func_id {
        SBI_DBCN_CONSOLE_WRITE_BYTE => {
            let _ = bus.write8(uart::UART_BASE, buf_addr as u8);
            SbiResult::ok()
        }
        SBI_DBCN_CONSOLE_WRITE => {
            let mut written: u64 = 0;
            for i in 0..buf_len {
                let addr = buf_addr.wrapping_add(i);
                if let Ok(byte) = bus.read8(addr) {
                    let _ = bus.write8(uart::UART_BASE, byte);
                    written = i + 1;
                } else {
                    break;
                }
            }
            SbiResult::value(written)
        }
        SBI_DBCN_CONSOLE_READ => SbiResult::value(0), // no input available
        _ => SbiResult::not_supported(),
    }
}

fn handle_sbi_base(func_id: u64, ext_id: u64) -> SbiResult {
    match func_id {
        SBI_BASE_GET_SPEC_VERSION => SbiResult::value(2),
        SBI_BASE_GET_IMP_ID => SbiResult::value(1),
        SBI_BASE_GET_IMP_VERSION => SbiResult::value(1),
        SBI_BASE_PROBE_EXT => SbiResult::value(matches!(
            ext_id,
            SBI_EXT_BASE
                | SBI_EXT_TIME
                | SBI_EXT_IPI
                | SBI_EXT_RFENCE
                | SBI_EXT_HSM
                | SBI_EXT_SRST
                | SBI_EXT_DBCN
        ) as u64),
        SBI_BASE_GET_MVENDORID => SbiResult::value(0),
        SBI_BASE_GET_MARCHID => SbiResult::value(0),
        SBI_BASE_GET_MIMPID => SbiResult::value(0),
        _ => SbiResult::not_supported(),
    }
}
