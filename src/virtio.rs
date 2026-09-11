use crate::mmu::DRAM_BASE;

#[cfg(not(feature = "std"))]
use alloc::{vec, vec::Vec};

const MAGIC: u64 = 0x000;
const VERSION: u64 = 0x004;
const DEVICE_ID: u64 = 0x008;
const VENDOR_ID: u64 = 0x00C;
const DEVICE_FEATURES: u64 = 0x010;
const DEVICE_FEATURES_SEL: u64 = 0x014;
const DRIVER_FEATURES: u64 = 0x020;
const DRIVER_FEATURES_SEL: u64 = 0x024;
const QUEUE_NUM_MAX: u64 = 0x034;
const QUEUE_NUM: u64 = 0x038;
const QUEUE_READY: u64 = 0x044;
const INTERRUPT_STATUS: u64 = 0x060;
const INTERRUPT_ACK: u64 = 0x064;
const STATUS: u64 = 0x070;
const QUEUE_DESC_LOW: u64 = 0x080;
const QUEUE_DESC_HIGH: u64 = 0x084;
const QUEUE_DRIVER_LOW: u64 = 0x090;
const QUEUE_DRIVER_HIGH: u64 = 0x094;
const QUEUE_DEVICE_LOW: u64 = 0x0A0;
const QUEUE_DEVICE_HIGH: u64 = 0x0A4;
const CONFIG: u64 = 0x100;

const VIRTIO_F_VERSION_1: u64 = 1 << 32;
const VRING_DESC_F_NEXT: u16 = 1;
const MAX_DESCRIPTOR_CHAIN: usize = 16;

pub struct Virtio {
    device_features_select: u32,
    driver_features_select: u32,
    driver_features: u64,
    queue_number: u32,
    queue_ready: u32,
    status: u32,
    interrupt_status: u32,
    descriptor_table_addr: u64,
    available_ring_addr: u64,
    used_ring_addr: u64,
    last_available_index: u16,
    pub disk: Vec<u8>,
    pub irq_pending: bool,
}

impl Virtio {
    pub fn new() -> Self {
        Self {
            device_features_select: 0,
            driver_features_select: 0,
            driver_features: 0,
            queue_number: 0,
            queue_ready: 0,
            status: 0,
            interrupt_status: 0,
            descriptor_table_addr: 0,
            available_ring_addr: 0,
            used_ring_addr: 0,
            last_available_index: 0,
            disk: vec![0; 64 * 1024 * 1024],
            irq_pending: false,
        }
    }

    pub fn read32(&self, offset: u64) -> u32 {
        match offset {
            MAGIC => 0x74726976,
            VERSION => 2,
            DEVICE_ID => 2,
            VENDOR_ID => 0x554d4551,
            DEVICE_FEATURES => match self.device_features_select {
                0 => VIRTIO_F_VERSION_1 as u32,
                1 => (VIRTIO_F_VERSION_1 >> 32) as u32,
                _ => 0,
            },
            QUEUE_NUM_MAX => 8,
            QUEUE_READY => self.queue_ready,
            INTERRUPT_STATUS => self.interrupt_status,
            STATUS => self.status,
            CONFIG => (self.disk.len() / 512) as u32,
            x if x == CONFIG + 4 => ((self.disk.len() / 512) >> 32) as u32,
            _ => 0,
        }
    }

    pub fn write32(&mut self, offset: u64, value: u32) {
        match offset {
            DEVICE_FEATURES_SEL => self.device_features_select = value,
            DRIVER_FEATURES_SEL => self.driver_features_select = value,
            DRIVER_FEATURES => {
                self.driver_features = if self.driver_features_select == 0 {
                    (self.driver_features & 0xFFFF_FFFF_0000_0000) | value as u64
                } else {
                    (self.driver_features & 0x0000_0000_FFFF_FFFF) | ((value as u64) << 32)
                };
            }
            QUEUE_NUM => self.queue_number = value,
            QUEUE_READY => self.queue_ready = value,
            INTERRUPT_ACK => self.interrupt_status &= !value,
            STATUS => self.status = value,
            QUEUE_DESC_LOW => {
                self.descriptor_table_addr = set_low(self.descriptor_table_addr, value)
            }
            QUEUE_DESC_HIGH => {
                self.descriptor_table_addr = set_high(self.descriptor_table_addr, value)
            }
            QUEUE_DRIVER_LOW => self.available_ring_addr = set_low(self.available_ring_addr, value),
            QUEUE_DRIVER_HIGH => {
                self.available_ring_addr = set_high(self.available_ring_addr, value)
            }
            QUEUE_DEVICE_LOW => self.used_ring_addr = set_low(self.used_ring_addr, value),
            QUEUE_DEVICE_HIGH => self.used_ring_addr = set_high(self.used_ring_addr, value),
            _ => {}
        }
    }

    pub fn handle_notify(&mut self, dram: &mut [u8]) {
        if self.queue_ready == 0
            || self.queue_number == 0
            || self.descriptor_table_addr == 0
            || self.available_ring_addr == 0
            || self.used_ring_addr == 0
        {
            return;
        }

        let queue_size = self.queue_number.min(8) as usize;
        let available_ring_offset = pa_to_off(self.available_ring_addr);
        let available_index = read_u16(dram, available_ring_offset + 2);

        while self.last_available_index != available_index {
            let slot = (self.last_available_index as usize) % queue_size;
            let descriptor_head = read_u16(dram, available_ring_offset + 4 + slot * 2);
            self.process_chain(descriptor_head, dram);
            self.last_available_index = self.last_available_index.wrapping_add(1);
        }

        self.irq_pending = true;
        self.interrupt_status |= 1;
    }

    fn process_chain(&mut self, descriptor_head: u16, dram: &mut [u8]) {
        let mut next_descriptor_index = descriptor_head;
        let mut descriptors = [(0u64, 0u32); 3];
        let mut descriptor_count = 0usize;

        for _ in 0..MAX_DESCRIPTOR_CHAIN {
            let descriptor_offset =
                pa_to_off(self.descriptor_table_addr + next_descriptor_index as u64 * 16);
            descriptors[descriptor_count] = (
                read_u64(dram, descriptor_offset),
                read_u32(dram, descriptor_offset + 8),
            );
            let flags = read_u16(dram, descriptor_offset + 12);
            next_descriptor_index = read_u16(dram, descriptor_offset + 14);
            descriptor_count += 1;

            if flags & VRING_DESC_F_NEXT == 0 || descriptor_count == 3 {
                break;
            }
        }

        if descriptor_count < 2 {
            return;
        }

        let request_header_offset = pa_to_off(descriptors[0].0);
        let request_type = read_u32(dram, request_header_offset);
        let sector_number = read_u64(dram, request_header_offset + 8) as usize;
        let disk_offset = sector_number.saturating_mul(512);

        if disk_offset >= self.disk.len() {
            self.push_used(dram, descriptor_head, 0);
            return;
        }

        let data_buffer_offset = pa_to_off(descriptors[1].0);
        let transfer_length = (descriptors[1].1 as usize).min(self.disk.len() - disk_offset);

        if request_type == 0 {
            dram[data_buffer_offset..data_buffer_offset + transfer_length]
                .copy_from_slice(&self.disk[disk_offset..disk_offset + transfer_length]);
        } else {
            self.disk[disk_offset..disk_offset + transfer_length]
                .copy_from_slice(&dram[data_buffer_offset..data_buffer_offset + transfer_length]);
        }

        if descriptor_count == 3 {
            let status_offset = pa_to_off(descriptors[2].0);
            if status_offset < dram.len() {
                dram[status_offset] = 0;
            }
        }

        self.push_used(dram, descriptor_head, transfer_length as u32);
    }

    fn push_used(&mut self, dram: &mut [u8], descriptor_head: u16, transfer_length: u32) {
        let used_ring_offset = pa_to_off(self.used_ring_addr);
        let used_index = read_u16(dram, used_ring_offset + 2);
        let slot = (used_index as usize) % self.queue_number.min(8) as usize;
        let entry_offset = used_ring_offset + 4 + slot * 8;

        write_u32(dram, entry_offset, descriptor_head as u32);
        write_u32(dram, entry_offset + 4, transfer_length);
        write_u16(dram, used_ring_offset + 2, used_index.wrapping_add(1));
    }
}

fn pa_to_off(physical_address: u64) -> usize {
    (physical_address - DRAM_BASE) as usize
}

fn set_low(value: u64, low: u32) -> u64 {
    (value & 0xFFFF_FFFF_0000_0000) | low as u64
}

fn set_high(value: u64, high: u32) -> u64 {
    (value & 0x0000_0000_FFFF_FFFF) | ((high as u64) << 32)
}

fn read_u16(dram: &[u8], offset: usize) -> u16 {
    u16::from_le_bytes(dram[offset..offset + 2].try_into().unwrap())
}

fn read_u32(dram: &[u8], offset: usize) -> u32 {
    u32::from_le_bytes(dram[offset..offset + 4].try_into().unwrap())
}

fn read_u64(dram: &[u8], offset: usize) -> u64 {
    u64::from_le_bytes(dram[offset..offset + 8].try_into().unwrap())
}

fn write_u16(dram: &mut [u8], offset: usize, value: u16) {
    dram[offset..offset + 2].copy_from_slice(&value.to_le_bytes());
}

fn write_u32(dram: &mut [u8], offset: usize, value: u32) {
    dram[offset..offset + 4].copy_from_slice(&value.to_le_bytes());
}