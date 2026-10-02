//! An in-memory [`SerialPort`] that answers Protocol 2.0 status packets, for tests.
//!
//! `DxlBus` hands its port to `rustypot`, which owns the wire format, so the only way to
//! exercise this crate without hardware is to speak that format back. The fake keeps no
//! model of a servo: each `write` (one instruction packet) pops the next scripted reply,
//! and [`status`] builds a well-formed status packet with a real CRC so `rustypot` accepts
//! it. What each test asserts is therefore the pair that is actually ours — the instruction
//! we emit and the conversion we apply to the bytes that come back.

use alloc::collections::VecDeque;
use alloc::sync::Arc;
use core::cell::RefCell;
use core::time::Duration;
use serialport::{ClearBuffer, DataBits, FlowControl, Parity, SerialPort, StopBits};
use std::io::{self, Read, Write};
use std::sync::Mutex;

/// CRC-16/BUYPASS (poly 0x8005, init 0, MSB first) — the checksum Protocol 2.0 uses.
pub(crate) fn crc16(data: &[u8]) -> u16 {
    let mut crc: u16 = 0;
    for &byte in data {
        crc ^= u16::from(byte) << 8;
        for _ in 0..8 {
            crc = if crc & 0x8000 == 0 {
                crc << 1
            } else {
                (crc << 1) ^ 0x8005
            };
        }
    }
    crc
}

/// Build one status packet: `FF FF FD 00 ID LEN_L LEN_H 0x55 ERR PARAMS.. CRC_L CRC_H`.
///
/// `LEN` counts everything after itself, i.e. the 0x55 marker + the error byte + the
/// parameters + the two CRC bytes.
#[allow(clippy::cast_possible_truncation)]
pub(crate) fn status(id: u8, params: &[u8]) -> Vec<u8> {
    let mut p = vec![0xFF, 0xFF, 0xFD, 0x00, id];
    // Test payloads are a handful of bytes; the field is 16 bits.
    let len = (params.len() + 4) as u16;
    p.extend_from_slice(&len.to_le_bytes());
    p.push(0x55); // status packet marker
    p.push(0x00); // no error
    p.extend_from_slice(params);
    let crc = crc16(&p);
    p.extend_from_slice(&crc.to_le_bytes());
    p
}

/// A `ping` answer carries model number (2 bytes) + firmware version (1 byte).
pub(crate) fn ping_status(id: u8) -> Vec<u8> {
    status(id, &[0x64, 0x04, 46])
}

/// See the module docs.
#[derive(Debug)]
pub(crate) struct FakeBus {
    rx: RefCell<VecDeque<u8>>,
    replies: RefCell<VecDeque<Vec<u8>>>,
    written: Arc<Mutex<Vec<Vec<u8>>>>,
}

impl FakeBus {
    /// `replies` are handed out one per instruction packet, in order. Running past the end
    /// leaves the bus silent, which `rustypot` reports as a timeout.
    pub(crate) fn new(replies: Vec<Vec<u8>>) -> (Self, Arc<Mutex<Vec<Vec<u8>>>>) {
        let written = Arc::new(Mutex::new(Vec::new()));
        let bus = Self {
            rx: RefCell::new(VecDeque::new()),
            replies: RefCell::new(replies.into_iter().collect()),
            written: Arc::clone(&written),
        };
        (bus, written)
    }
}

impl Read for FakeBus {
    fn read(&mut self, buf: &mut [u8]) -> io::Result<usize> {
        let mut rx = self.rx.borrow_mut();
        if rx.is_empty() {
            return Err(io::Error::new(io::ErrorKind::TimedOut, "fake bus is idle"));
        }
        let n = buf.len().min(rx.len());
        for slot in buf.iter_mut().take(n) {
            *slot = rx.pop_front().unwrap_or(0);
        }
        Ok(n)
    }
}

impl Write for FakeBus {
    fn write(&mut self, buf: &[u8]) -> io::Result<usize> {
        self.written
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .push(buf.to_vec());
        if let Some(reply) = self.replies.borrow_mut().pop_front() {
            self.rx.borrow_mut().extend(reply);
        }
        Ok(buf.len())
    }

    fn flush(&mut self) -> io::Result<()> {
        Ok(())
    }
}

/// Settings plumbing `rustypot` never reads back; it only has to exist.
impl SerialPort for FakeBus {
    fn name(&self) -> Option<String> {
        Some("fake".to_string())
    }
    fn baud_rate(&self) -> serialport::Result<u32> {
        Ok(1_000_000)
    }
    fn data_bits(&self) -> serialport::Result<DataBits> {
        Ok(DataBits::Eight)
    }
    fn flow_control(&self) -> serialport::Result<FlowControl> {
        Ok(FlowControl::None)
    }
    fn parity(&self) -> serialport::Result<Parity> {
        Ok(Parity::None)
    }
    fn stop_bits(&self) -> serialport::Result<StopBits> {
        Ok(StopBits::One)
    }
    fn timeout(&self) -> Duration {
        Duration::from_millis(10)
    }
    fn set_baud_rate(&mut self, _: u32) -> serialport::Result<()> {
        Ok(())
    }
    fn set_data_bits(&mut self, _: DataBits) -> serialport::Result<()> {
        Ok(())
    }
    fn set_flow_control(&mut self, _: FlowControl) -> serialport::Result<()> {
        Ok(())
    }
    fn set_parity(&mut self, _: Parity) -> serialport::Result<()> {
        Ok(())
    }
    fn set_stop_bits(&mut self, _: StopBits) -> serialport::Result<()> {
        Ok(())
    }
    fn set_timeout(&mut self, _: Duration) -> serialport::Result<()> {
        Ok(())
    }
    fn write_request_to_send(&mut self, _: bool) -> serialport::Result<()> {
        Ok(())
    }
    fn write_data_terminal_ready(&mut self, _: bool) -> serialport::Result<()> {
        Ok(())
    }
    fn read_clear_to_send(&mut self) -> serialport::Result<bool> {
        Ok(false)
    }
    fn read_data_set_ready(&mut self) -> serialport::Result<bool> {
        Ok(false)
    }
    fn read_ring_indicator(&mut self) -> serialport::Result<bool> {
        Ok(false)
    }
    fn read_carrier_detect(&mut self) -> serialport::Result<bool> {
        Ok(false)
    }
    fn bytes_to_read(&self) -> serialport::Result<u32> {
        Ok(u32::try_from(self.rx.borrow().len()).unwrap_or(u32::MAX))
    }
    fn bytes_to_write(&self) -> serialport::Result<u32> {
        Ok(0)
    }
    fn clear(&self, buffer_to_clear: ClearBuffer) -> serialport::Result<()> {
        if matches!(buffer_to_clear, ClearBuffer::Input | ClearBuffer::All) {
            self.rx.borrow_mut().clear();
        }
        Ok(())
    }
    fn try_clone(&self) -> serialport::Result<Box<dyn SerialPort>> {
        Err(serialport::Error::new(
            serialport::ErrorKind::Unknown,
            "the fake bus is not clonable",
        ))
    }
    fn set_break(&self) -> serialport::Result<()> {
        Ok(())
    }
    fn clear_break(&self) -> serialport::Result<()> {
        Ok(())
    }
}
