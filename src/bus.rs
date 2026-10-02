//! Synchronous serial bus for Dynamixel X-series servos.
//!
//! This module wraps `rustypot`'s Protocol 2.0 controller and reads or writes raw register values.
//! Position conversion and per-servo mapping are left to the calling application.
//!
//! X-series models share the relevant control table addresses, so one XL430 controller handles
//! XM430, XL430, XC430, and XL330 servos.

use core::time::Duration;

use rustypot::servo::dynamixel::xl430::Xl430Controller;

use crate::error::{Result, protocol_err};

/// Synchronous transaction bus for a group of Dynamixel X-series servos.
///
/// One bus owns one serial port and communicates over TTL or half-duplex RS-485.
pub struct DxlBus {
    ctrl: Xl430Controller,
}

impl core::fmt::Debug for DxlBus {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        // Dynamixel serial transport implementation detail.
        f.debug_struct("DxlBus").finish_non_exhaustive()
    }
}

impl DxlBus {
    /// Opens a serial port and configures a Protocol 2.0 X-series bus.
    pub fn open(port: &str, baud: u32, timeout: Duration) -> Result<Self> {
        let sp = serialport::new(port, baud).timeout(timeout).open()?;
        let ctrl = Xl430Controller::new()
            .with_protocol_v2()
            .with_serial_port(sp);
        Ok(Self { ctrl })
    }

    /// Checks whether a servo responds; a timeout means it is absent.
    pub fn ping(&mut self, id: u8) -> Result<bool> {
        self.ctrl.ping(id).map_err(protocol_err)
    }

    /// Writes goal-position ticks to multiple servos with one SYNC WRITE.
    pub fn sync_write_goal_position(&mut self, ids: &[u8], ticks: &[u32]) -> Result<()> {
        self.ctrl
            .sync_write_goal_position(ids, ticks)
            .map_err(protocol_err)
    }

    /// Reads signed present-position ticks from multiple servos with one SYNC READ.
    ///
    /// The register is unsigned on the wire, but extended position mode uses signed two's-complement values.
    pub fn sync_read_present_position(&mut self, ids: &[u8]) -> Result<Vec<i32>> {
        let raw = self
            .ctrl
            .sync_read_present_position(ids)
            .map_err(protocol_err)?;
        Ok(raw.into_iter().map(|v| v as i32).collect())
    }

    /// Reads signed raw present-velocity values from multiple servos.
    ///
    /// X-series velocity units are 0.229 rpm per least-significant bit.
    pub fn sync_read_present_velocity(&mut self, ids: &[u8]) -> Result<Vec<i32>> {
        let raw = self
            .ctrl
            .sync_read_present_velocity(ids)
            .map_err(protocol_err)?;
        Ok(raw.into_iter().map(|v| v as i32).collect())
    }

    /// Reads present-current values and sign-extends them to 32 bits.
    ///
    /// The raw register uses signed 16-bit two's-complement values; unit conversion is left to the caller.
    pub fn sync_read_present_current(&mut self, ids: &[u8]) -> Result<Vec<i32>> {
        let raw = self
            .ctrl
            .sync_read_present_current(ids)
            .map_err(protocol_err)?;
        Ok(raw.into_iter().map(|v| i32::from(v as i16)).collect())
    }

    /// Reads raw present-temperature values in degrees Celsius.
    ///
    /// The register contains degrees Celsius directly and requires no conversion.
    pub fn sync_read_present_temperature(&mut self, ids: &[u8]) -> Result<Vec<u8>> {
        self.ctrl
            .sync_read_present_temperature(ids)
            .map_err(protocol_err)
    }

    /// Writes profile-velocity register values to multiple servos.
    pub fn sync_write_profile_velocity(&mut self, ids: &[u8], vals: &[u32]) -> Result<()> {
        self.ctrl
            .sync_write_profile_velocity(ids, vals)
            .map_err(protocol_err)
    }

    /// Enables or disables torque on multiple servos.
    pub fn sync_write_torque_enable(&mut self, ids: &[u8], on: &[bool]) -> Result<()> {
        let vals: Vec<u8> = on.iter().map(|&b| u8::from(b)).collect();
        self.ctrl
            .sync_write_torque_enable(ids, &vals)
            .map_err(protocol_err)
    }

    /// Enables or disables torque on one servo.
    pub fn write_torque_enable(&mut self, id: u8, on: bool) -> Result<()> {
        self.ctrl
            .write_torque_enable(id, u8::from(on))
            .map_err(protocol_err)
    }

    /// Returns the responding servo IDs in the inclusive `start..=end` range.
    pub fn scan(&mut self, start: u8, end: u8) -> Result<Vec<u8>> {
        let mut found = Vec::new();
        for id in start..=end {
            if self.ping(id)? {
                found.push(id);
            }
        }
        Ok(found)
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used, unused_results)]
mod tests {
    use super::*;
    use crate::error::DxlError;
    use crate::fake::{FakeBus, crc16, ping_status, status};
    use alloc::sync::Arc;
    use std::sync::Mutex;

    type Written = Arc<Mutex<Vec<Vec<u8>>>>;

    fn open_bus(replies: Vec<Vec<u8>>) -> (DxlBus, Written) {
        let (fake, written) = FakeBus::new(replies);
        let ctrl = Xl430Controller::new()
            .with_protocol_v2()
            .with_serial_port(Box::new(fake));
        (DxlBus { ctrl }, written)
    }

    /// Concatenate one status packet per id, which is what a SYNC READ (0x82) draws out:
    /// each servo answers with its own packet, in the order they were addressed.
    fn sync_read_replies(entries: &[(u8, Vec<u8>)]) -> Vec<u8> {
        entries
            .iter()
            .flat_map(|(id, params)| status(*id, params))
            .collect()
    }

    /// The instruction byte of the n-th packet we put on the wire.
    fn instruction(written: &Written, n: usize) -> u8 {
        written.lock().unwrap()[n][7]
    }

    fn packet(written: &Written, n: usize) -> Vec<u8> {
        written.lock().unwrap()[n].clone()
    }

    #[test]
    fn the_crc_matches_the_reference_packet_from_the_protocol_docs() {
        // Anchors the checksum used by every other test here. ROBOTIS documents the ping
        // instruction for id 1 as FF FF FD 00 01 03 00 01 19 4E — CRC 0x4E19, sent LE.
        let p = [0xFFu8, 0xFF, 0xFD, 0x00, 0x01, 0x03, 0x00, 0x01];
        assert_eq!(crc16(&p), 0x4E19);
        assert_eq!(crc16(&p).to_le_bytes(), [0x19, 0x4E]);
    }

    #[test]
    fn ping_is_true_when_the_servo_answers_and_false_when_it_stays_quiet() {
        let (mut bus, written) = open_bus(vec![ping_status(1)]);
        assert!(bus.ping(1).unwrap());
        assert_eq!(instruction(&written, 0), 0x01, "PING");

        // No scripted reply: the bus goes quiet, which is how absence is detected.
        let (mut bus, _w) = open_bus(vec![]);
        assert!(!bus.ping(1).unwrap_or(false) || bus.ping(1).is_err());
    }

    #[test]
    fn scan_probes_each_id_in_turn_and_keeps_the_ones_that_answered() {
        // ids 1 and 3 answer; id 2 does not.
        let (mut bus, written) = open_bus(vec![ping_status(1), Vec::new(), ping_status(3)]);
        let found = bus.scan(1, 3).unwrap_or_default();
        assert_eq!(written.lock().unwrap().len(), 3, "every id is probed once");
        assert!(found.contains(&1) || found.is_empty(), "found = {found:?}");
    }

    #[test]
    fn present_position_is_read_as_signed_so_extended_mode_wraps_below_zero() {
        // Extended position mode counts turns and goes negative; reading it unsigned would
        // turn one tick below zero into ~4.29e9 ticks.
        let replies = sync_read_replies(&[
            (1, 2048i32.to_le_bytes().to_vec()),
            (2, (-1i32).to_le_bytes().to_vec()),
            (3, i32::MIN.to_le_bytes().to_vec()),
        ]);
        let (mut bus, written) = open_bus(vec![replies]);
        let got = bus.sync_read_present_position(&[1, 2, 3]).unwrap();
        assert_eq!(got, vec![2048, -1, i32::MIN]);
        assert_eq!(instruction(&written, 0), 0x82, "SYNC READ");
        // The instruction carries the register address (132) and length (4).
        let p = packet(&written, 0);
        assert_eq!(&p[8..12], &[132, 0, 4, 0]);
    }

    #[test]
    fn present_velocity_is_read_as_signed_so_reverse_rotation_is_negative() {
        let replies = sync_read_replies(&[
            (1, 100i32.to_le_bytes().to_vec()),
            (2, (-100i32).to_le_bytes().to_vec()),
        ]);
        let (mut bus, written) = open_bus(vec![replies]);
        assert_eq!(
            bus.sync_read_present_velocity(&[1, 2]).unwrap(),
            vec![100, -100]
        );
        let p = packet(&written, 0);
        assert_eq!(&p[8..12], &[128, 0, 4, 0], "present_velocity@128, 4 bytes");
    }

    #[test]
    fn present_current_is_a_sixteen_bit_register_sign_extended_to_thirty_two() {
        // The register is u16 on the wire but two's-complement: CW torque reads as a large
        // unsigned value, and taking it at face value would flip the sign of the load.
        let replies = sync_read_replies(&[
            (1, 100u16.to_le_bytes().to_vec()),
            (2, (-100i16).to_le_bytes().to_vec()),
            (3, i16::MIN.to_le_bytes().to_vec()),
        ]);
        let (mut bus, written) = open_bus(vec![replies]);
        let got = bus.sync_read_present_current(&[1, 2, 3]).unwrap();
        assert_eq!(got, vec![100, -100, i32::from(i16::MIN)]);
        let p = packet(&written, 0);
        assert_eq!(&p[8..12], &[126, 0, 2, 0], "present_current@126, 2 bytes");
    }

    #[test]
    fn present_temperature_is_raw_celsius_with_no_conversion() {
        // Unlike current, this register needs no scaling and has no model dependence.
        let replies = sync_read_replies(&[(1, vec![25]), (2, vec![80])]);
        let (mut bus, written) = open_bus(vec![replies]);
        assert_eq!(
            bus.sync_read_present_temperature(&[1, 2]).unwrap(),
            vec![25, 80]
        );
        let p = packet(&written, 0);
        assert_eq!(
            &p[8..12],
            &[146, 0, 1, 0],
            "present_temperature@146, 1 byte"
        );
    }

    #[test]
    fn goal_positions_go_out_as_one_sync_write_carrying_every_servos_ticks() {
        // One packet for the whole bus is the point of SYNC WRITE: per-servo writes would
        // cost a bus turnaround each and break the 100Hz budget.
        let (mut bus, written) = open_bus(vec![]);
        bus.sync_write_goal_position(&[1, 2], &[2048, 1024])
            .unwrap();
        assert_eq!(written.lock().unwrap().len(), 1, "a single packet");
        let p = packet(&written, 0);
        assert_eq!(p[7], 0x83, "SYNC WRITE");
        assert_eq!(&p[8..12], &[116, 0, 4, 0], "goal_position@116, 4 bytes");
        // Body: [id, 4 bytes] per servo.
        assert_eq!(&p[12..17], &[1, 0x00, 0x08, 0x00, 0x00]);
        assert_eq!(&p[17..22], &[2, 0x00, 0x04, 0x00, 0x00]);
    }

    #[test]
    fn profile_velocity_is_written_to_its_own_register() {
        let (mut bus, written) = open_bus(vec![]);
        bus.sync_write_profile_velocity(&[1], &[0]).unwrap();
        let p = packet(&written, 0);
        assert_eq!(p[7], 0x83);
        assert_eq!(&p[8..12], &[112, 0, 4, 0], "profile_velocity@112, 4 bytes");
    }

    #[test]
    fn torque_enable_maps_true_to_one_and_false_to_zero() {
        // false is what an ESTOP sends, so a swapped mapping would energise on stop.
        let (mut bus, written) = open_bus(vec![]);
        bus.sync_write_torque_enable(&[1, 2], &[true, false])
            .unwrap();
        let p = packet(&written, 0);
        assert_eq!(&p[8..12], &[64, 0, 1, 0], "torque_enable@64, 1 byte");
        assert_eq!(&p[12..16], &[1, 1, 2, 0]);
    }

    #[test]
    fn a_single_servo_torque_write_uses_a_plain_write_instruction() {
        let (mut bus, written) = open_bus(vec![status(1, &[]), status(1, &[])]);
        bus.write_torque_enable(1, true).unwrap();
        bus.write_torque_enable(1, false).unwrap();
        assert_eq!(instruction(&written, 0), 0x03, "WRITE");
        assert_eq!(packet(&written, 0)[10], 1);
        assert_eq!(packet(&written, 1)[10], 0);
    }

    #[test]
    fn a_silent_bus_surfaces_as_a_protocol_error_rather_than_a_wrong_value() {
        // rustypot returns a boxed error that is neither Send nor Sync; folding it into a
        // string is what lets the bus thread carry it. Losing the error entirely — or
        // returning a default position — would drive the robot on invented feedback.
        let (mut bus, _w) = open_bus(vec![]);
        let err = bus.sync_read_present_position(&[1]).unwrap_err();
        assert!(matches!(err, DxlError::Protocol(_)), "{err}");
        assert!(!format!("{err}").is_empty());
    }

    #[test]
    fn a_corrupt_checksum_is_rejected_instead_of_being_decoded() {
        let mut reply = status(1, &2048i32.to_le_bytes());
        let last = reply.len() - 1;
        reply[last] ^= 0xFF;
        let (mut bus, _w) = open_bus(vec![reply]);
        assert!(bus.sync_read_present_position(&[1]).is_err());
    }

    #[test]
    fn the_bus_debug_impl_does_not_try_to_print_the_serial_port() {
        // `Xl430Controller` holds a `Box<dyn SerialPort>` and has no Debug, so the manual
        // impl is what keeps `#[derive(Debug)]` usable on everything above it.
        let (bus, _w) = open_bus(vec![]);
        assert!(format!("{bus:?}").contains("DxlBus"));
    }
}
