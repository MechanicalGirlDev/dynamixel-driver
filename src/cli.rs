//! Command-line tools for controlling Dynamixel X-series servos over serial.
//!
//! Supports bus scanning, position and velocity reads, goal-position writes, and torque control.

use clap::{Parser, Subcommand};

use crate::bus::DxlBus;
use crate::error::Result;

/// Command-line arguments for the `dynamixel` tool.
#[derive(Parser, Debug)]
#[command(name = "dynamixel")]
#[command(author, version, about = "Dynamixel X-series control (Protocol 2.0)", long_about = None)]
pub struct Cli {
    /// Serial port (for example, `COM3` on Windows or `/dev/ttyUSB0` on Linux)
    #[arg(short, long, default_value = "/dev/ttyUSB0")]
    pub port: String,

    /// Serial baud rate (the X-series default is 1 Mbps)
    #[arg(short, long, default_value_t = 1_000_000)]
    pub baud: u32,

    /// Response timeout in milliseconds
    #[arg(short, long, default_value_t = 50)]
    pub timeout: u64,

    /// Command to execute
    #[command(subcommand)]
    pub command: Commands,
}

/// Supported Dynamixel bus operations.
#[derive(Subcommand, Debug)]
pub enum Commands {
    /// Scan the bus for responding servos
    Scan {
        /// First servo ID to scan
        #[arg(long, default_value_t = 0)]
        start: u8,
        /// Last servo ID to scan
        #[arg(long, default_value_t = 30)]
        end: u8,
    },
    /// Read present position ticks
    Position {
        /// Servo IDs to read
        ids: Vec<u8>,
    },
    /// Read present velocity values
    Velocity {
        /// Servo IDs to read
        ids: Vec<u8>,
    },
    /// Write a goal position in ticks
    Move {
        /// Servo ID
        id: u8,
        /// Goal position (0..4095 represents one rotation)
        tick: u32,
    },
    /// Enable or disable servo torque
    Torque {
        /// Servo ID
        id: u8,
        /// Use `on` to enable torque or `off` to release the servo
        #[arg(value_parser = ["on", "off"])]
        state: String,
    },
}

/// Executes the selected CLI command.
pub fn run(cli: &Cli) -> Result<()> {
    let timeout = core::time::Duration::from_millis(cli.timeout);
    let mut bus = DxlBus::open(&cli.port, cli.baud, timeout)?;

    match &cli.command {
        Commands::Scan { start, end } => {
            let found = bus.scan(*start, *end)?;
            println!("Responding servos: {found:?}");
        }
        Commands::Position { ids } => {
            let pos = bus.sync_read_present_position(ids)?;
            for (id, p) in ids.iter().zip(pos) {
                println!("ID {id}: present_position = {p} tick");
            }
        }
        Commands::Velocity { ids } => {
            let vel = bus.sync_read_present_velocity(ids)?;
            for (id, v) in ids.iter().zip(vel) {
                println!("ID {id}: present_velocity = {v} raw");
            }
        }
        Commands::Move { id, tick } => {
            bus.sync_write_torque_enable(&[*id], &[true])?;
            bus.sync_write_goal_position(&[*id], &[*tick])?;
            println!("ID {id}: goal_position <- {tick} tick");
        }
        Commands::Torque { id, state } => {
            let on = state == "on";
            bus.write_torque_enable(*id, on)?;
            println!("ID {id}: torque_enable <- {on}");
        }
    }
    Ok(())
}
