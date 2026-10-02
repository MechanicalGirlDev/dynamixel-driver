//! Dynamixel X-series serial transport and command-line interface.

use clap::Parser;
use dynamixel_serial::cli::{Cli, run};

fn main() {
    if let Err(e) = run(&Cli::parse()) {
        eprintln!("Error: {e}");
        std::process::exit(1);
    }
}
