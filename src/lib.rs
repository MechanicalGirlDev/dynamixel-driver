//! Dynamixel X-series serial transport and command-line interface.
//!
//! Dynamixel X-series serial transport and command-line interface.
//! Dynamixel X-series serial transport and command-line interface.
//! Dynamixel X-series serial transport and command-line interface.
//! Dynamixel X-series serial transport and command-line interface.
//! Dynamixel X-series serial transport and command-line interface.
//!
//! Dynamixel X-series serial transport and command-line interface.
//! Dynamixel X-series serial transport and command-line interface.

extern crate alloc;

pub mod bus;
pub mod cli;
pub mod error;
#[cfg(test)]
pub(crate) mod fake;

pub use bus::DxlBus;
pub use error::{DxlError, Result};
