//! Error types returned by the Dynamixel serial transport.

use thiserror::Error;

/// Errors produced while accessing a Dynamixel serial bus.
#[derive(Error, Debug)]
pub enum DxlError {
    /// The serial port could not be opened or configured.
    #[error("Serial port error: {0}")]
    Serial(#[from] serialport::Error),

    /// A serial I/O operation failed.
    #[error("I/O error: {0}")]
    Io(#[from] std::io::Error),

    /// The Dynamixel protocol transaction failed.
    ///
    /// The underlying `rustypot` error is stored as text so it can cross thread boundaries.
    #[error("Dynamixel protocol error: {0}")]
    Protocol(String),

    /// A supplied argument is invalid.
    #[error("Invalid argument: {0}")]
    InvalidArgument(String),
}

/// Result type used by this crate.
pub type Result<T> = core::result::Result<T, DxlError>;

/// Converts a `rustypot` error into a crate-level protocol error.
pub(crate) fn protocol_err(e: Box<dyn core::error::Error>) -> DxlError {
    DxlError::Protocol(e.to_string())
}
