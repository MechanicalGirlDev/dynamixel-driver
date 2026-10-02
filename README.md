# Dynamixel Serial

An independent Rust crate providing synchronous serial (TTL/RS-485) transport and a command-line interface for Dynamixel X-series servos using Protocol 2.0.

The library wraps `rustypot`'s `Xl430Controller` and exposes raw register values in ticks. Position-to-radian conversion and per-servo mapping belong to the calling application.

## Build and test

```sh
cargo fmt --all --check
cargo clippy --workspace --all-targets --all-features -- -D warnings
cargo test --workspace --all-features
```

On Linux, install `libudev-dev` and `pkg-config` before building.

## CLI

Run `cargo run -- --help` for usage. The `dynamixel` command supports scanning a bus, reading position and velocity, moving a servo, and enabling or disabling torque. The default serial port is `/dev/ttyUSB0`, baud rate is 1 Mbps, and timeout is 50 ms.

## License

Licensed under the Apache License, Version 2.0. See [LICENSE](LICENSE) and [NOTICE](NOTICE).
