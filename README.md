# Dynamixel Serial

An independent Rust crate providing synchronous serial (TTL/RS-485) transport and a command-line interface for Dynamixel X-series servos using Protocol 2.0.

The library wraps `rustypot`'s `Xl430Controller` and exposes raw register values in ticks. Position-to-radian conversion and per-servo mapping belong to the calling application.

## Reiny 0.8 integration

`dynamixel-serial` owns serial transactions, not deployment state or message
schemas. Use it from a Reiny adapter rather than adding the SDK to the transport.
The `dynamixel` CLI remains a standalone bring-up tool.

The adapter declares command inputs and feedback outputs in `main.yaml`, opens
the corresponding named `Cloudy::input`/`output` ports, initializes its bus and
then calls `Cloudy::ready()`. It owns ticks-to-radians conversion, servo mapping,
feedback freshness and the safe torque/output policy on `Cloudy::shutdown()`.
Release the bus after that policy has completed; Reiny's stop acknowledgement
alone does not mean the actuators are safe. Preserve the full deployment/module
namespace in feedback provenance.

Use published `reiny = "0.8.0"` and `reiny-build = "0.8.0"` in the adapter.
Its runtime definition and schema catalog use `version: 2`. The adapter's
own `main.yaml` declares its executable, build and endpoint policies:
input `replay`/`buffer` and output `qos`/`retention`. Callers reuse it through
`source` and wire inputs with `from`, without repeating child outputs.

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
