# daisy_bsp

Rust `no_std`, `embedded_hal` board support package for the Electro-Smith Daisy Seed.

# Getting Started

```
cargo install cargo-generate

cargo generate \
    --git https://github.com/antoinevg/hello-daisy \
    --name hello-daisy
```


# Discussion

https://forum.electro-smith.com/t/rust-starter-for-daisy-seed/


# Status

Work in progress.


# Examples

Examples can be run with:

    cargo run --example <example_name>

Callbacks in the audio examples can take the form of either function pointers or closures.

To make use of closures, you will need to activate the `alloc` feature and compile with rust nightly.

For example:

    cargo +nightly run --example audio_midi --features="alloc, uses_num"
