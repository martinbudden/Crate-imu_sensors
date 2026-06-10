# Notes

To run tests on the PC:

```sh
cargo test --no-default-features --features "std,libm" --target x86_64-unknown-linux-gnu
```

To check with a given target use:

```sh
cargo check --target thumbv8m.main-none-eabihf
```
