# k

Kinematics library for rust-lang.

k uses [nalgebra](http://nalgebra.org) as math library.

See [Document](http://docs.rs/k) and examples/ for more details.

* [Document](http://docs.rs/k)

## Enjoy an IK example

```bash
cargo run --release --example interactive_ik
```

![ik_sample](img/screenshot.png)


Push below keys to move the end of the manipulator.

- `f`: forward
- `b`: backward
- `p`: up
- `n`: down
- `l`: left
- `r`: right
- `z`: reset the manipulator state.

## Create link tree from urdf

```rust
```