# k

Kinematics (forward/inverse) library using [nalgebra](http://nalgebra.org).

It uses JointWithLinkStar structure to handle joints and links.
See examples/ for more details.

## Enjoy example at first!!

```bash
cargo run --release --example interactive_ik
```

![ik_sample](screenshot.png)


Push below keys to move the end of the manipulator.

- *f*: forward
- *b*: backward
- *p*: up
- *n*: down
- *l*: left
- *r*: right
- *z*: reset the manipulator state.

## URDF

You can use URDF file to build your robot model using `k-urdf` crate.
