# libaubo

`libaubo` is the Aubo model and behavior-interface scaffold in the [Robot-Exp driver ecosystem](https://github.com/Robot-Exp-Platform/robot_behavior). The published crate is **`libaubo 0.2.0`**.

**This version does not connect to or control an Aubo robot.** `AuboRobot::new(ip)` ignores its IP argument, `read_state()` returns a default placeholder state, and target motion and load setting are unimplemented. Use it to explore the typed API and current parameters, not as a working hardware backend.

## Why it exists

The intended separation matches other Robot-Exp drivers: model types identify the robot and joint count, `robot_behavior` traits describe capabilities, and a device backend implements the actual protocol. This crate provides the first part and the shape of the second. Aubo C, E, I, IH, and IS aliases share the generic `AuboRobot<T, N>` implementation.

The useful workflow today is **select a model type → inspect capability types → configure local parameters → develop or integrate a real backend**. A trait containing placeholders does not establish hardware support. Generic joint/Cartesian bounds in the source are not a calibrated database for every Aubo model.

## Install

Create a binary project with `cargo new aubo-types`, then add:

```toml
[dependencies]
libaubo = "0.2.0"
robot_behavior = "0.6.1"
```

Use Rust nightly because these crates use unstable Rust features. The direct behavior dependency provides the traits below and selects the published 0.6.1 fixes. This offline scaffold needs no parent `drives` checkout or vendor SDK installation.

The default feature set is empty. Although `ffi`, `to_c`, `to_py`, and `to_cxx` appear in the manifest, the current library entry point does not expose a working Aubo language-binding module. Enabling these flags does not add a controller connection. There is no supported robot/firmware/host matrix to infer from this release.

## Minimal example: inspect a model type offline

Put this in `src/main.rs`. It creates a local object, sets a local parameter, and prints generic joint bounds. It performs no network or robot operations.

```rust
use libaubo::AuboI5;
use robot_behavior::{Joints, RobotResult};

fn main() -> RobotResult<()> {
    // The constructor currently ignores this argument; no connection is made.
    let mut robot = AuboI5::new("unused");
    robot.set_scale(0.1)?;
    println!("generic lower bounds: {:?}", <AuboI5 as Joints<6>>::JOINT_MIN);
    println!("generic upper bounds: {:?}", <AuboI5 as Joints<6>>::JOINT_MAX);
    Ok(())
}
```

Check without running:

```sh
cargo +nightly check
```

The constructor and `set_scale()` only affect the local Rust object. The printed bounds are implementation defaults, not measurements or a promise that they are appropriate limits for a physical Aubo I5.

## Implemented behavior and next steps

| Area | Current behavior |
|---|---|
| Model aliases | Typed model names and joint counts. |
| Parameter builders | Local coordinate, scale, velocity, and acceleration settings; several jerk/torque builders are placeholders. |
| `Robot::read_state()` / `Arm::state()` | Default `ArmState`, not device feedback. |
| `get_joint()` / `get_endpoint()` | Zero/default placeholders. |
| Joint/flange `move_to()` and `set_load()` | `unimplemented!()`; they panic. |
| Transport and control sessions | No implemented Aubo network backend or native controller session. |

A real backend needs explicit connection errors, live state conversion, and command-completion semantics, together with units, model-specific parameters, and platform requirements. Those are necessary before this API can support hardware motion.

The repository has no examples directory. Start with [model aliases](src/aubo), [the generic implementation](src/robot.rs), the [published API](https://docs.rs/libaubo/0.2.0/libaubo/), and the [behavior guide](https://github.com/Robot-Exp-Platform/robot_behavior). The complete example above stays within implemented offline behavior.

## Source builds and license

Checkout manifests pin development behavior dependencies to a Git revision and require access to that GitHub source. Registry installation uses published dependencies and needs no adjacent workspace.

The scaffold is maintained by Robot-Exp-Platform under [Apache-2.0](LICENSE). Aubo is the manufacturer's name. This repository does not bundle an Aubo SDK or confer rights to separate vendor firmware and software.
