//! Kinematics(forward/inverse) library using [nalgebra](http://nalgebra.org).
//!
//!
//! # Examples
//!
//! Build `JointWithLinkArray` using `JointWithLinkBuilder` at first.
//! Instead of using the builder, You can use `URDF` format
//! by `nkinamtics-urdf` cate if you want.
//!
//! ```
//! extern crate k;
//! extern crate nalgebra;
//!
//! use k::{JointWithLinkArray, JointWithLinkBuilder, JointType,
//!                   KinematicChain, JacobianIKSolverBuilder,
//!                   InverseKinematicsSolver};
//! use nalgebra::{Vector3, Translation3};
//!
//! fn main() {
//!   let l0 = JointWithLinkBuilder::new()
//!       .name("shoulder_link1")
//!       .joint("shoulder_pitch",
//!              JointType::Rotational { axis: Vector3::y_axis() })
//!       .finalize();
//!   let l1 = JointWithLinkBuilder::new()
//!       .name("shoulder_link2")
//!       .joint("shoulder_roll",
//!              JointType::Rotational { axis: Vector3::x_axis() })
//!       .translation(Translation3::new(0.0, 0.1, 0.0))
//!       .finalize();
//!   let l2 = JointWithLinkBuilder::new()
//!       .name("shoulder_link3")
//!       .joint("shoulder_yaw",
//!              JointType::Rotational { axis: Vector3::z_axis() })
//!       .translation(Translation3::new(0.0, 0.0, -0.30))
//!       .finalize();
//!   let l3 = JointWithLinkBuilder::new()
//!       .name("elbow_link1")
//!       .joint("elbow_pitch",
//!              JointType::Rotational { axis: Vector3::y_axis() })
//!       .translation(Translation3::new(0.0, 0.0, -0.15))
//!       .finalize();
//!   let l4 = JointWithLinkBuilder::new()
//!       .name("wrist_link1")
//!       .joint("wrist_yaw",
//!              JointType::Rotational { axis: Vector3::z_axis() })
//!       .translation(Translation3::new(0.0, 0.0, -0.15))
//!       .finalize();
//!   let l5 = JointWithLinkBuilder::new()
//!       .name("wrist_link2")
//!       .joint("wrist_pitch",
//!              JointType::Rotational { axis: Vector3::y_axis() })
//!       .translation(Translation3::new(0.0, 0.0, -0.15))
//!       .finalize();
//!   let l6 = JointWithLinkBuilder::new()
//!       .name("wrist_link3")
//!       .joint("wrist_roll",
//!              JointType::Rotational { axis: Vector3::x_axis() })
//!       .translation(Translation3::new(0.0, 0.0, -0.10))
//!       .finalize();
//!   let mut arm = JointWithLinkArray::new("arm", vec![l0, l1, l2, l3, l4, l5, l6]);
//!
//!   // set joint angles
//!   let angles = vec![0.8, 0.2, 0.0, -1.5, 0.0, -0.3, 0.0];
//!   arm.set_joint_angles(&angles).unwrap();
//!   // get the transform of the end of the manipulator (forward kinematics)
//!   let mut target = arm.calc_end_transform();
//!   target.translation.vector[2] += 0.1;
//!   let solver = JacobianIKSolverBuilder::new().finalize();
//!   // solve and move the manipulator angles
//!   solver.solve(&mut arm, &target)
//!         .unwrap_or_else(|err| {
//!                               println!("Err: {}", err);
//!                               0.0f32
//!                               });
//!   println!("angles={:?}", arm.get_joint_angles());
//! }
//! ```

extern crate alga;
extern crate nalgebra as na;
#[macro_use]
extern crate log;

mod links;
mod ik;
mod math;
mod rctree;
mod rctree_links;
pub mod urdf;

pub use self::links::*;
pub use self::ik::*;
pub use self::math::*;
pub use self::rctree::*;
pub use self::rctree_links::*;