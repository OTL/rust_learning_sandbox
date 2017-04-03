extern crate nalgebra as na;

use alga::general::Real;
use na::{Isometry3, Vector3, Vector6, Matrix6, UnitQuaternion};
use std::error::Error;
use std::fmt;

use links::*;

fn calc_vector6_pose<T: Real>(pose: &Isometry3<T>) -> Vector6<T> {
    let rpy = to_euler_angles(&pose.rotation);
    Vector6::new(pose.translation.vector[0],
                 pose.translation.vector[1],
                 pose.translation.vector[2],
                 rpy[0],
                 rpy[1],
                 rpy[2])
}

fn to_euler_angles<T: Real>(q: &UnitQuaternion<T>) -> Vector3<T> {
    let x = q[0];
    let y = q[1];
    let z = q[2];
    let w = q[3];
    let ysqr = y * y;
    let _1: T = T::one();
    let _2: T = na::convert(2.0);

    // roll
    let t0 = _2 * (w * x + y * z);
    let t1 = _1 - _2 * (x * x + ysqr);
    let roll = t0.atan2(t1);

    // pitch
    let t2 = _2 * (w * y - z * x);
    let t2 = if t2 > _1 { _1 } else { t2 };
    let t2 = if t2 < -_1 { -_1 } else { t2 };
    let pitch = t2.asin();

    // yaw
    let t3 = _2 * (w * z + x * y);
    let t4 = _1 - _2 * (ysqr + z * z);
    let yaw = t3.atan2(t4);

    Vector3::new(roll, pitch, yaw)
}

#[derive(Debug)]
pub enum IKError {
    NotConverged,
    InverseMatrixError,
    PreconditionError,
}

impl fmt::Display for IKError {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        match *self {
            IKError::NotConverged => write!(f, "not converted"),
            IKError::InverseMatrixError => write!(f, "failed to solve inverse matrix"),
            IKError::PreconditionError => write!(f, "precondition not match"),
        }
    }
}

impl Error for IKError {
    fn description(&self) -> &str {
        match *self {
            IKError::NotConverged => "not converted",
            IKError::InverseMatrixError => "inverse matrix error",
            IKError::PreconditionError => "precondition not match",
        }
    }
}

#[derive(Debug)]
pub struct InverseKinematicsSolver<T: Real> {
    pub jacobian_move_epsilon: T,
    pub allowable_target_distance: T,
    pub num_max_try: i32,
}

impl<T> InverseKinematicsSolver<T>
    where T: Real
{
    pub fn new(jacobian_move_epsilon: T,
               allowable_target_distance: T,
               num_max_try: i32)
               -> InverseKinematicsSolver<T> {
        InverseKinematicsSolver {
            jacobian_move_epsilon: jacobian_move_epsilon,
            allowable_target_distance: allowable_target_distance,
            num_max_try: num_max_try,
        }
    }
    fn solve_one_loop(&self,
                      arm: &mut LinkedFrame<T>,
                      target_pose: &Isometry3<T>)
                      -> Result<T, IKError> {
        let orig_angles = arm.get_joint_angles();
        let orig_angles_vec = Vector6::new(orig_angles[0],
                                           orig_angles[1],
                                           orig_angles[2],
                                           orig_angles[3],
                                           orig_angles[4],
                                           orig_angles[5]);
        let orig_pose6 = calc_vector6_pose(&arm.calc_end_transform());
        let target_pose6 = calc_vector6_pose(&target_pose);
        let mut yacobi_vec = Vec::new();
        for i in 0..6 {
            let mut small_diff_angles_i = orig_angles.clone();
            small_diff_angles_i[i] += self.jacobian_move_epsilon;
            arm.set_joint_angles(&small_diff_angles_i);
            let small_diff_pose6 = calc_vector6_pose(&arm.calc_end_transform());
            let jacobi = small_diff_pose6 - orig_pose6;
            yacobi_vec.push(jacobi);
        }
        let jacobi = Matrix6::from_columns(&[yacobi_vec[0],
                                             yacobi_vec[1],
                                             yacobi_vec[2],
                                             yacobi_vec[3],
                                             yacobi_vec[4],
                                             yacobi_vec[5]]);
        let j_inv = try!(jacobi.try_inverse().ok_or(IKError::InverseMatrixError));
        let new_angles_diff = j_inv * (target_pose6 - orig_pose6) * self.jacobian_move_epsilon;
        let new_angles = orig_angles_vec + new_angles_diff;
        arm.set_joint_angles(&vec![new_angles[0],
                                   new_angles[1],
                                   new_angles[2],
                                   new_angles[3],
                                   new_angles[4],
                                   new_angles[5]]);
        let new_pose6 = calc_vector6_pose(&arm.calc_end_transform());
        Ok((target_pose6 - new_pose6).norm())
    }

    pub fn solve(&self,
                 arm: &mut LinkedFrame<T>,
                 target_pose: &Isometry3<T>)
                 -> Result<T, IKError> {
        if arm.linked_joints.len() != 6 {
            println!("support only 6 DoF now");
            return Err(IKError::PreconditionError);
        }
        let orig_angles = arm.get_joint_angles();
        for _ in 0..self.num_max_try {
            let target_distance = try!(self.solve_one_loop(arm, &target_pose));
            if target_distance < self.allowable_target_distance {
                return Ok(target_distance);
            }
        }
        arm.set_joint_angles(&orig_angles);
        Err(IKError::NotConverged)
    }
}
