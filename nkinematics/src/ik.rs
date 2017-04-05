extern crate nalgebra as na;

use alga::general::Real;
use na::{Isometry3, Vector3, Vector6, DVector, UnitQuaternion, DMatrix};
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
    JointOutOfLimit(JointError),
}

impl From<JointError> for IKError {
    fn from(err: JointError) -> IKError {
        IKError::JointOutOfLimit(err)
    }
}

impl fmt::Display for IKError {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        match *self {
            IKError::NotConverged => write!(f, "ik solve not converted"),
            IKError::InverseMatrixError => write!(f, "ik failed to solve inverse matrix"),
            IKError::PreconditionError => write!(f, "ik precondition not match"),
            IKError::JointOutOfLimit(ref err) => write!(f, "ik error : {}", err),
        }
    }
}

impl Error for IKError {
    fn description(&self) -> &str {
        match *self {
            IKError::NotConverged => "not converted",
            IKError::InverseMatrixError => "inverse matrix error",
            IKError::PreconditionError => "precondition not match",
            IKError::JointOutOfLimit(ref err) => err.description(),
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
        let dof = arm.linked_joints.len();
        let orig_angles = arm.get_joint_angles();
        //        let orig_angles_vec = DVector::from_fn(|i| orig_angles[i]);
        //        let orig_angles_vec = DVector::from_iterator(dof, orig_angles.iter());
        let mut orig_angles_vec = DVector::from_element(dof, T::zero());
        for i in 0..dof {
            orig_angles_vec[i] = orig_angles[i];
        }
        //println!("{}", orig_angles_vec);
        let orig_pose6 = calc_vector6_pose(&arm.calc_end_transform());
        let target_pose6 = calc_vector6_pose(&target_pose);
        let mut jacobi_vec = Vec::new();
        for i in 0..dof {
            let mut small_diff_angles_i = orig_angles.clone();
            small_diff_angles_i[i] += self.jacobian_move_epsilon;
            try!(arm.set_joint_angles(&small_diff_angles_i));
            let small_diff_pose6 = calc_vector6_pose(&arm.calc_end_transform());
            jacobi_vec.push(small_diff_pose6 - orig_pose6);
        }
        let jacobi = DMatrix::from_fn(6, dof, |r, c| jacobi_vec[c][r]);
        let j_inv = if dof > 6 {
            // use pseudo inverse
            jacobi.transpose() *
            try!((&jacobi * jacobi.transpose())
                     .try_inverse()
                     .ok_or(IKError::InverseMatrixError))
        } else {
            try!(jacobi.try_inverse().ok_or(IKError::InverseMatrixError))
        };
        let new_angles_diff = j_inv * (target_pose6 - orig_pose6) * self.jacobian_move_epsilon;
        //println!("{:}", new_angles_diff);
        let new_angles = orig_angles_vec + new_angles_diff;
        let mut angles_vec = Vec::new();
        for i in 0..dof {
            angles_vec.push(new_angles[i]);
        }
        try!(arm.set_joint_angles(&angles_vec));
        let new_pose6 = calc_vector6_pose(&arm.calc_end_transform());
        Ok((target_pose6 - new_pose6).norm())
    }

    pub fn solve(&self,
                 arm: &mut LinkedFrame<T>,
                 target_pose: &Isometry3<T>)
                 -> Result<T, IKError> {
        if arm.linked_joints.len() < 6 {
            println!("support only 6 or more DoF now");
            return Err(IKError::PreconditionError);
        }
        let orig_angles = arm.get_joint_angles();
        for _ in 0..self.num_max_try {
            let target_distance = try!(self.solve_one_loop(arm, &target_pose));
            if target_distance < self.allowable_target_distance {
                return Ok(target_distance);
            }
        }
        try!(arm.set_joint_angles(&orig_angles));
        Err(IKError::NotConverged)
    }
}
