extern crate nalgebra as na;

use std::marker::PhantomData;
use alga::general::Real;
use na::{Isometry3, Vector6, DMatrix};
use std::error::Error;
use std::fmt;
use links::*;
use math::*;

fn calc_vector6_pose<T: Real>(pose: &Isometry3<T>) -> Vector6<T> {
    let rpy = to_euler_angles(&pose.rotation);
    Vector6::new(pose.translation.vector[0],
                 pose.translation.vector[1],
                 pose.translation.vector[2],
                 rpy[0],
                 rpy[1],
                 rpy[2])
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
            IKError::NotConverged => "not converged",
            IKError::InverseMatrixError => "inverse matrix error",
            IKError::PreconditionError => "precondition not match",
            IKError::JointOutOfLimit(ref err) => err.description(),
        }
    }
}


pub trait InverseKinematicsSolver<T, K>
    where T: Real,
          K: KinematicChain<T>
{
    fn solve(&self, arm: &mut K, target_pose: &Isometry3<T>) -> Result<T, IKError>;
}


#[derive(Debug)]
pub struct JacobianIKSolver<T: Real, K: KinematicChain<T>> {
    pub jacobian_move_epsilon: T,
    pub allowable_target_distance: T,
    pub num_max_try: i32,
    phantom: PhantomData<K>,
}

impl<T, K> JacobianIKSolver<T, K>
    where T: Real,
          K: KinematicChain<T>
{
    pub fn new(jacobian_move_epsilon: T,
               allowable_target_distance: T,
               num_max_try: i32)
               -> JacobianIKSolver<T, K> {
        JacobianIKSolver {
            jacobian_move_epsilon: jacobian_move_epsilon,
            allowable_target_distance: allowable_target_distance,
            num_max_try: num_max_try,
            phantom: PhantomData::<K>,
        }
    }
    fn solve_one_loop(&self, arm: &mut K, target_pose: &Isometry3<T>) -> Result<T, IKError> {
        let orig_angles = arm.get_joint_angles();
        let dof = orig_angles.len();
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
            try!(try_pseudo_inverse(&jacobi).ok_or(IKError::InverseMatrixError))
        } else {
            try!(jacobi.try_inverse().ok_or(IKError::InverseMatrixError))
        };
        let new_angles_diff = j_inv * (target_pose6 - orig_pose6) * self.jacobian_move_epsilon;
        let mut angles_vec = Vec::new();
        for i in 0..dof {
            angles_vec.push(orig_angles[i] + new_angles_diff[i]);
        }
        try!(arm.set_joint_angles(&angles_vec));
        let new_pose6 = calc_vector6_pose(&arm.calc_end_transform());
        Ok((target_pose6 - new_pose6).norm())
    }
}

impl<T, K> InverseKinematicsSolver<T, K> for JacobianIKSolver<T, K>
    where T: Real,
          K: KinematicChain<T>
{
    fn solve(&self, arm: &mut K, target_pose: &Isometry3<T>) -> Result<T, IKError> {
        let orig_angles = arm.get_joint_angles();
        if orig_angles.len() < 6 {
            println!("support only 6 or more DoF now");
            return Err(IKError::PreconditionError);
        }
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
