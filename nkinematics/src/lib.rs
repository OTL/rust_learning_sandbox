extern crate alga;
extern crate nalgebra as na;

use alga::general::Real;
use na::{Isometry3, Vector3, Vector6, Matrix6, Unit, UnitQuaternion, Translation3};

fn to_euler_angles<T: Real>(q: &UnitQuaternion<T>) -> Vector3<T> {
    let x = q[0];
    let y = q[1];
    let z = q[2];
    let w = q[3];
    let ysqr = y * y;
    let _1: T = T::one();
    let _2: T = na::convert(1.0);

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

#[derive(Copy)]
pub enum JointType<T: Real> {
    /// Fixed joint. angle is not used.
    Fixed,
    /// Rotational joint around axis. angle [rad].
    Rotational{axis: Unit<Vector3<T>>},
    /// Linear joint. angle is length
    Linear{axis: Unit<Vector3<T>>},
}

impl<T> Clone for JointType<T> where T: Real {
    fn clone(&self) -> JointType<T> {
        *self
    }
}

pub struct RobotFrame<T: Real> {
    pub name: String,
    pub frames: Vec<LinkedFrame<T>>,
    pub transform: Isometry3<T>,
}

impl<T> RobotFrame<T> where T: Real {
    pub fn new(name: &str, frames: Vec<LinkedFrame<T>>) -> RobotFrame<T> {
        RobotFrame {
            name: name.to_string(),
            frames: frames,
            transform: Isometry3::identity(),
        }
    }
    pub fn calc_transforms(&self) -> Vec<Vec<Isometry3<T>>> {
        self.frames.iter().map(
            |ref lf|
            lf.calc_transforms().iter().map(|&tf| self.transform * tf).collect()).collect()
    }
}

/// Set of Joint and Link
///
/// imagine below structure
///
/// [transform] -> linked_joints([[joint] -> [Link]] -> [[joint] -> [Link]] -> ...)
pub struct LinkedFrame<T: Real> {
    pub name: String,
    pub linked_joints: Vec<LinkedJoint<T>>,
    pub transform: Isometry3<T>,
}

impl<T> LinkedFrame<T> where T: Real {
    pub fn new(name: &str) -> LinkedFrame<T> {
        LinkedFrame {
            name: name.to_string(),
            linked_joints: Vec::new(),
            transform: Isometry3::identity(),
        }
    }
    pub fn calc_transforms(&self) -> Vec<Isometry3<T>> {
        let mut vec = Vec::new();
        vec.push(self.transform);
        for lj in &self.linked_joints {
            let next = vec.last().unwrap() * lj.calc_transform();
            vec.push(next);
        }
        vec
    }

    pub fn calc_end_transform(&self) -> Isometry3<T> {
        self.calc_transforms().last().unwrap().clone()
    }

    pub fn set_joint_angles(&mut self, angles: &Vec<T>) {
        for (i, ang) in angles.iter().enumerate()  {
            self.linked_joints[i].set_joint_angle(*ang);
        }
    }
    pub fn get_joint_angles(&self) -> Vec<T> {
        self.linked_joints.iter().map(|ref linked_joint| linked_joint.get_joint_angle()).collect()
    }
    pub fn solve_inverse_kinematics_one_loop(&mut self, target_pose: &Isometry3<T>,
                                             move_velocity: T) -> T {
        fn calc_vector6_pose<T: Real>(pose: &Isometry3<T>) -> Vector6<T> {
            let rpy = to_euler_angles(&pose.rotation);
            Vector6::new(
                pose.translation.vector[0],
                pose.translation.vector[1],
                pose.translation.vector[2],
                rpy[0], rpy[1], rpy[2])
        }
        let orig_angles = self.get_joint_angles();
        let orig_angles_vec = Vector6::new(orig_angles[0], orig_angles[1], orig_angles[2],
                                           orig_angles[3], orig_angles[4], orig_angles[5]);
        let orig_pose6 = calc_vector6_pose(&self.calc_end_transform());
//        println!("orig_pose6 = {}", orig_pose6);
        let target_pose6 = calc_vector6_pose(&target_pose);
//        println!("target_pose6 = {}", target_pose6);
        let mut yacobi_vec = Vec::new();
        let da = T::from_f64(0.01).unwrap();
        for i in 0..6 {
            let mut small_diff_angles_i = orig_angles.clone();
            small_diff_angles_i[i] += da;
            self.set_joint_angles(&small_diff_angles_i);
            let small_diff_pose6 = calc_vector6_pose(&self.calc_end_transform());
            let jacobi = small_diff_pose6 - orig_pose6;
            yacobi_vec.push(jacobi);
        }
        let jacobi = Matrix6::from_columns(&[
            yacobi_vec[0],
            yacobi_vec[1],
            yacobi_vec[2],
            yacobi_vec[3],
            yacobi_vec[4],
            yacobi_vec[5]]);
        let j_inv = jacobi.try_inverse().unwrap();
        let new_angles_diff = j_inv * (target_pose6 - orig_pose6) * move_velocity;
        let new_angles = orig_angles_vec + new_angles_diff;
        self.set_joint_angles(&vec![new_angles[0], new_angles[1], new_angles[2], new_angles[3], new_angles[4],
                                   new_angles[5]]);
        let new_pose6 = calc_vector6_pose(&self.calc_end_transform());
        (target_pose6 - new_pose6).norm()
    }

    pub fn solve_inverse_kinematics(&mut self, target_pose: Isometry3<T>) -> bool {
        // currently
        if self.linked_joints.len() != 6 {
            println!("support only 6 DoF now");
            return false;
        }
        let allowable_target_distance = na::convert(0.01);
        for i in 0..100 {
            let target_distance = self.solve_inverse_kinematics_one_loop(&target_pose,
                                                                         na::convert(0.001));
            if target_distance < allowable_target_distance {
                return true;
            }
//            println!("[{}]dist = {}", i, target_distance);
        }
        false
    }
}

/// Joint and Link
///
/// [[joint] -> [Link]]
pub struct LinkedJoint<T: Real> {
    pub name: String,
    pub joint: Joint<T>,
    pub transform: Isometry3<T>,
}

impl<T> LinkedJoint<T> where T: Real {
    /// Construct a LinkedJoint from name and joint instance
    ///
    /// You can use LinkedJointBuilder<T> if you want.
    pub fn new(name: &str, joint: Joint<T>) -> LinkedJoint<T> {
        LinkedJoint {
            name: name.to_string(),
            joint: joint,
            transform: Isometry3::identity(),
        }
    }
    pub fn get_joint_name(&self) -> &str {
        &self.joint.name
    }
    pub fn calc_transform(&self) -> Isometry3<T> {
        self.joint.calc_transform() * self.transform
    }
    pub fn set_joint_angle(&mut self, angle: T) {
        self.joint.set_angle(angle)
    }
    pub fn get_joint_angle(&self) -> T {
        self.joint.get_angle()
    }
}

pub struct Range<T: Real> {
    pub min: T,
    pub max: T,
}

/// Joint with type
pub struct Joint<T: Real> {
    pub name: String,
    pub joint_type: JointType<T>,
    pub angle: T,
    pub limits: Option<Range<T>>,
}

impl<T> Joint<T> where T: Real {
    pub fn new(name: &str, joint_type: JointType<T>) -> Joint<T> {
        Joint { name: name.to_string(),
                joint_type: joint_type,
                angle: T::zero(),
                limits: None,
        }
    }
    pub fn set_limits(&mut self, limits: Option<Range<T>>) {
        self.limits = limits;
    }
    pub fn set_angle(&mut self, angle: T) {
        self.angle = angle;
    }
    pub fn get_angle(&self) -> T {
        self.angle
    }
    pub fn calc_transform(&self) -> Isometry3<T> {
        match self.joint_type {
            JointType::Fixed => Isometry3::identity(),
            JointType::Rotational{axis} =>
                Isometry3::from_parts(
                    Translation3::new(T::zero(), T::zero(), T::zero()),
                    UnitQuaternion::from_axis_angle(&axis, self.angle)),
            JointType::Linear{axis} =>
                Isometry3::from_parts(
                    Translation3::from_vector(&axis.unwrap() * self.angle),
                    UnitQuaternion::identity()),
        }
    }
}


/// Build a LinkedJoint<T>
///
/// # Examples
///
/// ```
/// let l0 = LinkedJointBuilder::new()
///     .name("link1")
///     .translation(Translation3::new(0.0, 0.1, 0.0))
///     .joint("link_pitch", JointType::Rotational{axis: Vector3::y_axis()})
///     .finalize();
/// ```

pub struct LinkedJointBuilder<T: Real> {
    name: String,
    joint: Joint<T>,
    transform: Isometry3<T>,
}

impl<T> LinkedJointBuilder<T> where T: Real {
    pub fn new() -> LinkedJointBuilder<T> {
        LinkedJointBuilder {
            name: "".to_string(),
            joint: Joint::new("", JointType::Fixed),
            transform: Isometry3::identity(),
        }
    }
    pub fn name(mut self, name: &str) -> LinkedJointBuilder<T> {
        self.name = name.to_string();
        self
    }
    pub fn joint(mut self, name: &str, joint_type: JointType<T>) -> LinkedJointBuilder<T> {
        self.joint = Joint::new(name, joint_type);
        self
    }
    pub fn transform(mut self, transform: Isometry3<T>) -> LinkedJointBuilder<T> {
        self.transform = transform;
        self
    }
    pub fn translation(mut self, translation: Translation3<T>) -> LinkedJointBuilder<T> {
        self.transform.translation = translation;
        self
    }
    pub fn rotation(mut self, rotation: UnitQuaternion<T>) -> LinkedJointBuilder<T> {
        self.transform.rotation = rotation;
        self
    }
    pub fn finalize(self) -> LinkedJoint<T> {
        LinkedJoint {name: self.name, joint: self.joint,
                     transform: self.transform}
    }
}


mod test {
    #[test]
    fn test_to_euler_angles() {
        use super::*;
        let q = na::UnitQuaternion::from_euler_angles(0.1, 0.2, 0.3);
        let rpy = to_euler_angles(&q);
//        println!("{:}", rpy);
        assert!((rpy[0] - 0.1).abs() < 0.0001);
        assert!((rpy[1] - 0.2).abs() < 0.0001);
        assert!((rpy[2] - 0.3).abs() < 0.0001);
    }
}
