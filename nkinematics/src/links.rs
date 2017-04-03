extern crate nalgebra as na;

use alga::general::Real;
use na::{Isometry3, Vector3, Unit, UnitQuaternion, Translation3};

#[derive(Copy)]
pub enum JointType<T: Real> {
    /// Fixed joint. angle is not used.
    Fixed,
    /// Rotational joint around axis. angle [rad].
    Rotational { axis: Unit<Vector3<T>> },
    /// Linear joint. angle is length
    Linear { axis: Unit<Vector3<T>> },
}

impl<T> Clone for JointType<T>
    where T: Real
{
    fn clone(&self) -> JointType<T> {
        *self
    }
}

pub struct RobotFrame<T: Real> {
    pub name: String,
    pub frames: Vec<LinkedFrame<T>>,
    pub transform: Isometry3<T>,
}

impl<T> RobotFrame<T>
    where T: Real
{
    pub fn new(name: &str, frames: Vec<LinkedFrame<T>>) -> RobotFrame<T> {
        RobotFrame {
            name: name.to_string(),
            frames: frames,
            transform: Isometry3::identity(),
        }
    }
    pub fn calc_transforms(&self) -> Vec<Vec<Isometry3<T>>> {
        self.frames
            .iter()
            .map(|ref lf| {
                     lf.calc_transforms()
                         .iter()
                         .map(|&tf| self.transform * tf)
                         .collect()
                 })
            .collect()
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

impl<T> LinkedFrame<T>
    where T: Real
{
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
        for (i, ang) in angles.iter().enumerate() {
            self.linked_joints[i].set_joint_angle(*ang);
        }
    }
    pub fn get_joint_angles(&self) -> Vec<T> {
        self.linked_joints
            .iter()
            .map(|ref linked_joint| linked_joint.get_joint_angle())
            .collect()
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

impl<T> LinkedJoint<T>
    where T: Real
{
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

impl<T> Joint<T>
    where T: Real
{
    pub fn new(name: &str, joint_type: JointType<T>) -> Joint<T> {
        Joint {
            name: name.to_string(),
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
            JointType::Rotational { axis } => {
                Isometry3::from_parts(Translation3::new(T::zero(), T::zero(), T::zero()),
                                      UnitQuaternion::from_axis_angle(&axis, self.angle))
            }
            JointType::Linear { axis } => {
                Isometry3::from_parts(Translation3::from_vector(&axis.unwrap() * self.angle),
                                      UnitQuaternion::identity())
            }
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

impl<T> LinkedJointBuilder<T>
    where T: Real
{
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
        LinkedJoint {
            name: self.name,
            joint: self.joint,
            transform: self.transform,
        }
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
