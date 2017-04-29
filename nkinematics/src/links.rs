extern crate nalgebra as na;

use alga::general::Real;
use na::{Isometry3, Vector3, Unit, UnitQuaternion, Translation3};
use std::error::Error;
use std::fmt;

#[derive(Copy, Debug, Clone)]
pub enum JointType<T: Real> {
    /// Fixed joitn
    Fixed,
    /// Rotational joint around axis. angle [rad].
    Rotational { axis: Unit<Vector3<T>> },
    /// Linear joint. angle is length
    Linear { axis: Unit<Vector3<T>> },
}

#[derive(Debug, Clone)]
pub enum JointError {
    OutOfLimit,
    SizeMisMatch,
}

impl fmt::Display for JointError {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        match *self {
            JointError::OutOfLimit => write!(f, "limit over"),
            JointError::SizeMisMatch => write!(f, "size is invalid"),
        }
    }
}

impl Error for JointError {
    fn description(&self) -> &str {
        match *self {
            JointError::OutOfLimit => "limit over",
            JointError::SizeMisMatch => "size is invalid",
        }
    }
}



/// Robot representation with set of LinkedFrames
///
/// This contains multiple LinkedFrame.
/// The frames must be serial without branch.
/// root is the only link which has branch.
#[derive(Debug, Clone)]
pub struct RobotFrame<T: Real> {
    pub name: String,
    pub frames: Vec<LinkedFrame<T>>,
    transform: Isometry3<T>,
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
    pub fn calc_link_transforms(&self) -> Vec<Vec<Isometry3<T>>> {
        self.frames
            .iter()
            .map(|ref lf| {
                     lf.calc_link_transforms()
                         .iter()
                         .map(|&tf| self.transform * tf)
                         .collect()
                 })
            .collect()
    }
    pub fn set_transform(&mut self, transform: Isometry3<T>) {
        self.transform = transform;
        for frame in self.frames.iter_mut() {
            frame.transform = self.transform * frame.transform;
        }
    }
    pub fn get_transform(&self) -> Isometry3<T> {
        return self.transform;
    }
}

pub trait KinematicChain<T>
    where T: Real
{
    fn calc_end_transform(&self) -> Isometry3<T>;
    fn set_joint_angles(&mut self, angles: &Vec<T>) -> Result<(), JointError>;
    fn get_joint_angles(&self) -> Vec<T>;
}

/// Set of Joint and Link
///
/// imagine below structure
///
/// `[transform] -> linked_joints([[joint] -> [Link]] -> [[joint] -> [Link]] -> ...)`
///
/// The order must be ordered.
///
/// - start from root link
/// - end with last link
#[derive(Debug, Clone)]
pub struct LinkedFrame<T: Real> {
    pub name: String,
    pub linked_joints: Vec<LinkedJoint<T>>,
    pub transform: Isometry3<T>,
}

impl<T> LinkedFrame<T>
    where T: Real
{
    pub fn new(name: &str, linked_joints: Vec<LinkedJoint<T>>) -> LinkedFrame<T> {
        LinkedFrame {
            name: name.to_string(),
            linked_joints: linked_joints,
            transform: Isometry3::identity(),
        }
    }
    /// returns transforms of links
    pub fn calc_link_transforms(&self) -> Vec<Isometry3<T>> {
        self.linked_joints
            .iter()
            .scan(self.transform, |base, ref lj| {
                *base *= lj.calc_transform();
                Some(*base)
            })
            .collect()
    }
    pub fn len(&self) -> usize {
        self.linked_joints.len()
    }
}

impl<T> KinematicChain<T> for LinkedFrame<T>
    where T: Real
{
    fn calc_end_transform(&self) -> Isometry3<T> {
        self.linked_joints
            .iter()
            .fold(self.transform, |trans, ref lj| trans * lj.calc_transform())
    }

    /// if failed, joints angles are non determined,
    fn set_joint_angles(&mut self, angles: &Vec<T>) -> Result<(), JointError> {
        // TODO: is it possible to cache the joint_with_angle to speed up?
        let mut joints_with_angle = self.linked_joints
            .iter_mut()
            .filter(|ref lj| lj.has_joint_angle())
            .collect::<Vec<_>>();
        if joints_with_angle.len() != angles.len() {
            return Err(JointError::SizeMisMatch);
        }
        for (i, lj) in joints_with_angle.iter_mut().enumerate() {
            try!(lj.set_joint_angle(angles[i]));
        }

        Ok(())
    }
    fn get_joint_angles(&self) -> Vec<T> {
        self.linked_joints
            .iter()
            .filter_map(|ref linked_joint| linked_joint.get_joint_angle())
            .collect()
    }
}

/// Joint and Link
///
#[derive(Debug, Clone)]
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
        self.transform * self.joint.calc_transform()
    }
    pub fn set_joint_angle(&mut self, angle: T) -> Result<(), JointError> {
        self.joint.set_angle(angle)
    }
    pub fn get_joint_angle(&self) -> Option<T> {
        self.joint.get_angle()
    }
    pub fn has_joint_angle(&self) -> bool {
        match self.joint.joint_type {
            JointType::Fixed => false,
            _ => true,
        }
    }
}

#[derive(Clone, Debug)]
pub struct Range<T: Real> {
    pub min: T,
    pub max: T,
}

impl<T> Range<T>
    where T: Real
{
    pub fn is_valid(&self, val: T) -> bool {
        val < self.max && val > self.min
    }
}

/// Joint with type
#[derive(Debug, Clone)]
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
    pub fn set_angle(&mut self, angle: T) -> Result<(), JointError> {
        match self.joint_type {
            JointType::Fixed => return Err(JointError::OutOfLimit),
            _ => {}
        }
        match self.limits.clone() {
            Some(range) => {
                if !range.is_valid(angle) {
                    return Err(JointError::OutOfLimit);
                }
            }
            None => {}
        }
        self.angle = angle;
        Ok(())
    }
    pub fn get_angle(&self) -> Option<T> {
        match self.joint_type {
            JointType::Fixed => None,
            _ => Some(self.angle),
        }
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
/// extern crate nalgebra as na;
/// extern crate nkinematics as nk;
/// let l0 = nk::LinkedJointBuilder::new()
///     .name("link1")
///     .translation(na::Translation3::new(0.0, 0.1, 0.0))
///     .joint("link_pitch", nk::JointType::Rotational{axis: na::Vector3::y_axis()})
///     .finalize();
/// println!("{:?}", l0);
/// ```
#[derive(Debug, Clone)]
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
