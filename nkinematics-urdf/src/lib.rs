extern crate nalgebra as na;
extern crate nkinematics as nk;
extern crate urdf_rs;
extern crate alga;

use alga::general::Real;
use std::collections::HashMap;

pub fn axis_from<T>(array3: [f64; 3]) -> na::Unit<na::Vector3<T>>
    where T: Real
{
    na::Unit::<_>::new_normalize(na::Vector3::new(na::convert(array3[0]),
                                                  na::convert(array3[1]),
                                                  na::convert(array3[2])))
}

pub fn quaternion_from<T>(array3: [f64; 3]) -> na::UnitQuaternion<T>
    where T: Real
{
    na::UnitQuaternion::from_euler_angles(na::convert(array3[0]),
                                          na::convert(array3[1]),
                                          na::convert(array3[2]))
}

pub fn translation_from<T>(array3: [f64; 3]) -> na::Translation3<T>
    where T: Real
{
    na::Translation3::new(na::convert(array3[0]),
                          na::convert(array3[1]),
                          na::convert(array3[2]))
}


pub fn create_linked_joint_from_urdf_joint<T>(joint: &urdf_rs::Joint) -> nk::LinkedJoint<T>
    where T: Real
{
    nk::LinkedJointBuilder::<T>::new()
        .joint(&joint.name,
               match joint.joint_type {
                   urdf_rs::JointType::Revolute => {
                       nk::JointType::Rotational { axis: axis_from(joint.axis.xyz) }
                   }
                   urdf_rs::JointType::Prismatic => {
                       nk::JointType::Linear { axis: axis_from(joint.axis.xyz) }
                   }
                   _ => nk::JointType::Fixed,
               })
        .name(&joint.child.link)
        .rotation(quaternion_from(joint.origin.rpy))
        .translation(translation_from(joint.origin.xyz))
        .finalize()
}

fn get_joint_until_root<'a, 'b>(end_name: &'a str,
                                parent_joint_map: &'b HashMap<&str, &urdf_rs::Joint>)
                                -> Vec<&'b urdf_rs::Joint> {
    let mut ret = Vec::new();
    let mut parent_link_name = end_name;
    while let Some(joint) = parent_joint_map.get(&parent_link_name) {
        ret.push(*joint);
        parent_link_name = &joint.parent.link;
    }
    ret.reverse();
    ret
}

pub fn create_robot<T>(robot: &urdf_rs::Robot) -> nk::RobotFrame<T>
    where T: Real
{
    // find end links
    let mut link_map = HashMap::new();
    for l in robot.links.iter() {
        match link_map.insert(&l.name, l) {
            Some(old) => println!("old {:?} found", old),
            None => {}
        }
    }
    let mut child_joint_map = HashMap::<&str, &urdf_rs::Joint>::new();
    for j in robot.joints.iter() {
        match child_joint_map.insert(&j.child.link, j) {
            Some(old) => println!("old {:?} found", old),
            None => {}
        }
    }
    for joint in robot.joints.iter() {
        link_map.remove(&joint.parent.link);
    }
    // link_map contains end links only here
    nk::RobotFrame::new(
        &robot.name,
        link_map
            .keys()
            .map(|end_name| {
                get_joint_until_root(&end_name, &child_joint_map)
                    .iter()
                    .map(|urdf_joint| create_linked_joint_from_urdf_joint(urdf_joint))
                    .collect()
            })
            .map(|link_vec| nk::LinkedFrame::new("", link_vec))
            .collect())
}

#[test]
fn it_works() {
    let robo = urdf_rs::read_file("sample.urdf").unwrap();
    assert_eq!(robo.name, "robo");
    assert_eq!(robo.links.len(), 1 + 6 + 6);

    let rf = create_robot::<f32>(&robo);
    assert_eq!(rf.frames.len(), 2);
    assert_eq!(rf.frames[0].len(), 6);
    assert_eq!(rf.frames[1].len(), 6);
}
