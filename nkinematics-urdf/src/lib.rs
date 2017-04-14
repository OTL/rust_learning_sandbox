extern crate nalgebra as na;
extern crate nkinematics as nk;
extern crate urdf_rs;
extern crate alga;

use alga::general::Real;
use std::collections::HashMap;

pub fn create_axis_from_vec3<T>(vec3: [f64; 3]) -> na::Unit<na::Vector3<T>>
    where T: Real
{
    na::Unit::<_>::new_normalize(na::Vector3::new(na::convert(vec3[0]),
                                                  na::convert(vec3[1]),
                                                  na::convert(vec3[2])))
}

pub fn create_linked_joint_from_urdf_joint<T>(joint: &urdf_rs::Joint) -> nk::LinkedJoint<T>
    where T: Real
{
    nk::LinkedJointBuilder::<T>::new()
        .joint(&joint.name,
               match joint.joint_type {
                   urdf_rs::JointType::Revolute => {
                       nk::JointType::Rotational {
                           axis: create_axis_from_vec3(joint.axis.xyz_as_array()),
                       }
                   }
                   urdf_rs::JointType::Prismatic => {
                       nk::JointType::Linear {
                           axis: create_axis_from_vec3(joint.axis.xyz_as_array()),
                       }
                   }
                   _ => nk::JointType::Fixed,
               })
        .name(&joint.child.link)
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

pub fn create_serial_linked_joints_vec<T>(robot: &urdf_rs::Robot) -> Vec<Vec<nk::LinkedJoint<T>>>
    where T: Real
{
    // find end links
    let mut link_map = HashMap::new();
    for l in robot.links.iter() {
        link_map.insert(&l.name, l);
    }
    let mut child_joint_map = HashMap::<&str, &urdf_rs::Joint>::new();
    for j in robot.joints.iter() {
        child_joint_map.insert(&j.child.link, j);
    }
    for joint in robot.joints.iter() {
        link_map.remove(&joint.parent.link);
    }
    // link_map contains end links only here
    link_map.keys()
        .map(|end_name| {
                 get_joint_until_root(&end_name, &child_joint_map)
                     .iter()
                     .map(|urdf_joint| create_linked_joint_from_urdf_joint(urdf_joint))
                     .collect()
             })
        .collect()
}

#[test]
fn it_works() {
    let robo = urdf_rs::read_file("sample.urdf").unwrap();
    assert_eq!(robo.name, "robo");
    assert_eq!(robo.links.len(), 7);
    let linked_joints = robo.joints
        .iter()
        .map(|joint| create_linked_joint_from_urdf_joint::<f32>(joint))
        .collect();
    let mut lf1 = nk::LinkedFrame::new(&robo.name);
    lf1.linked_joints = linked_joints;

    let lj_vec_vec = create_serial_linked_joints_vec::<f32>(&robo);
    assert_eq!(lj_vec_vec.len(), 1);
    assert_eq!(lj_vec_vec[0].len(), 6);
}
