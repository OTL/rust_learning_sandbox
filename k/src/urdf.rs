extern crate nalgebra as na;
extern crate urdf_rs;
extern crate alga;

use links::*;
use rctree::*;
use rctree_links::*;

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


pub fn create_joint_with_link_from_urdf_joint<T>(joint: &urdf_rs::Joint) -> JointWithLink<T>
    where T: Real
{
    JointWithLinkBuilder::<T>::new()
        .joint(&joint.name,
               match joint.joint_type {
                   urdf_rs::JointType::Revolute |
                   urdf_rs::JointType::Continuous => {
                       JointType::Rotational { axis: axis_from(joint.axis.xyz) }
                   }
                   urdf_rs::JointType::Prismatic => {
                       JointType::Linear { axis: axis_from(joint.axis.xyz) }
                   }
                   _ => JointType::Fixed,
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

pub fn get_root_link_name(robot: &urdf_rs::Robot) -> String {
    let mut child_joint_map = HashMap::<&str, &urdf_rs::Joint>::new();
    for j in &robot.joints {
        if let Some(old) = child_joint_map.insert(&j.child.link, j) {
            warn!("old {:?} found", old);
        }
    }
    let mut parent_link_name: &str = &robot.links[0].name;
    while let Some(joint) = child_joint_map.get(&parent_link_name) {
        parent_link_name = &joint.parent.link;
    }
    parent_link_name.to_string()
}

pub fn create_robot<T>(robot: &urdf_rs::Robot) -> JointWithLinkStar<T>
    where T: Real
{
    // find end links
    let mut link_map = HashMap::new();
    for l in &robot.links {
        if let Some(old) = link_map.insert(&l.name, l) {
            warn!("old {:?} found", old);
        }
    }
    let mut child_joint_map = HashMap::<&str, &urdf_rs::Joint>::new();
    for j in &robot.joints {
        if let Some(old) = child_joint_map.insert(&j.child.link, j) {
            warn!("old {:?} found", old);
        }
    }

    for joint in &robot.joints {
        link_map.remove(&joint.parent.link);
    }
    // link_map contains end links only here
    JointWithLinkStar::new(&robot.name,
                           link_map
                               .keys()
                               .map(|end_name| {
                                        get_joint_until_root(end_name, &child_joint_map)
                                         .iter()
                                         .map(|urdf_joint| {
                                                  create_joint_with_link_from_urdf_joint(urdf_joint)
                                              })
                                         .collect()
                                    })
                               .map(|link_vec| JointWithLinkArray::new("", link_vec))
                               .collect())
}

pub fn create_tree<T>(robot: &urdf_rs::Robot) -> JointWithLinkTree<T>
    where T: Real
{
    let root_name = get_root_link_name(robot);
    let mut ref_nodes = Vec::new();
    let mut child_ref_map = HashMap::new();
    let mut parent_ref_map = HashMap::<&String, Vec<RefJointWithLinkNode<T>>>::new();

    let root_node = create_ref_node(JointWithLinkBuilder::<T>::new()
                                        .joint("root", JointType::Fixed)
                                        .name(&root_name)
                                        .finalize());
    for j in &robot.joints {
        let node = create_ref_node(create_joint_with_link_from_urdf_joint(j));
        child_ref_map.insert(&j.child.link, node.clone());
        if parent_ref_map.get(&j.parent.link).is_some() {
            parent_ref_map
                .get_mut(&j.parent.link)
                .unwrap()
                .push(node.clone());
        } else {
            parent_ref_map.insert(&j.parent.link, vec![node.clone()]);
        }
        ref_nodes.push(node);
    }
    for l in &robot.links {
        info!("link={}", l.name);
        if let Some(parent_node) = child_ref_map.get(&l.name) {
            if let Some(child_nodes) = parent_ref_map.get(&l.name) {
                for child_node in child_nodes.iter() {
                    info!("set paremt = {}, child = {}",
                          parent_node.borrow().data.get_joint_name(),
                          child_node.borrow().data.get_joint_name());
                    set_parent_child(parent_node, child_node);
                }
            }
        }
    }
    // set root as parent of root joint nodes
    let root_joint_nodes = ref_nodes
        .iter()
        .filter_map(|ref_node| match ref_node.borrow().parent {
                        None => Some(ref_node),
                        Some(_) => None,
                    });
    for rjn in root_joint_nodes {
        set_parent_child(&root_node, rjn);
    }
    // create root node..
    JointWithLinkTree::new(&robot.name, root_node)
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

    assert_eq!(get_root_link_name(&robo), "root".to_string());
}

#[test]
fn test_tree() {
    let robo = urdf_rs::read_file("sample.urdf").unwrap();
    assert_eq!(robo.name, "robo");
    assert_eq!(robo.links.len(), 1 + 6 + 6);

    let tree = create_tree::<f32>(&robo);
    assert_eq!(tree.map(&|ref_joint| ref_joint.clone()).len(), 13);
}
