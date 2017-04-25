extern crate nalgebra as na;

use alga::general::Real;
use na::Isometry3;
use links::*;

use std::collections::HashMap;

#[derive(Debug)]
pub struct KinematicTree<T: Real> {
    pub linked_joints: HashMap<String, LinkedJointNode<T>>,
    link_relations: Vec<(String, String)>,
    is_finalized: bool,
    root: String,
}

impl<T> KinematicTree<T> where T: Real {
    pub fn new() -> KinematicTree<T> {
        KinematicTree {
            linked_joints: HashMap::new(),
            link_relations: Vec::new(),
            is_finalized: false,
            root: "".to_string(),
        }
    }
    pub fn add_link(mut self, parent_name: Option<String>,
                     linked_joint: LinkedJoint<T>) -> Self {
        let name = linked_joint.get_joint_name().to_string();
        match parent_name {
            Some(ref parent) => self.link_relations.push((parent.to_string(), name.clone())),
            None => {},
        }
        self.linked_joints.insert(
            name,
            LinkedJointNode {
                parent: parent_name,
                children: Vec::new(),
                linked_joint: linked_joint,
                world_transform: Isometry3::identity(),
            });
        self
    }
    pub fn get_roots(&self) -> Vec<String> {
        self.linked_joints.values().filter_map(|ljn|
                                          match ljn.parent {
                                              Some(_) => None,
                                              None => Some(ljn.get_name().to_string()),
                                          }).collect()
    }
    pub fn create_link_vec(&self, root_name: &str, end_name: &str) -> Option<Vec<LinkedJoint<T>>> {
        let mut ret = Vec::new();
        let mut next_node = self.get(end_name);
        while let Some(node) = next_node {
            ret.push(node.linked_joint.clone());
            if node.get_name() == root_name {
                next_node = None;
            } else {
                match node.parent {
                    Some(ref parent) => next_node = self.get(&parent),
                    None => return None,
                }
            }
        }
        ret.reverse();
        Some(ret)
    }
    pub fn all_descendants<K, F>(&self, func: &F) -> Vec<K>
        where F: Fn(&LinkedJointNode<T>) -> K {
        self.descendants(&self.root, func)
    }
    pub fn descendants<K, F>(&self, name: &str, func: &F) -> Vec<K>
        where F: Fn(&LinkedJointNode<T>) -> K {
        let mut ret = Vec::new();
        self.descendants_internal(name, func, &mut ret);
        ret
    }
    pub fn descendants_internal<K, F>(&self, name: &str, func: &F, ret: &mut Vec<K>)
        where F: Fn(&LinkedJointNode<T>) -> K {
        match self.get(name) {
            None => {},
            Some(node) => {
                ret.push(func(node));
                for child in &node.children {
                    self.descendants_internal(&child, func, ret);
                }
            }
        };
    }

    pub fn all_descendants_mut<K, F>(&mut self, func: &F) -> Vec<K>
        where F: FnMut(&mut LinkedJointNode<T>) -> K {
        self.descendants_mut(&self.root, func)
    }
    pub fn descendants_mut<K, F>(&mut self, name: &str, func: &F) -> Vec<K>
        where F: FnMut(&mut LinkedJointNode<T>) -> K {
        let mut ret = Vec::new();
        self.descendants_internal_mut(name, func, &mut ret);
        ret
    }
    pub fn descendants_internal_mut<K, F>(&mut self, name: &str, func: &F, ret: &mut Vec<K>)
        where F: FnMut(&mut LinkedJointNode<T>) -> K {
        match self.get_mut(name) {
            None => {},
            Some(node) => {
                ret.push(func(node));
                for child in &node.children {
                    self.descendants_internal_mut(&child, func, ret);
                }
            }
        };
    }

    pub fn finalize(mut self) -> Self {
        for iter in self.link_relations.iter() {
            self.linked_joints.get_mut(&iter.0).unwrap()
                .children.push(iter.1.clone());
        }
        self.is_finalized = true;
        let mut roots = self.get_roots();
        // root must be 1
        assert_eq!(roots.len(), 1);
        self.root = roots.pop().unwrap();
        self
    }
    pub fn get(&self, name: &str) -> Option<&LinkedJointNode<T>> {
        self.linked_joints.get(name)
    }
    pub fn get_mut(&mut self, name: &str) -> Option<&mut LinkedJointNode<T>> {
        self.linked_joints.get_mut(name)
    }
//    pub fn calc_descendants_transformations(&self, name: &str) -> Vec<Isometry3> {
//    }
}

#[derive(Debug, Clone)]
pub struct LinkedJointNode<T: Real> {
    pub parent: Option<String>,
    pub children: Vec<String>,
    pub linked_joint: LinkedJoint<T>,
    pub world_transform: Isometry3<T>,
}

impl<T> LinkedJointNode<T> where T: Real {
    pub fn get_name(&self) -> &str {
        &self.linked_joint.get_joint_name()
    }
}

#[test]
fn it_works() {
    let l0 = LinkedJointBuilder::new()
        .name("link1")
        .translation(na::Translation3::new(0.0, 0.1, 0.0))
        .joint("j0", JointType::Rotational{axis: na::Vector3::y_axis()})
        .finalize();
    let l1 = LinkedJointBuilder::new()
        .name("link1")
        .translation(na::Translation3::new(0.0, 0.1, 0.0))
        .joint("j1", JointType::Rotational{axis: na::Vector3::y_axis()})
        .finalize();
    let l2 = LinkedJointBuilder::new()
        .name("link1")
        .translation(na::Translation3::new(0.0, 0.1, 0.0))
        .joint("j2", JointType::Rotational{axis: na::Vector3::y_axis()})
        .finalize();
    let l3 = LinkedJointBuilder::new()
        .name("link3")
        .translation(na::Translation3::new(0.0, 0.1, 0.0))
        .joint("j3", JointType::Rotational{axis: na::Vector3::y_axis()})
        .finalize();
    let l4 = LinkedJointBuilder::new()
        .name("link4")
        .translation(na::Translation3::new(0.0, 0.1, 0.0))
        .joint("j4", JointType::Rotational{axis: na::Vector3::y_axis()})
        .finalize();
    let l5 = LinkedJointBuilder::new()
        .name("link5")
        .translation(na::Translation3::new(0.0, 0.1, 0.0))
        .joint("j5", JointType::Rotational{axis: na::Vector3::y_axis()})
        .finalize();
    let mut tree = KinematicTree::new()
        .add_link(None, l0)
        .add_link(Some("j0".to_string()), l1)
        .add_link(Some("j1".to_string()), l2)
        .add_link(Some("j0".to_string()), l3)
        .add_link(Some("j2".to_string()), l4)
        .add_link(Some("j3".to_string()), l5)
        .finalize();
    println!("j0->{:?}", tree.descendants("j0", &|ljn| ljn.get_name().to_string()));
    println!("j3->{:?}", tree.descendants("j3", &|ljn| ljn.get_name().to_string()));
    println!("j1={:?}", tree.get("j1").unwrap());
    println!("root={:?}", tree.root);
    let all_names = tree.all_descendants(&|ljn| ljn.get_name().to_string());
    assert_eq!(all_names.len(), 6);
    println!("all = {:?}", all_names);
    let link_vec = tree.create_link_vec("j0", "j4").unwrap();
    assert_eq!(link_vec.len(), 4);
    println!("{:?}", link_vec);
    println!("{:?}", tree.get("j0").unwrap().children);
    tree.get_mut("j0").unwrap().linked_joint.set_joint_angle(1.0).unwrap();
    let all_poses = tree.all_descendants_mut(
        &|mut ljn|
        ljn.world_transform =
            match ljn.parent {
                None => Isometry3::identity(), // root
                Some(ref parent_name) => tree.get(parent_name).unwrap().world_transform
            } * ljn.linked_joint.calc_transform());
    println!("poses = {:?}", all_poses);
//    let angles = tree.all_descendants().iter().filter_map(|name| tree.get(&name).unwrap().linked_joint.get_joint_angle()).collect::<Vec<_>>();
//    println!("{:?}", angles);
    // update transform
    }
