extern crate nalgebra as na;

use alga::general::Real;
use links::*;
use std::cell::RefCell;
use std::rc::{Rc, Weak};


#[derive(Debug, Clone)]
pub struct LinkedJointNode<T: Real> {
    parent: Option<Rc<RefCell<LinkedJointNode<T>>>>,
    children: Vec<Weak<RefCell<LinkedJointNode<T>>>>,
    linked_joint: LinkedJoint<T>,
}

impl<T> LinkedJointNode<T> where T: Real {
    pub fn new(linked_joint: LinkedJoint<T>) -> LinkedJointNode<T> {
        LinkedJointNode {
            parent: None,
            children: Vec::new(),
            linked_joint: linked_joint,
        }
    }
    pub fn get_name(&self) -> &str {
        &self.linked_joint.get_joint_name()
    }
}

pub fn set_parent_child<T>(parent: &Rc<RefCell<LinkedJointNode<T>>>,
                           child: &Rc<RefCell<LinkedJointNode<T>>>)
    where T: Real {
    child.borrow_mut().parent = Some(parent.clone());
    parent.borrow_mut().children.push(Rc::downgrade(child));
}

// todo: move vec to iter?
pub fn map_descendants<T, F, K>(root: &Rc<RefCell<LinkedJointNode<T>>>, func: &F) -> Vec<K>
    where F: Fn(&LinkedJointNode<T>) -> K,
          T: Real {
    let mut ret = Vec::new();
    map_descendants_internal(root, func, &mut ret);
    ret
}

// can be removed..
fn map_descendants_internal<T, F, K>(root: &Rc<RefCell<LinkedJointNode<T>>>, func: &F, ret: &mut Vec<K>)
    where F: Fn(&LinkedJointNode<T>) -> K,
          T: Real {
    ret.push(func(&*root.borrow()));
    for c in &root.borrow().children {
        map_descendants_internal(&Weak::upgrade(c).unwrap(), func, ret);
    }
}

// todo: move vec to iter?
pub fn map_descendants_mut<T, F, K>(root: &Rc<RefCell<LinkedJointNode<T>>>, func: &F) -> Vec<K>
    where F: Fn(&mut LinkedJointNode<T>) -> K,
          T: Real {
    let mut ret = Vec::new();
    map_descendants_internal_mut(root, func, &mut ret);
    ret
}

// can be removed..
fn map_descendants_internal_mut<T, F, K>(root: &Rc<RefCell<LinkedJointNode<T>>>, func: &F, ret: &mut Vec<K>)
    where F: Fn(&mut LinkedJointNode<T>) -> K,
          T: Real {
    ret.push(func(&mut *root.borrow_mut()));
    for c in &root.borrow_mut().children {
        map_descendants_internal_mut(&Weak::upgrade(c).unwrap(), func, ret);
    }
}

pub fn map_ancestors<T, F, K>(end: &Rc<RefCell<LinkedJointNode<T>>>, func: &F) -> Vec<K>
    where F: Fn(&LinkedJointNode<T>) -> K,
          T: Real {
    let mut ret = Vec::new();
    ret.push(func(&*end.borrow()));
    while let Some(ref parent) = end.borrow().parent {
        ret.push(func(&*parent.borrow()))
    }
    ret
}

pub fn map_ancestors_mut<T, F, K>(end: &Rc<RefCell<LinkedJointNode<T>>>, func: &F) -> Vec<K>
    where F: Fn(&mut LinkedJointNode<T>) -> K,
          T: Real {
    let mut ret = Vec::new();
    ret.push(func(&mut *end.borrow_mut()));
    while let Some(ref parent) = end.borrow_mut().parent {
        ret.push(func(&mut *parent.borrow_mut()))
    }
    ret
}

pub struct RefKinematicChain<T: Real> {
    pub name: String,
    pub linked_joints: Vec<Rc<RefCell<LinkedJointNode<T>>>>,
    pub transform: Isometry3<T>,
}

impl<T> RefKinematicChain<T> where T: Real> {
    pub fn new(name: &str, end: Rc<RefCell<LinkedJointNode<T>>>) -> Self {
        RefKinematicChain {
            name: name.to_string(),
            linked_joints: map_ancestors(end, &|ljn| ljn.clone()),
            transform: Isometry3::identity(),
        }
    }
    pub fn map<F, K>(&self, func: &F) -> Vec<K>
        where F: Fn(&LinkedJointNode<T>) -> K {
        map_descendants(&self.root)
    }
    pub fn map_mut<F, K>(&mut self, func: &F) -> Vec<K>
        where F: Fn(&mut LinkedJointNode<T>) -> K {
        map_descendants_mut(&mut self.root)
    }
}

impl<T> KinematicChain<T> for RefKinematicChain<T> where T: Real {
    fn calc_end_transform(&self) -> Isometry3<T> {
        self.linked_joints
            .iter()
            .fold(self.transform, |trans, ref ljn_ref| trans * ljn_ref.borrow().linked_joint.calc_transform())
    }
    fn set_joint_angles(&mut self, angles: &Vec<T>) -> Result<(), JointError> {
        // TODO: is it possible to cache the joint_with_angle to speed up?
        let mut joints_with_angle = self.linked_joints
            .iter_mut()
            .filter(|ref ljn_ref| ljn_ref.borrow().linked_joint.has_joint_angle())
            .collect::<Vec<_>>();
        if joints_with_angle.len() != angles.len() {
            return Err(JointError::SizeMisMatch);
        }
        for (i, ljn_ref) in joints_with_angle.iter_mut().enumerate() {
            try!(ljn_ref.borrow_mut().linked_joint.set_joint_angle(angles[i]));
        }
        Ok(())
    }
    fn get_joint_angles(&self) -> Vec<T> {
        self.linked_joints
            .iter()
            .filter_map(|ref ljn_ref| ljn_ref.borrow().linked_joint.get_joint_angle())
            .collect()
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
        .translation(na::Translation3::new(0.0, 0.1, 0.1))
        .joint("j1", JointType::Rotational{axis: na::Vector3::y_axis()})
        .finalize();
    let l2 = LinkedJointBuilder::new()
        .name("link1")
        .translation(na::Translation3::new(0.0, 0.1, 0.1))
        .joint("j2", JointType::Rotational{axis: na::Vector3::y_axis()})
        .finalize();
    let l3 = LinkedJointBuilder::new()
        .name("link3")
        .translation(na::Translation3::new(0.0, 0.1, 0.2))
        .joint("j3", JointType::Rotational{axis: na::Vector3::y_axis()})
        .finalize();
    let l4 = LinkedJointBuilder::new()
        .name("link4")
        .translation(na::Translation3::new(0.0, 0.1, 0.1))
        .joint("j4", JointType::Rotational{axis: na::Vector3::y_axis()})
        .finalize();
    let l5 = LinkedJointBuilder::new()
        .name("link5")
        .translation(na::Translation3::new(0.0, 0.1, 0.1))
        .joint("j5", JointType::Rotational{axis: na::Vector3::y_axis()})
        .finalize();

    let ljn0 = Rc::new(RefCell::new(LinkedJointNode::new(l0)));
    let ljn1 = Rc::new(RefCell::new(LinkedJointNode::new(l1)));
    let ljn2 = Rc::new(RefCell::new(LinkedJointNode::new(l2)));
    let ljn3 = Rc::new(RefCell::new(LinkedJointNode::new(l3)));
    let ljn4 = Rc::new(RefCell::new(LinkedJointNode::new(l4)));
    let ljn5 = Rc::new(RefCell::new(LinkedJointNode::new(l5)));
    set_parent_child(&ljn0, &ljn1);
    set_parent_child(&ljn1, &ljn2);
    set_parent_child(&ljn2, &ljn3);
    set_parent_child(&ljn0, &ljn4);
    set_parent_child(&ljn4, &ljn5);
    let names = map_descendants(&ljn0, &|ljn| ljn.get_name().to_string());
    println!("{:?}", ljn0);
    println!("names = {:?}", names);
    let angles = map_descendants(&ljn0, &|ljn| ljn.linked_joint.get_joint_angle());
    println!("angles = {:?}", angles);

    let get_z = |ljn: &LinkedJointNode<f32> | match ljn.parent {
        Some(ref parent) => (parent.borrow().linked_joint.calc_transform() *
                             ljn.linked_joint.calc_transform()).translation.vector.z,
        None => ljn.linked_joint.calc_transform().translation.vector.z,
    };

    let poses = map_descendants(&ljn0, &get_z);
    println!("poses = {:?}", poses);

    let _ = map_descendants_mut(&ljn0, &|mut ljn| ljn.linked_joint.set_joint_angle(-0.5));
    let angles = map_descendants(&ljn0, &|ljn| ljn.linked_joint.get_joint_angle());
    println!("angles = {:?}", angles);

    let poses = map_descendants(&ljn0, &get_z);
    println!("poses = {:?}", poses);

    //     Option<Rc<RefCell<LinkedJointNode<T>>>>,
//    children: Vec<Rc<RefCell<LinkedJointNode<T>>>>,
//        linked_joint: LinkedJoint<T>,
}
