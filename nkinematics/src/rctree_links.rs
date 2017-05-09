extern crate nalgebra as na;

use na::Isometry3;
use alga::general::Real;
use links::*;
use rctree::*;

pub type RefLinkedJointNode<T> = RefNode<LinkedJoint<T>>;
pub type LinkedJointNode<T> = Node<LinkedJoint<T>>;

pub struct RefKinematicChain<T: Real> {
    pub name: String,
    pub linked_joints: Vec<RefLinkedJointNode<T>>,
    pub transform: Isometry3<T>,
}

impl<T> RefKinematicChain<T>
    where T: Real
{
    pub fn new(name: &str, end: &RefLinkedJointNode<T>) -> Self {
        let mut links = map_ancestors(end, &|ljn| ljn.clone());
        links.reverse();
        RefKinematicChain {
            name: name.to_string(),
            linked_joints: links,
            transform: Isometry3::identity(),
        }
    }
}

impl<T> KinematicChain<T> for RefKinematicChain<T>
    where T: Real
{
    fn calc_end_transform(&self) -> Isometry3<T> {
        self.linked_joints
            .iter()
            .fold(self.transform,
                  |trans, ref ljn_ref| trans * ljn_ref.borrow().data.calc_transform())
    }
    fn set_joint_angles(&mut self, angles: &Vec<T>) -> Result<(), JointError> {
        // TODO: is it possible to cache the joint_with_angle to speed up?
        let mut joints_with_angle = self.linked_joints
            .iter_mut()
            .filter(|ref ljn_ref| ljn_ref.borrow().data.has_joint_angle())
            .collect::<Vec<_>>();
        if joints_with_angle.len() != angles.len() {
            return Err(JointError::SizeMisMatch);
        }
        for (i, ljn_ref) in joints_with_angle.iter_mut().enumerate() {
            try!(ljn_ref.borrow_mut().data.set_joint_angle(angles[i]));
        }
        Ok(())
    }
    fn get_joint_angles(&self) -> Vec<T> {
        self.linked_joints
            .iter()
            .filter_map(|ref ljn_ref| ljn_ref.borrow().data.get_joint_angle())
            .collect()
    }
}

pub struct LinkedJointTree<T: Real> {
    pub name: String,
    pub root_link: RefLinkedJointNode<T>,
}

impl<T: Real> LinkedJointTree<T> {
    pub fn new(name: &str, root_link: RefLinkedJointNode<T>) -> Self {
        LinkedJointTree {
            name: name.to_string(),
            root_link: root_link,
        }
    }
    pub fn calc_link_transforms(&self) -> Vec<Isometry3<T>> {
        self.map(&|ljn| {
            let parent_transform = match ljn.borrow().parent {
                Some(ref parent) => {
                    let rc_parent = parent.upgrade().unwrap().clone();
                    let parent_obj = rc_parent.borrow();
                    match parent_obj.data.world_transform_cache {
                        Some(trans) => trans,
                        None => Isometry3::identity(),
                    }
                }
                None => Isometry3::identity(),
            };
            let trans = parent_transform * ljn.borrow().data.calc_transform();
            ljn.borrow_mut().data.world_transform_cache = Some(trans.clone());
            trans
        })
    }
    pub fn map<F, K>(&self, func: &F) -> Vec<K>
        where F: Fn(&RefLinkedJointNode<T>) -> K
    {
        map_descendants(&self.root_link, func)
    }
}

#[test]
fn it_works() {
    let l0 = LinkedJointBuilder::new()
        .name("link1")
        .translation(na::Translation3::new(0.0, 0.1, 0.0))
        .joint("j0", JointType::Rotational { axis: na::Vector3::y_axis() })
        .finalize();
    let l1 = LinkedJointBuilder::new()
        .name("link1")
        .translation(na::Translation3::new(0.0, 0.1, 0.1))
        .joint("j1", JointType::Rotational { axis: na::Vector3::y_axis() })
        .finalize();
    let l2 = LinkedJointBuilder::new()
        .name("link1")
        .translation(na::Translation3::new(0.0, 0.1, 0.1))
        .joint("j2", JointType::Rotational { axis: na::Vector3::y_axis() })
        .finalize();
    let l3 = LinkedJointBuilder::new()
        .name("link3")
        .translation(na::Translation3::new(0.0, 0.1, 0.2))
        .joint("j3", JointType::Rotational { axis: na::Vector3::y_axis() })
        .finalize();
    let l4 = LinkedJointBuilder::new()
        .name("link4")
        .translation(na::Translation3::new(0.0, 0.1, 0.1))
        .joint("j4", JointType::Rotational { axis: na::Vector3::y_axis() })
        .finalize();
    let l5 = LinkedJointBuilder::new()
        .name("link5")
        .translation(na::Translation3::new(0.0, 0.1, 0.1))
        .joint("j5", JointType::Rotational { axis: na::Vector3::y_axis() })
        .finalize();

    let ljn0 = create_ref_node(l0);
    let ljn1 = create_ref_node(l1);
    let ljn2 = create_ref_node(l2);
    let ljn3 = create_ref_node(l3);
    let ljn4 = create_ref_node(l4);
    let ljn5 = create_ref_node(l5);
    set_parent_child(&ljn0, &ljn1);
    set_parent_child(&ljn1, &ljn2);
    set_parent_child(&ljn2, &ljn3);
    set_parent_child(&ljn0, &ljn4);
    set_parent_child(&ljn4, &ljn5);
    let names = map_descendants(&ljn0, &|ljn| ljn.borrow().data.get_joint_name().to_string());
    println!("{:?}", ljn0);
    println!("names = {:?}", names);
    let angles = map_descendants(&ljn0, &|ljn| ljn.borrow().data.get_joint_angle());
    println!("angles = {:?}", angles);

    let get_z = |ljn: &RefLinkedJointNode<f32>| match ljn.borrow().parent {
        Some(ref parent) => {
            (parent.borrow().data.calc_transform() * ljn.borrow().data.calc_transform())
                .translation
                .vector
                .z
        }
        None => {
            ljn.borrow()
                .data
                .calc_transform()
                .translation
                .vector
                .z
        }
    };

    let poses = map_descendants(&ljn0, &get_z);
    println!("poses = {:?}", poses);

    let _ = map_descendants(&ljn0, &|ljn| ljn.borrow_mut().data.set_joint_angle(-0.5));
    let angles = map_descendants(&ljn0, &|ljn| ljn.borrow().data.get_joint_angle());
    println!("angles = {:?}", angles);

    let poses = map_descendants(&ljn0, &get_z);
    println!("poses = {:?}", poses);

    let arm = RefKinematicChain::new("chain1", &ljn3);
    assert_eq!(arm.get_joint_angles().len(), 4);
    println!("{:?}", arm.get_joint_angles());
    println!("{:?}", arm.calc_end_transform());
}
