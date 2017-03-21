//use std::rc::Rc;
//use std::cell::RefCell;

//use na::{Isometry3, Translation3, UnitQuaternion, Point2, Vector3, Point3};

// node -> link
// edge -> joint

// based on the code below
// https://github.com/nikomatsakis/simple-graph/blob/master/src/lib.rs

pub struct Frame {
    name: String,
    links: Vec<LinkData>,
    joints: Vec<JointData>,
}

pub type LinkIndex = usize;

pub struct LinkData {
    name: String,
    first_child_joint: Option<JointIndex>,
}

pub type JointIndex = usize;

pub struct JointData {
    name: String,
    child_link: LinkIndex,
    sister_joint: Option<JointIndex>,
}

impl Frame {
    pub fn new(name: &str) -> Frame {
        Frame { name: name.to_string(),
                joints: Vec::new(),
                links: Vec::new(),
        }
    }
    pub fn add_link(&mut self, name: &str) -> LinkIndex {
        let index = self.links.len();
        self.links.push(LinkData { name: name.to_string(),
                                   first_child_joint: None });
        index
    }

    pub fn add_joint(&mut self, name: &str,
                     parent: LinkIndex, child: LinkIndex) {
        let joint_index = self.joints.len();
        let parent_link_data = &mut self.links[parent];
        self.joints.push(JointData {
            name: name.to_string(),
            child_link: child,
            sister_joint: parent_link_data.first_child_joint,
        });
        parent_link_data.first_child_joint = Some(joint_index)
    }

    pub fn successors(&self, from: LinkIndex) -> Successors {
        let first_child_joint = self.links[from].first_child_joint;
        Successors { graph: self, current_joint_index: first_child_joint }
    }
}

pub struct Successors<'graph> {
    graph: &'graph Frame,
    current_joint_index: Option<JointIndex>,
}

impl<'graph> Iterator for Successors<'graph> {
    type Item = LinkIndex;

    fn next(&mut self) -> Option<LinkIndex> {
        match self.current_joint_index {
            None => None,
            Some(joint_num) => {
                let joint = &self.graph.joints[joint_num];
                self.current_joint_index = joint.sister_joint;
                Some(joint.child_link)
            }
        }
    }
}


mod test {
    #[test]
    fn it_works() {
        use super::*;
        let mut robot = Frame::new("robot");
        let root = robot.add_link("root_link");
        let larm1 = robot.add_link("larm1");
        let larm2 = robot.add_link("larm2");
        let larm3 = robot.add_link("larm3");
        let rarm1 = robot.add_link("rarm1");
        let rarm2 = robot.add_link("rarm2");
        let rarm3 = robot.add_link("rarm3");
        robot.add_joint("l_shoulder", root, larm1);
        robot.add_joint("l_elbow", larm1, larm2);
        robot.add_joint("l_wrist", larm2, larm3);
        robot.add_joint("r_shoulder", root, rarm1);
        robot.add_joint("r_elbow", rarm1, rarm2);
        robot.add_joint("r_wrist", rarm2, rarm3);
        let successors_all: Vec<LinkIndex> = robot.successors(root).collect();
        assert_eq!(&successors_all[..], &[rarm1, larm1]);
        for ind in robot.successors(root) {
            println!("{}", robot.links[ind].name);
        }
        let successors_rarm: Vec<LinkIndex> = robot.successors(rarm1).collect();
        assert_eq!(&successors_rarm[..], &[rarm2]);
    }
}
