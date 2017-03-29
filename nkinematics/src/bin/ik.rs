extern crate kiss3d;
extern crate nkinematics;
extern crate alga;
use nkinematics::*;
use kiss3d::window::Window;
use kiss3d::scene::SceneNode;
use kiss3d::light::Light;
//use alga::general::Real;

extern crate nalgebra as na;
use na::{Vector3, Translation3};

fn create_linked_frame(name: &str) -> LinkedFrame<f32> {
    let l0 = LinkedJointBuilder::new()
        .name("shoulder_link1")
        .joint("shoulder_pitch", JointType::Rotational{axis: Vector3::y_axis()})
        .finalize();
    let l1 = LinkedJointBuilder::new()
        .name("shoulder_link2")
        .joint("shoulder_roll", JointType::Rotational{axis: Vector3::x_axis()})
        .translation(Translation3::new(0.0, 0.1, 0.0))
        .finalize();
    let l2 = LinkedJointBuilder::new()
        .name("shoulder_link3")
        .joint("shoulder_yaw", JointType::Rotational{axis: Vector3::z_axis()})
        .translation(Translation3::new(0.0, 0.0, -0.15))
        .finalize();
    let l3 = LinkedJointBuilder::new()
        .name("elbow_link1")
        .joint("elbow_pitch", JointType::Rotational{axis: Vector3::y_axis()})
        .translation(Translation3::new(0.0, 0.0, -0.15))
        .finalize();
    let l4 = LinkedJointBuilder::new()
        .name("wrist_link1")
        .joint("wrist_yaw", JointType::Rotational{axis: Vector3::z_axis()})
        .translation(Translation3::new(0.0, 0.0, -0.15))
        .finalize();
    let l5 = LinkedJointBuilder::new()
        .name("wrist_link2")
        .joint("wrist_pitch", JointType::Rotational{axis: Vector3::y_axis()})
        .translation(Translation3::new(0.0, 0.0, -0.15))
        .finalize();
    let mut lf1 = LinkedFrame::new(name);
    lf1.linked_joints = vec![l0, l1, l2, l3, l4, l5];
    lf1
}

fn create_cubes(window: &mut Window) -> Vec<SceneNode> {
    let mut c0 = window.add_cube(0.1, 0.1, 0.1);
    c0.set_color(1.0, 0.0, 1.0);
    let mut c1 = window.add_cube(0.1, 0.1, 0.1);
    c1.set_color(1.0, 0.0, 0.0);
    let mut c2 = window.add_cube(0.1, 0.1, 0.1);
    c2.set_color(0.0, 1.0, 0.0);
    let mut c3 = window.add_cube(0.1, 0.1, 0.1);
    c3.set_color(0.0, 0.5, 1.0);
    let mut c4 = window.add_cube(0.1, 0.1, 0.1);
    c4.set_color(1.0, 0.5, 1.0);
    let mut c5 = window.add_cube(0.1, 0.1, 0.1);
    c5.set_color(0.5, 0.0, 1.0);
    let mut c6 = window.add_cube(0.1, 0.1, 0.1);
    c6.set_color(0.0, 0.5, 0.2);
    vec![c0, c1, c2, c3, c4, c5, c6]
}

fn main() {
    let mut arm = create_linked_frame("arm");

    let mut window = Window::new("nkinematics ui");
    window.set_light(Light::StickToCamera);
    let mut cubes = create_cubes(&mut window);
    //let mut angles = vec!(0.5, 0.2, 0.0, -1.0, 1.0, 0.0);
    //let mut angles = vec!(0.0, 0.2, 0.0, -1.0, 1.0, 0.0);
    let mut angles = vec!(0.5, 0.2, 0.0, -1.0, 0.0, 0.0);
    //    let mut t = 0.0;
    let mut i = 0;
    let mut dir = 1.0;
    while window.render() && i < 6 {
        angles[i] += 0.1 * dir;
        if angles[i] > 1.5 { dir = -1.0; }
        if angles[i] < -1.5 { dir = 1.0; i += 1; }
        arm.set_joint_angles(&angles);
        for (i, trans) in arm.calc_transforms().iter().enumerate() {
            cubes[i].set_local_transformation(trans.clone());
        }
    }
}
