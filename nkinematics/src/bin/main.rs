extern crate kiss3d;
extern crate nkinematics;
extern crate alga;
use nkinematics::*;
use kiss3d::window::Window;
use kiss3d::scene::SceneNode;
use kiss3d::light::Light;
use alga::general::Real;

extern crate nalgebra as na;
use na::{Isometry3, Vector3, UnitQuaternion, Translation3};

fn create_linked_frame(name: &str) -> LinkedFrame<f32> {
    let j1 = Joint::new("lj1",
                            JointType::Rotational{axis: Vector3::x_axis()});
    let j2 = Joint::new("lj2",
                        JointType::Linear{axis: Vector3::y_axis()});
    let j3 = Joint::new("lj3",
                        JointType::Rotational{axis: Vector3::x_axis()});
    let mut linked_joint1 = LinkedJoint::new("llink1", j1);
    linked_joint1.transform = Isometry3::from_parts(
        Translation3::new(0.0, 0.2, 0.0),
        UnitQuaternion::identity());
    let mut linked_joint2 = LinkedJoint::new("link2", j2);
    linked_joint2.transform = Isometry3::from_parts(
        Translation3::new(0.0, 0.2, 0.0),
        UnitQuaternion::identity());
    let mut linked_joint3 = LinkedJoint::new("link2", j3);
    linked_joint3.transform = Isometry3::from_parts(
        Translation3::new(0.0, 0.2, 0.0),
        UnitQuaternion::identity());
    let mut lf1 = LinkedFrame::new(name);
    lf1.linked_joints = vec![linked_joint1, linked_joint2, linked_joint3];
    lf1
}

fn create_cubes(window: &mut Window) -> Vec<SceneNode> {
    let mut c1 = window.add_cube(0.1, 0.1, 0.1);
    c1.set_color(1.0, 0.0, 0.0);
    let mut c2 = window.add_cube(0.1, 0.1, 0.1);
    c2.set_color(0.0, 1.0, 0.0);
    let mut c3 = window.add_cube(0.1, 0.1, 0.1);
    c3.set_color(0.0, 0.0, 1.0);
    let mut c4 = window.add_cube(0.1, 0.1, 0.1);
    c4.set_color(1.0, 0.0, 1.0);
    vec![c1, c2, c3, c4]
}

fn main() {
    let mut lleg = create_linked_frame("left_leg");
    lleg.transform = Isometry3::from_parts(
        Translation3::new(0.2, 0.2, 0.0),
        UnitQuaternion::identity());

    let mut rleg = create_linked_frame("right_leg");
    rleg.transform = Isometry3::from_parts(
        Translation3::new(-0.2, 0.2, 0.0),
        UnitQuaternion::identity());
    let mut larm = create_linked_frame("left_arm");
    larm.transform = Isometry3::from_parts(
        Translation3::new(0.2, -0.2, 0.0),
        UnitQuaternion::identity());
    let mut rarm = create_linked_frame("right_arm");
    rarm.transform = Isometry3::from_parts(
        Translation3::new(-0.2, -0.2, 0.0),
        UnitQuaternion::identity());

    let mut rf = RobotFrame::new("robo", vec![lleg, rleg, larm, rarm]);

    let mut window = Window::new("nkinematics ui");
    window.set_light(Light::StickToCamera);
    let mut cubes = vec![create_cubes(&mut window),
                         create_cubes(&mut window),
                         create_cubes(&mut window),
                         create_cubes(&mut window)];
    let mut root_cube = window.add_cube(0.2, 0.2, 0.2);

    let mut angles = vec!(0.0, 0.0, 0.0);
    let mut t = 0.0;
    while window.render() {
        t += 0.1;
        angles[0] = t.sin();
        angles[1] = t.cos() * 0.05;
        angles[2] = t.sin();
        for fr in &mut rf.frames {
            fr.set_joint_angles(&angles);
        }
        rf.transform = Isometry3::from_parts(
            Translation3::new(0.0, 0.1 * t.sin(), 0.0),
            UnitQuaternion::from_euler_angles(3.14, 0.0, 0.0));
        for (i, trans) in rf.calc_transforms().iter().enumerate() {
            for j in 0..4 {
                cubes[i][j].set_local_transformation(trans[j]);
            }
        }
        root_cube.set_local_transformation(rf.transform);
    }
}
