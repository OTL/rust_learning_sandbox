extern crate kiss3d;
extern crate nkinematics;
extern crate alga;
use nkinematics::*;
extern crate glfw;
use glfw::{Action, WindowEvent, Key};

use kiss3d::window::Window;
use kiss3d::scene::SceneNode;
use kiss3d::light::Light;
use kiss3d::camera::ArcBall;
//use alga::general::Real;
//use alga::general::Inverse; // for inverse
//use na::{Isometry3, Vector3, Unit, UnitQuaternion, Translation3};
extern crate nalgebra as na;
use na::{Isometry3, Vector3, Translation3, UnitQuaternion, Vector6, Point3};

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
        .translation(Translation3::new(0.0, 0.0, -0.30))
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
    let orig_angles_vec = Vector6::new(0.5, 0.2, 0.0, -1.0, 0.0, 0.0);
    let angles = vec![orig_angles_vec[0], orig_angles_vec[1], orig_angles_vec[2],
                      orig_angles_vec[3], orig_angles_vec[4], orig_angles_vec[5]];
    arm.set_joint_angles(&angles);
    let mut target = Isometry3::from_parts(
        Translation3::new(0.40, 0.2, -0.3),
        UnitQuaternion::from_euler_angles(0.0, -1.0, 0.0));

    let mut c_t = window.add_sphere(0.05);
    c_t.set_color(1.0, 0.2, 0.2);
//    let eye              = Point3::new(2.0f32, 1.0, 2.0);
//    let at               = Point3::origin();
//    let mut arc_ball     = ArcBall::new(eye, at);
//    arc_ball.set_pitch(1.57);
//    let mut i = 0;
    //    while window.render_with_camera(&mut arc_ball) {
    while window.render() {
//        let curr_yaw = arc_ball.yaw();
//        arc_ball.set_yaw(curr_yaw + 0.05);
        for mut event in window.events().iter() {
            match event.value {
                WindowEvent::Key(code, _, Action::Release, _) => {
                    match code {
                        Key::P => { target.translation.vector[2] -= 0.1; },
                        Key::N => { target.translation.vector[2] += 0.1; },
                        Key::F => { target.translation.vector[0] += 0.1; },
                        Key::B => { target.translation.vector[0] -= 0.1; },
                        _ => {},
                    }
                    event.inhibited = true // override the default keyboard handler
                },
                _ => {}

            }
        }
        let dist = arm.solve_inverse_kinematics_one_loop(&target, 0.001);
        c_t.set_local_transformation(target.clone());
        for (i, trans) in arm.calc_transforms().iter().enumerate() {
            cubes[i].set_local_transformation(trans.clone());
        }
    }

}
