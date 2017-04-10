extern crate alga;
extern crate glfw;
extern crate kiss3d;
extern crate nalgebra as na;
extern crate nkinematics;

use glfw::{Action, WindowEvent, Key};
use kiss3d::camera::ArcBall;
use kiss3d::light::Light;
use kiss3d::scene::SceneNode;
use kiss3d::window::Window;
use na::{Isometry3, Vector3, Translation3, UnitQuaternion, Point3};
use nkinematics::*;

fn create_linked_frame(name: &str) -> LinkedFrame<f32> {
    let l0 = LinkedJointBuilder::new()
        .name("shoulder_link1")
        .joint("shoulder_pitch",
               JointType::Rotational { axis: Vector3::y_axis() })
        .finalize();
    let l1 = LinkedJointBuilder::new()
        .name("shoulder_link2")
        .joint("shoulder_roll",
               JointType::Rotational { axis: Vector3::x_axis() })
        .translation(Translation3::new(0.0, 0.1, 0.0))
        .finalize();
    let l2 = LinkedJointBuilder::new()
        .name("shoulder_link3")
        .joint("shoulder_yaw",
               JointType::Rotational { axis: Vector3::z_axis() })
        .translation(Translation3::new(0.0, 0.0, -0.30))
        .finalize();
    let l3 = LinkedJointBuilder::new()
        .name("elbow_link1")
        .joint("elbow_pitch",
               JointType::Rotational { axis: Vector3::y_axis() })
        .translation(Translation3::new(0.0, 0.0, -0.15))
        .finalize();
    let l4 = LinkedJointBuilder::new()
        .name("wrist_link1")
        .joint("wrist_yaw",
               JointType::Rotational { axis: Vector3::z_axis() })
        .translation(Translation3::new(0.0, 0.0, -0.15))
        .finalize();
    let l5 = LinkedJointBuilder::new()
        .name("wrist_link2")
        .joint("wrist_pitch",
               JointType::Rotational { axis: Vector3::y_axis() })
        .translation(Translation3::new(0.0, 0.0, -0.15))
        .finalize();
    let l6 = LinkedJointBuilder::new()
        .name("wrist_link3")
        .joint("wrist_roll",
               JointType::Rotational { axis: Vector3::x_axis() })
        .translation(Translation3::new(0.0, 0.0, -0.10))
        .finalize();
    let mut lf1 = LinkedFrame::new(name);
    lf1.linked_joints = vec![l0, l1, l2, l3, l4, l5, l6];
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
    let mut c7 = window.add_cube(0.1, 0.1, 0.1);
    c7.set_color(0.5, 0.5, 0.2);
    vec![c0, c1, c2, c3, c4, c5, c6, c7]
}

fn main() {
    let mut arm = create_linked_frame("arm");

    let mut window = Window::new("nkinematics ui");
    window.set_light(Light::StickToCamera);
    let mut cubes = create_cubes(&mut window);
    let angles = vec![0.5, 0.2, 0.0, -1.0, 0.0, 0.0, 0.0];
    arm.set_joint_angles(&angles).unwrap();
    let mut target = Isometry3::from_parts(Translation3::new(0.40, 0.2, -0.3),
                                           UnitQuaternion::from_euler_angles(0.0, -1.0, 0.0));

    let mut c_t = window.add_sphere(0.05);
    c_t.set_color(1.0, 0.2, 0.2);
    let eye = Point3::new(2.0f32, 1.0, 2.0);
    let at = Point3::origin();
    let mut arc_ball = ArcBall::new(eye, at);
    arc_ball.set_pitch(1.57);
    let solver = JacobianIKSolver::new(0.001, 0.001, 100);
    //    let mut i = 0;
    while window.render_with_camera(&mut arc_ball) {
        //    while window.render() {
        //        let curr_yaw = arc_ball.yaw();
        //        arc_ball.set_yaw(curr_yaw + 0.05);
        for mut event in window.events().iter() {
            match event.value {
                WindowEvent::Key(code, _, Action::Release, _) => {
                    match code {
                        Key::Z => {
                            arm.set_joint_angles(&angles).unwrap();
                        }
                        Key::F => {
                            target.translation.vector[0] += 0.1;
                        }
                        Key::B => {
                            target.translation.vector[0] -= 0.1;
                        }
                        Key::R => {
                            target.translation.vector[1] += 0.1;
                        }
                        Key::L => {
                            target.translation.vector[1] -= 0.1;
                        }
                        Key::P => {
                            target.translation.vector[2] -= 0.1;
                        }
                        Key::N => {
                            target.translation.vector[2] += 0.1;
                        }
                        _ => {}
                    }
                    event.inhibited = true // override the default keyboard handler
                }
                _ => {}

            }
        }
        solver
            .solve(&mut arm, &target)
            .unwrap_or_else(|err| {
                                println!("Err: {}", err);
                                0.0f32
                            });
        c_t.set_local_transformation(target.clone());
        cubes[0].set_local_transformation(arm.transform);
        for (i, trans) in arm.calc_link_transforms().iter().enumerate() {
            cubes[i + 1].set_local_transformation(trans.clone());
        }
    }

}
