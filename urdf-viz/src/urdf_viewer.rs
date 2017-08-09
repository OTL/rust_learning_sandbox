extern crate alga;
extern crate clap;
extern crate env_logger;
extern crate glfw;
extern crate nalgebra as na;
extern crate k;
extern crate urdf_rs;
extern crate urdf_viz;

use clap::{Arg, App};
use glfw::{Action, WindowEvent, Key};
use k::InverseKinematicsSolver;
use k::KinematicChain;
use std::path::Path;

#[cfg(target_os = "macos")]
static NATIVE_MOD: glfw::Modifiers = glfw::Super;

#[cfg(not(target_os = "macos"))]
static NATIVE_MOD: glfw::Modifiers = glfw::Control;

fn move_ang(index: usize, rot: f32, angles_vec: &mut Vec<f32>, robot: &mut k::LinkTree<f32>) {
    let dof = angles_vec.len();
    assert!(index < dof);
    angles_vec[index] += rot;
    robot.set_joint_angles(angles_vec);
}

struct LoopIndex {
    index: usize,
    size: usize,
}

impl LoopIndex {
    fn new(size: usize) -> Self {
        Self {
            index: 0,
            size: size,
        }
    }
    fn get(&self) -> usize {
        self.index
    }
    fn inc(&mut self) {
        self.index += 1;
        self.index %= self.size;
    }
    fn dec(&mut self) {
        if self.index == 0 {
            self.index = self.size - 1;
        } else {
            self.index -= 1;
        }
    }
}
fn main() {
    env_logger::init().unwrap();
    let matches = App::new("urdf_viewer")
        .author("Takashi Ogura <t.ogura@gmail.com>")
        .about("Show and move urdf robot model for debuging")
        .arg(Arg::with_name("input")
                 .help("input urdf or xacro file")
                 .required(true))
        .arg(Arg::with_name("assimp")
                 .short("a")
                 .long("assimp")
                 .help("Use assimp instead of meshlab to convert .dae to .obj for visualization"))
        .get_matches();
    let mesh_convert = if matches.is_present("assimp") {
        urdf_viz::MeshConvert::Assimp
    } else {
        urdf_viz::MeshConvert::Meshlab
    };

    let arg_input = matches.value_of("input").unwrap();
    let abs_urdf_path;
    let input_path = Path::new(&arg_input);
    let urdf_path = if input_path.extension().unwrap() == "xacro" {
        abs_urdf_path = format!("/tmp/urdf_viz/{}",
                                input_path.with_extension("urdf").to_str().unwrap());
        let tmp_urdf_path = Path::new(&abs_urdf_path);
        urdf_viz::convert_xacro_to_urdf(&input_path, &tmp_urdf_path).unwrap();
        tmp_urdf_path
    } else {
        input_path
    };

    let urdf_robo = urdf_rs::read_file(&urdf_path).unwrap();
    let mut robot = k::urdf::create_tree::<f32>(&urdf_robo);
    let mut viewer = urdf_viz::Viewer::new(urdf_robo);
    viewer.setup(mesh_convert);
    let base_transform =
        na::Isometry3::from_parts(na::Translation3::new(0.0, 0.0, 0.0),
                                  na::UnitQuaternion::from_euler_angles(0.0, 1.57, 1.57));
    robot.root_link.borrow_mut().data.transform = base_transform;
    let mut arms = k::create_kinematic_chains(&robot);
    let num_arms = arms.len();
    println!("num_arms = {}", num_arms);
    let solver = k::JacobianIKSolverBuilder::new().finalize();

    let dof = robot.dof();
    let mut angles_vec = vec![0.0f32; dof];
    let mut index_of_move_joint = LoopIndex::new(dof);
    let mut index_of_arm = LoopIndex::new(num_arms);
    let mut is_ctrl = false;
    let mut is_shift = false;
    let mut last_cur_pos_y = 0f64;
    let mut last_cur_pos_x = 0f64;
    let joint_names = robot.get_joint_names();
    while viewer.render() {
        viewer.draw_text("[: joint ID +1\n]: joint ID -1\nUp: joint angle +0.1\nDown: joint angle -0.1\nCtrl+Drag: move joint\nShift+Drag: IK",
                         40,
                         &na::Point2::new(1000f32, 10.0),
                         &na::Point3::new(1f32, 1.0, 1.0));
        viewer.draw_text(&format!("moving joint name [{}]", joint_names[index_of_move_joint.get()]),
                         60,
                         &na::Point2::new(10f32, 80.0),
                         &na::Point3::new(0.5f32, 0.5, 1.0));
        viewer.draw_text(&format!("IK target name [{}]", arms[index_of_arm.get()].name),
                         60,
                         &na::Point2::new(10f32, 200.0),
                         &na::Point3::new(0.5f32, 0.8, 0.2));
        if is_ctrl {
            viewer.draw_text("moving joint by drag",
                             60,
                             &na::Point2::new(10f32, 150.0),
                             &na::Point3::new(0.9f32, 0.5, 1.0));
        }
        if is_shift {
            viewer.draw_text("solving ik",
                             60,
                             &na::Point2::new(10f32, 150.0),
                             &na::Point3::new(0.9f32, 0.5, 1.0));
        }
        for mut event in viewer.events().iter() {
            match event.value {
                WindowEvent::MouseButton(_, Action::Press, mods) => {
                    if mods.contains(NATIVE_MOD) {
                        is_ctrl = true;
                        event.inhibited = true;
                    } else if mods.contains(glfw::Shift) {
                        is_shift = true;
                        event.inhibited = true;
                    }
                }
                WindowEvent::CursorPos(x, y) => {
                    if is_ctrl {
                        event.inhibited = true;
                        move_ang(index_of_move_joint.get(),
                                 ((y - last_cur_pos_y) / 100.0) as f32,
                                 &mut angles_vec,
                                 &mut robot);
                    }
                    if is_shift {
                        event.inhibited = true;
                        let mut target = arms[index_of_arm.get()].calc_end_transform();
                        target.translation.vector[2] -= ((x - last_cur_pos_x) / 100.0) as f32;
                        if is_ctrl {
                            target.translation.vector[1] += ((y - last_cur_pos_y) / 100.0) as f32;
                        } else {
                            target.translation.vector[0] += ((y - last_cur_pos_y) / 100.0) as f32;
                        }
                        solver
                            .solve(&mut arms[index_of_arm.get()], &target)
                            .unwrap_or_else(|err| {
                                                println!("Err: {}", err);
                                                0.0f32
                                            });
                    }
                    last_cur_pos_x = x;
                    last_cur_pos_y = y;
                }
                WindowEvent::MouseButton(_, Action::Release, _) => {
                    if is_ctrl {
                        is_ctrl = false;
                        event.inhibited = true;
                    } else if is_shift {
                        is_shift = false;
                        event.inhibited = true;
                    }
                }
                WindowEvent::Key(code, _, Action::Press, _) => {
                    match code {
                        Key::LeftBracket => index_of_move_joint.inc(),
                        Key::RightBracket => index_of_move_joint.dec(),
                        Key::Period => index_of_arm.inc(),
                        Key::Comma => index_of_arm.dec(),
                        Key::Up => move_ang(index_of_move_joint.get(), 0.1, &mut angles_vec, &mut robot),
                        Key::Down => move_ang(index_of_move_joint.get(), -0.1, &mut angles_vec, &mut robot),
                        _ => {}
                    };
                    event.inhibited = true;
                }
                _ => {}
            }
        }
        viewer.update(&mut robot);
    }
}
