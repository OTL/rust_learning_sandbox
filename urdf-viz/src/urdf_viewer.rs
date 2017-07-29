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
    if index == 0 {
        for ang in angles_vec.iter_mut() {
            *ang += rot;
        }
    } else {
        let dof = angles_vec.len();
        angles_vec[index % dof] += rot;
    }
    robot.set_joint_angles(angles_vec);
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

    let dof = robot
        .map(&|ljn_ref| ljn_ref.borrow().data.get_joint_angle())
        .len();

    let mut angles_vec = vec![0.0f32; dof];
    let mut j = 0;
    let mut is_ctrl = false;
    let mut is_shift = false;
    let mut last_cur_pos_y = 0f64;
    let mut last_cur_pos_x = 0f64;
    while viewer.render() {
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
                        move_ang(j,
                                 ((y - last_cur_pos_y) / 100.0) as f32,
                                 &mut angles_vec,
                                 &mut robot);
                    }
                    if is_shift {
                        event.inhibited = true;
                        let mut target = arms[j % num_arms].calc_end_transform();
                        target.translation.vector[2] -= ((x - last_cur_pos_x) / 100.0) as f32;
                        if is_ctrl {
                            target.translation.vector[1] += ((y - last_cur_pos_y) / 100.0) as f32;
                        } else {
                            target.translation.vector[0] += ((y - last_cur_pos_y) / 100.0) as f32;
                        }
                        solver
                            .solve(&mut arms[j % num_arms], &target)
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
                        Key::Num0 => j = 0,
                        Key::Num1 => j = 1,
                        Key::Num2 => j = 2,
                        Key::Num3 => j = 3,
                        Key::Num4 => j = 4,
                        Key::Num5 => j = 5,
                        Key::Num6 => j = 6,
                        Key::Num7 => j = 7,
                        Key::Num8 => j = 8,
                        Key::Num9 => j = 9,
                        Key::A => j = 10,
                        Key::B => j = 11,
                        Key::C => j = 12,
                        Key::D => j = 13,
                        Key::E => j = 14,
                        Key::F => j = 15,
                        Key::G => j = 16,
                        Key::H => j = 17,
                        Key::I => j = 18,
                        Key::J => j = 19,
                        Key::K => j = 20,
                        Key::L => j = 21,
                        Key::M => j = 22,
                        Key::N => j = 23,
                        Key::O => j = 24,
                        Key::Up => move_ang(j, 0.1, &mut angles_vec, &mut robot),
                        Key::Down => move_ang(j, -0.1, &mut angles_vec, &mut robot),
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
