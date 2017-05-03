extern crate alga;
extern crate env_logger;
extern crate glfw;
extern crate kiss3d;
extern crate nalgebra as na;
extern crate nkinematics as nk;
extern crate nkinematics_urdf as nk_urdf;
extern crate urdf_rs;
extern crate urdf_vis;

use glfw::{Action, WindowEvent, Key};
use kiss3d::camera::ArcBall;
use kiss3d::light::Light;
use kiss3d::window::Window;
use std::collections::HashMap;
use std::path::Path;

fn main() {
    env_logger::init().unwrap();

    let args: Vec<String> = std::env::args().collect();
    if args.len() < 2 {
        println!("Usage: {} URDF_FILE or XACRO_FILE", args[0]);
        std::process::exit(1);
    }
    let mut window = Window::new("urdf_viewer");
    window.set_light(Light::StickToCamera);
    let eye = na::Point3::new(0.5f32, 1.0, -2.0);
    let at = na::Point3::new(0.0f32, 0.5, 0.0);
    let mut arc_ball = ArcBall::new(eye, at);

    let input_path = Path::new(&args[1]);
    let urdf_file_path;
    let urdf_path;
    if input_path.extension().unwrap() == "xacro" {
        urdf_file_path = format!("/tmp/urdf_vis/{}",
                                 &input_path.with_extension("urdf").to_str().unwrap());
        urdf_path = Path::new(&urdf_file_path);
        urdf_vis::convert_xacro_to_urdf(&input_path, &urdf_path).unwrap();
    } else {
        urdf_path = Path::new(&args[1]);
    }
    let urdf_robo = urdf_rs::read_file(&urdf_path).unwrap();
    let robot = nk_urdf::create_tree::<f32>(&urdf_robo);
    let base_transform = na::Isometry3::from_parts(na::Translation3::new(0.0, 0.0, 0.0),
                                                   na::UnitQuaternion::from_euler_angles(0.0, 1.57, 1.57));
    robot.root_link.borrow_mut().data.transform = base_transform;

    let mut scenes = HashMap::new();
    for l in urdf_robo.links {
        scenes.insert(l.name, urdf_vis::add_geometry(&l.visual, &mut window).unwrap());
    }

    let dof = robot
        .map(&|ljn_ref| ljn_ref.borrow().data.get_joint_angle())
        .len();

    let mut angles_vec = vec![0.0f32; dof];
    let mut j = 0;
    while window.render_with_camera(&mut arc_ball) {
        for mut event in window.events().iter() {
            match event.value {
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
                        Key::Up => angles_vec[j] += 0.3,
                        Key::Down => angles_vec[j] -= 0.3,
                        _ => {},
                    }
                    event.inhibited = true // override the default keyboard handler
                }
                _ => {}
            }
        }
        for (lj, angle) in robot.map(&|ljn_ref| ljn_ref.clone())
            .iter()
            .zip(angles_vec.iter()) {
                let _ = lj.borrow_mut().data.set_joint_angle(*angle);
            }

        for (trans, link_name) in robot.calc_link_transforms().iter()
            .zip(robot.map(&|ljn_ref| ljn_ref.borrow().data.name.clone())) {
                match scenes.get_mut(&link_name) {
                    Some(obj) => obj.set_local_transformation(trans.clone()),
                    None => {
                        println!("{} not found", link_name);
                    }
                }
            }
    }
}
