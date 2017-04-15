extern crate alga;
extern crate glfw;
extern crate kiss3d;
extern crate nalgebra as na;
extern crate nkinematics as nk;
extern crate nkinematics_urdf as nk_urdf;
extern crate urdf_rs;
extern crate urdf_vis;

use kiss3d::light::Light;
use kiss3d::window::Window;
use nk::KinematicChain;
use std::collections::HashMap;

fn main() {
    let args: Vec<String> = std::env::args().collect();
    if args.len() < 2 {
        println!("Usage: {} URDF_FILE", args[0]);
        std::process::exit(1);
    }
    let mut window = Window::new("urdf_viewer");
    window.set_light(Light::StickToCamera);
    let urdf_robo;
    match urdf_rs::read_file(&args[1]) {
        Ok(urdf) => urdf_robo = urdf,
        Err(_) => {
            println!("failed to load {}", args[1]);
            std::process::exit(2);
        }
    }
    let mut robot = nk_urdf::create_robot::<f32>(&urdf_robo);
    let base_transform = na::Isometry3::from_parts(na::Translation3::new(0.0, 0.0, 0.0),
                                                   na::UnitQuaternion::from_euler_angles(0.0, -1.57, -1.57));
    robot.set_transform(base_transform);
    let mut scenes = HashMap::new();
    for l in urdf_robo.links {
        scenes.insert(l.name, urdf_vis::add_geometry(&l.visual, &mut window));
    }

    // TODO: set root link transform without defined name "root"
    match scenes.get_mut("root") {
        Some(obj) => obj.set_local_transformation(robot.get_transform()),
        None => { println!("root not found") },
    }

    let mut angles_vec = Vec::new();
    for i in 0..robot.frames.len() {
        let dof = robot.frames[i].linked_joints.len();
        angles_vec.push(vec![0.0f32; dof]);
    }

    let mut j = 0;
    let mut t = 0.0;

    while window.render() {
        t += 0.1;
        if t > std::f32::consts::PI * 2.0 {
            j += 1;
            t = 0.0;
        }
        for i in 0..robot.frames.len() {
            let dof = angles_vec[i].len();
            angles_vec[i][j % dof] = t.sin();
            let mut manip: &mut nk::LinkedFrame<f32> = &mut robot.frames[i];
            manip.set_joint_angles(&angles_vec[i]).unwrap();
            for (trans, link_name) in manip.calc_link_transforms().iter()
                .zip(manip.linked_joints.iter().map(|lj| lj.name.clone())) {
                    match scenes.get_mut(&link_name) {
                        Some(obj) => obj.set_local_transformation(trans.clone()),
                        None => {
                            println!("{} not found", link_name);
                        }
                    }
                }
        }
    }
}
