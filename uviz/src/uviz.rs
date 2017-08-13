extern crate alga;
extern crate clap;
extern crate env_logger;
extern crate nalgebra as na;
extern crate k;
extern crate urdf_rs;
extern crate urdf_viz;

#[macro_use]
extern crate rosrust;

use k::InverseKinematicsSolver;
use k::KinematicChain;
use std::path::Path;
use std::sync::{Arc, Mutex};
use rosrust::Ros;

rosmsg_include!();

fn main() {
    env_logger::init().unwrap();
    let mut ros = Ros::new("uviz").unwrap();
    let opt = urdf_viz::Opt::from_args();
    let mesh_convert = opt.get_mesh_convert_method();
    let ik_dof = opt.ik_dof;
    let input_path = Path::new(&opt.input_urdf_or_xacro);
    let urdf_path = urdf_viz::convert_xacro_if_needed_and_get_path(input_path).unwrap();
    let urdf_robo = urdf_rs::read_file(&urdf_path).unwrap();
    let mut robot = k::urdf::create_tree::<f32>(&urdf_robo);
    let mut viewer = urdf_viz::Viewer::new(urdf_robo);
    viewer.setup(mesh_convert);
    let base_transform =
        na::Isometry3::from_parts(na::Translation3::new(0.0, 0.0, 0.0),
                                  na::UnitQuaternion::from_euler_angles(0.0, 1.57, 1.57));
    robot.root_link.borrow_mut().data.transform = base_transform;
    let mut arms = k::create_kinematic_chains_with_dof_limit(&robot, ik_dof);
    let num_arms = arms.len();
    println!("num_arms = {}", num_arms);

    let solver = k::JacobianIKSolverBuilder::<f32>::new().finalize();
    let mut end_vel_map = Vec::new();
    for arm in &arms {
        let end_vel = Arc::new(Mutex::new(vec![0.0f32; 3]));
        end_vel_map.push(end_vel.clone());
        ros.subscribe(format!("arm_{}_cmd_vel", arm.name).as_str(),
                       move |v: msg::geometry_msgs::Twist| {
                           let mut vel = end_vel.lock().unwrap();
                           vel[0] = v.linear.x as f32;
                           vel[1] = v.linear.y as f32;
                           vel[2] = v.linear.z as f32;
                       })
            .unwrap();
    }

    let dof = robot.dof();

    let angles_vec = Arc::new(Mutex::new(vec![0.0f32; dof]));
    let angles_vec_copy = angles_vec.clone();
    ros.subscribe("joint_state",
                   move |js: msg::sensor_msgs::JointState| for (i, p) in js.position
                           .iter()
                           .enumerate() {
                       angles_vec.lock().unwrap()[i] = *p as f32;
                   })
        .unwrap();
    while viewer.render() {
        robot
            .set_joint_angles(&angles_vec_copy.lock().unwrap())
            .unwrap();
        for (vel_mutex, arm) in end_vel_map.iter().zip(arms.iter_mut()) {
            let vel = vel_mutex.lock().unwrap();
            if vel[0].abs() > 0.01 || vel[1].abs() > 0.01 || vel[2].abs() > 0.01 {
                let mut target = arm.calc_end_transform();
                target.translation.vector[0] += vel[0];
                target.translation.vector[1] += vel[1];
                target.translation.vector[2] += vel[2];
                solver
                    .solve(arm, &target)
                    .unwrap_or_else(|err| {
                                        println!("Err: {}", err);
                                        0.0f32
                                    });
            }
        }
        viewer.update(&mut robot);
    }
}
