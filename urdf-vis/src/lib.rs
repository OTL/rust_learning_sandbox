extern crate urdf_rs;
extern crate kiss3d;
extern crate nalgebra as na;
extern crate glfw;
extern crate nkinematics as nk;
extern crate nkinematics_urdf as nk_urdf;
extern crate alga;

use kiss3d::scene::SceneNode;
use kiss3d::window::Window;
use std::path::Path;

pub fn add_geometry(visual: &urdf_rs::Visual, window: &mut Window)
                    -> SceneNode {
    let mut geom = match visual.geometry {
        urdf_rs::Geometry::Box{ref size} => {
            window.add_cube(size[0] as f32, size[1] as f32, size[2] as f32)
        },
        urdf_rs::Geometry::Cylinder{radius, length} => {
            window.add_cylinder(radius as f32, length as f32)
        },
        urdf_rs::Geometry::Sphere{radius} => {
            window.add_sphere(radius as f32)
        },
        urdf_rs::Geometry::Mesh{ref filename, scale} => {
            let path = Path::new(&filename);
            let mtl_path = Path::new("");
            window.add_obj(&path, &mtl_path,
                           na::Vector3::new(scale as f32, scale as f32, scale as f32))
        }
    };
    let rgba = &visual.material.color.rgba;
    geom.set_color(rgba[0] as f32, rgba[1] as f32, rgba[2] as f32);
    geom
}

#[test]
fn it_works() {
    use kiss3d::light::Light;
    use std::collections::HashMap;
    use nk::KinematicChain;
    use alga::general::Real;

    let mut window = Window::new("nkinematics ui");
    window.set_light(Light::StickToCamera);
    let urdf_robo = urdf_rs::read_file("sample.urdf").unwrap();
    let mut manip = nk::LinkedFrame::new("arm");
    manip.linked_joints = nk_urdf::create_serial_linked_joints_vec::<f32>(&urdf_robo).pop().unwrap();
    let dof = manip.linked_joints.len();
    let mut angles = vec![0.0f32; dof];
    manip.set_joint_angles(&angles).unwrap();
    let mut scenes = HashMap::new();
    for l in urdf_robo.links {
        scenes.insert(l.name, add_geometry(&l.visual, &mut window));
    }

    let mut i = 0;
    let mut t = 0.0;
    while window.render() {
        t += 0.1;
        angles[i % dof] = t.sin();
        if t > 6.28 {
            i += 1;
            t = 0.0;
        }
        manip.set_joint_angles(&angles).unwrap();
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
