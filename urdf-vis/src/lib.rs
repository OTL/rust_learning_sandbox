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
use std::process::Command;

fn convert_to_obj_file(filename: &Path, new_path: &Path) {
    let output = Command::new("assimp")
        .args(&["export", filename.to_str().unwrap(), new_path.to_str().unwrap()])
        .output()
        .expect("failed to execute assimp. install assimp command by apt-get install assimp-utils");
    println!("converting {:?} to {:?}", filename, new_path);
    println!("{:?}", output);
}

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
            let filename = filename.trim_left_matches("package://");
            let path = Path::new(&filename);
            let mtl_path = Path::new("");
            assert!(path.exists(), "{} not found", filename);
            let new_path = path.with_extension("obj");
            if !new_path.exists() {
                convert_to_obj_file(&path, &new_path);
            }
            window.add_obj(&new_path, &mtl_path,
                           na::Vector3::new(scale[0] as f32, scale[1] as f32, scale[2] as f32))
        }
    };
    let rgba = &visual.material.color.rgba;
    geom.set_color(rgba[0] as f32, rgba[1] as f32, rgba[2] as f32);
    geom
}

#[test]
fn it_works() {
}
