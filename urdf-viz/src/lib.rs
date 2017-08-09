//! # urdf visualization
//!
//! # Limitation
//!
//! ## Mesh
//!
//! Only `.obj` is supported by [kiss3d](https://github.com/sebcrozet/kiss3d),
//! the visualization library used this crate.
//! Other files are converted by `meshlabserver`.
//! If you add `-a` option, `assimp` is used instead of meshlab.
//! You need to install meshlab or assimp anyway.
//!
extern crate alga;
extern crate glfw;
extern crate kiss3d;
extern crate nalgebra as na;
extern crate k;
extern crate regex;
extern crate urdf_rs;
#[macro_use]
extern crate log;

use kiss3d::scene::SceneNode;
use kiss3d::window::Window;
use regex::Regex;
use std::collections::HashMap;
use std::path::Path;
use std::process::Command;
use std::rc::Rc;

pub enum MeshConvert {
    Assimp,
    Meshlab,
}

fn create_parent_dir(new_path: &Path) -> Result<(), std::io::Error> {
    let new_parent_dir = new_path.parent().unwrap();
    if !new_parent_dir.is_dir() {
        info!("creating dir {}", new_parent_dir.to_str().unwrap());
        std::fs::create_dir_all(new_parent_dir)?;
    }
    Ok(())
}

pub fn convert_xacro_to_urdf<P>(filename: P, new_path: P) -> Result<(), std::io::Error>
    where P: AsRef<Path>
{
    create_parent_dir(new_path.as_ref())?;
    let output = Command::new("xacro")
        .args(&["--inorder",
                filename.as_ref().to_str().unwrap(),
                "-o",
                new_path.as_ref().to_str().unwrap()])
        .output()
        .expect("failed to execute xacro. install by apt-get install ros-*-xacro");
    if output.status.success() {
        Ok(())
    } else {
        error!("{}", String::from_utf8(output.stderr).unwrap());
        Err(std::io::Error::new(std::io::ErrorKind::Other, "faild to xacro"))
    }
}

pub fn convert_to_obj_file_by_meshlab(filename: &Path,
                                      new_path: &Path)
                                      -> Result<(), std::io::Error> {
    create_parent_dir(new_path)?;
    info!("converting {:?} to {:?}", filename, new_path);
    let output = Command::new("meshlabserver")
        .args(&["-i", filename.to_str().unwrap(), "-o", new_path.to_str().unwrap()])
        .output()
        .expect("failed to execute meshlabserver. install by apt-get install meshlab");
    if output.status.success() {
        Ok(())
    } else {
        Err(std::io::Error::new(std::io::ErrorKind::Other, "faild to meshlab"))
    }
}

pub fn convert_to_obj_file_by_assimp(filename: &Path,
                                     new_path: &Path)
                                     -> Result<(), std::io::Error> {
    create_parent_dir(new_path)?;
    info!("converting {:?} to {:?}", filename, new_path);
    let output =
        Command::new("assimp")
            .args(&["export", filename.to_str().unwrap(), new_path.to_str().unwrap(), "-ptv"])
            .output()
            .expect("failed to execute meshlabserver. install by apt-get install assimp-utils");
    if output.status.success() {
        Ok(())
    } else {
        Err(std::io::Error::new(std::io::ErrorKind::Other, "faild to assimp"))
    }
}


fn rospack_find(package: &str) -> Option<String> {
    let output = Command::new("rospack")
        .arg("find")
        .arg(package)
        .output()
        .expect("rospack find failed");
    if output.status.success() {
        String::from_utf8(output.stdout)
            .map(|string| string.trim().to_string())
            .ok()
    } else {
        None
    }
}

pub fn add_geometry(visual: &urdf_rs::Visual,
                    mesh_convert: &MeshConvert,
                    window: &mut Window)
                    -> Option<SceneNode> {
    let mut geom = match visual.geometry {
        urdf_rs::Geometry::Box { ref size } => {
            Some(window.add_cube(size[0] as f32, size[1] as f32, size[2] as f32))
        }
        urdf_rs::Geometry::Cylinder { radius, length } => {
            Some(window.add_cylinder(radius as f32, length as f32))
        }
        urdf_rs::Geometry::Sphere { radius } => Some(window.add_sphere(radius as f32)),
        urdf_rs::Geometry::Mesh {
            ref filename,
            scale,
        } => {
            let re = Regex::new("^package://(\\w+)/").unwrap();
            let replaced_filename =
                re.replace(filename,
                           |ma: &regex::Captures| match rospack_find(&ma[1]) {
                               Some(found_path) => found_path + "/",
                               None => panic!("failed to find ros package {}", &ma[1]),
                           });
            let path = Path::new(&replaced_filename);
            assert!(path.exists(), "{} not found", replaced_filename);
            let mut cache_path = "".to_string();
            let new_path;
            if path.extension().unwrap() == "obj" {
                new_path = path;
            } else {
                let new_rel_path = Path::new(&replaced_filename);
                cache_path = format!("/tmp/urdf_vis{}",
                                     &new_rel_path.with_extension("obj").to_str().unwrap());
                new_path = Path::new(&cache_path);
                info!("cache obj path = {:?}", new_path);
            }
            let mtl_path_string = cache_path.clone() + ".mtl";
            let mtl_path = Path::new(&mtl_path_string);
            if !new_path.exists() {
                match *mesh_convert {
                        MeshConvert::Assimp => convert_to_obj_file_by_assimp(path, new_path),
                        MeshConvert::Meshlab => convert_to_obj_file_by_meshlab(path, new_path),
                    }
                    .unwrap();
            }
            Some(window.add_obj(new_path,
                                mtl_path,
                                na::Vector3::new(scale[0] as f32,
                                                 scale[1] as f32,
                                                 scale[2] as f32)))
        }
    };
    let rgba = &visual.material.color.rgba;
    match geom {
        Some(ref mut obj) => obj.set_color(rgba[0] as f32, rgba[1] as f32, rgba[2] as f32),
        None => return None,
    }
    geom
}


pub struct Viewer {
    pub window: kiss3d::window::Window,
    pub urdf_robot: urdf_rs::Robot,
    pub scenes: HashMap<String, SceneNode>,
    pub arc_ball: kiss3d::camera::ArcBall,
    font_map: HashMap<i32, Rc<kiss3d::text::Font>>,
    font_data: &'static [u8],
}

impl Viewer {
    pub fn new(urdf_robot: urdf_rs::Robot) -> Viewer {
        let eye = na::Point3::new(0.5f32, 1.0, -3.0);
        let at = na::Point3::new(0.0f32, 0.25, 0.0);
        Viewer {
            window: kiss3d::window::Window::new("urdf_viewer"),
            urdf_robot: urdf_robot,
            scenes: HashMap::new(),
            arc_ball: kiss3d::camera::ArcBall::new(eye, at),
            font_map: HashMap::new(),
            font_data: include_bytes!("font/Inconsolata.otf"),
        }
    }
    pub fn setup(&mut self, mesh_convert: MeshConvert) {
        self.window
            .set_light(kiss3d::light::Light::StickToCamera);
        for l in &self.urdf_robot.links {
            self.scenes
                .insert(l.name.to_string(),
                        add_geometry(&l.visual, &mesh_convert, &mut self.window).unwrap());
        }
    }
    pub fn render(&mut self) -> bool {
        self.window.render_with_camera(&mut self.arc_ball)
    }
    pub fn update(&mut self, robot: &mut k::LinkTree<f32>) {
        for (trans, link_name) in
            robot
                .calc_link_transforms()
                .iter()
                .zip(robot.map(&|ljn_ref| ljn_ref.borrow().data.name.clone())) {
            match self.scenes.get_mut(&link_name) {
                Some(obj) => obj.set_local_transformation(*trans),
                None => {
                    println!("{} not found", link_name);
                }
            }
        }
    }
    pub fn draw_text(&mut self,
                     text: &str,
                     size: i32,
                     pos: &na::Point2<f32>,
                     color: &na::Point3<f32>) {
        self.window
            .draw_text(text,
                       pos,
                       self.font_map
                           .entry(size)
                           .or_insert(kiss3d::text::Font::from_memory(self.font_data, size)),
                       color);
    }
    pub fn events(&self) -> kiss3d::window::EventManager {
        self.window.events()
    }
}

#[test]
fn it_works() {}
