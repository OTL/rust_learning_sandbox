extern crate kiss3d;
extern crate nalgebra as na;

use na::{Vector3, UnitQuaternion};
use na::geometry::{Isometry3};
use kiss3d::window::Window;
use kiss3d::light::Light;

fn main() {
    let mut window = Window::new("Kiss3d: cube");
    let mut c1      = window.add_cube(0.1, 0.1, 0.1);
    c1.set_color(1.0, 0.0, 0.0);

    let mut c2 = c1.add_cube(1.0, 1.0, 1.0);
    c2.set_color(0.0, 1.0, 0.0);

    let mut c3 = c2.add_cube(1.0, 1.0, 1.0);
    c3.set_color(0.0, 0.0, 1.0);

    let move1 = Isometry3::new(
        Vector3::new(0.0, 0.0, 0.0),
        Vector3::new(0.0, 0.0, 0.0));
    c1.set_local_transformation(move1);

    let move2 = Isometry3::new(
        Vector3::new(0.2, 0.0, 0.0),
        Vector3::new(0.0, 0.0, 0.0));
    c2.set_local_transformation(move2);

    let move3 = Isometry3::new(
        Vector3::new(0.2, 0.0, 0.0),
        Vector3::new(0.0, 0.0, 0.0));
    c3.set_local_transformation(move3);

    window.set_light(Light::StickToCamera);

    let rot1 = UnitQuaternion::from_axis_angle(
        &Vector3::y_axis(), 0.014);

    let rot2 = UnitQuaternion::from_axis_angle(
        &Vector3::y_axis(), 0.03);

    while window.render() {
        c1.prepend_to_local_rotation(&rot1);
        c2.prepend_to_local_rotation(&rot2);
    }
}
