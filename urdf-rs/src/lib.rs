#[macro_use]
extern crate serde_derive;
extern crate serde_xml_rs;

#[derive(Debug, Deserialize)]
struct Mass {
    pub value: f64,
}

#[derive(Debug, Deserialize)]
struct Inertia {
    pub ixx: f64,
    pub ixy: f64,
    pub ixz: f64,
    pub iyy: f64,
    pub iyz: f64,
    pub izz: f64,
}

fn default_pose() -> Pose {
    Pose {
        xyz: empty_vec3(),
        rpy: empty_vec3(),
    }
}

fn default_mass() -> Mass {
    Mass { value: 0.0 }
}

fn default_inertia() -> Inertia {
    Inertia {
        ixx: 0.0,
        ixy: 0.0,
        ixz: 0.0,
        iyy: 0.0,
        iyz: 0.0,
        izz: 0.0,
    }
}

fn default_inertial() -> Inertial {
    Inertial {
        origin: default_pose(),
        mass: default_mass(),
        inertia: default_inertia(),
    }
}


#[derive(Debug, Deserialize)]
struct Inertial {
    #[serde(default = "default_pose")]
    pub origin: Pose,
    pub mass: Mass,
    pub inertia: Inertia,
}

#[derive(Debug, Deserialize)]
pub enum Geometry {
    #[serde(rename = "box")]
    Box { size: String },
    #[serde(rename = "cylinder")]
    Cylinder { radius: f64, length: f64 },
    #[serde(rename = "sphere")]
    Sphere { radius: f64 },
    #[serde(rename = "mesh")]
    Mesh { filename: String, scale: f64 },
}


#[derive(Debug, Deserialize)]
struct Color {
    #[serde(default)]
    rgba: String,
}

#[derive(Debug, Deserialize)]
struct Texture {
    #[serde(default)]
    filename: String,
}

#[derive(Debug, Deserialize)]
struct Material {
    #[serde(default)]
    pub name: String,
    #[serde(default = "default_color")]
    pub color: Color,
    #[serde(default = "default_texture")]
    pub texture: Texture,
}

fn default_geometry() -> Geometry {
    Geometry::Box { size: "0 0 0".to_string() }
}

fn default_visual() -> Visual {
    Visual {
        name: "".to_string(),
        origin: default_pose(),
        geometry: default_geometry(),
        material: default_material(),
    }
}

fn default_texture() -> Texture {
    Texture { filename: "".to_string() }
}

fn default_color() -> Color {
    Color { rgba: "0 0 0 0".to_string() }
}

fn default_material() -> Material {
    Material {
        name: "".to_string(),
        color: default_color(),
        texture: default_texture(),
    }
}


#[derive(Debug, Deserialize)]
struct Visual {
    #[serde(default)]
    pub name: String,
    #[serde(default = "default_pose")]
    pub origin: Pose,
    pub geometry: Geometry,
    #[serde(default = "default_material")]
    pub material: Material,
}

fn default_collision() -> Collision {
    Collision {
        name: "".to_string(),
        origin: default_pose(),
        geometry: default_geometry(),
    }
}

#[derive(Debug, Deserialize)]
struct Collision {
    #[serde(default)]
    pub name: String,
    pub origin: Pose,
    pub geometry: Geometry,
}

#[derive(Debug, Deserialize)]
struct Link {
    pub name: String,
    #[serde(default = "default_inertial")]
    pub inertial: Inertial,
    #[serde(default = "default_visual")]
    pub visual: Visual,
    #[serde(default = "default_collision")]
    pub collision: Collision,
}

#[derive(Debug, Deserialize)]
pub struct Axis {
    pub xyz: String,
}

fn default_axis() -> Axis {
    Axis { xyz: "1 0 0".to_string() }
}

fn empty_vec3() -> String {
    "0 0 0".to_string()
}

#[derive(Debug, Deserialize)]
struct Pose {
    #[serde(default = "empty_vec3")]
    pub xyz: String,
    #[serde(default = "empty_vec3")]
    pub rpy: String,
}

#[derive(Debug, Deserialize)]
struct LinkName {
    pub link: String,
}

#[derive(Debug, Deserialize)]
struct Joint {
    pub name: String,
    #[serde(rename = "type")]
    pub joint_type: String,
    #[serde(default = "default_pose")]
    pub origin: Pose,
    pub parent: LinkName,
    pub child: LinkName,
    #[serde(default = "default_axis")]
    pub axis: Axis,
}

#[derive(Debug, Deserialize)]
struct Robot {
    pub name: String,

    #[serde(rename = "link", default)]
    pub links: Vec<Link>,

    #[serde(rename = "joint", default)]
    pub joints: Vec<Joint>,
}


#[test]
fn it_works() {
    use serde_xml_rs::deserialize;
    let s = r##"
        <robot name="robo">
            <link name="shoulder1">
   <inertial>
     <origin xyz="0 0 0.5" rpy="0 0 0"/>
     <mass value="1"/>
     <inertia ixx="100"  ixy="0"  ixz="0" iyy="100" iyz="0" izz="100" />
   </inertial>

   <visual>
     <origin xyz="0 0 0" rpy="0 0 0" />
     <geometry>
       <box size="1 1 1" />
     </geometry>
     <material name="Cyan">
       <color rgba="0 1.0 1.0 1.0"/>
     </material>
   </visual>

   <collision>
     <origin xyz="0 0 0" rpy="0 0 0"/>
     <geometry>
       <cylinder radius="1" length="0.5"/>
     </geometry>
   </collision>
            </link>
            <link name="elbow1" />
            <link name="wrist1" />
            <joint name="shoulder_pitch" type="revolute">
                <origin xyz="0.0 0.0 0.1" />
                <parent link="shoulder1" />
                <child link="elbow1" />
                <axis xyz="0 1 0" />
            </joint>
            <joint name="shoulder_pitch" type="revolute">
                <origin xyz="0.0, 0.0, 0.0" />
                <parent link="elbow1" />
                <child link="wrist1" />
                <axis xyz="0 1 0" />
            </joint>
        </robot>
    "##;
    let robo: Robot = deserialize(s.as_bytes()).unwrap();
    assert_eq!(robo.name, "robo");
    assert_eq!(robo.links.len(), 3);
    assert_eq!(robo.joints.len(), 2);
    assert_eq!(robo.joints[0].name, "shoulder_pitch");
    assert_eq!(robo.joints[0].axis.xyz, "0 1 0");
    assert_eq!(robo.joints[0].origin.xyz, "0.0 0.0 0.1");
}
