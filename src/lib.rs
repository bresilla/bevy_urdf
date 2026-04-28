//! Pure URDF → Bevy library. Parses URDF (`urdf-rs`), loads referenced
//! meshes (`mesh-loader`), spawns one Bevy entity per link with the
//! authored materials, and runs forward kinematics from a `k::Chain`
//! into per-link `Transform`s. No physics, no UI, no camera — those
//! are consumer concerns.

pub mod kinematics;
pub mod mesh;
pub mod plugin;
pub mod robot;
pub mod urdf;

pub use plugin::UrdfPlugin;
pub use robot::{
    GeomBaseColor, GeomKind, JointKind, LoadRobot, Robot, RobotJoint, RobotLink, RobotRoot,
};
pub use urdf::{PackageMap, UrdfError};
