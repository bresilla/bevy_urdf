use bevy::prelude::*;

use crate::{kinematics, mesh, robot};

/// Library-only URDF plugin. Wires up the three systems a consumer
/// always wants: spawning the robot from a `LoadRobot` event, loading
/// the URDF's referenced meshes into Bevy assets, and running
/// forward kinematics from `k::Chain` into per-link `Transform`s.
/// Physics, camera, UI, and overlays are explicitly NOT here — bring
/// your own.
#[derive(Default)]
pub struct UrdfPlugin;

impl Plugin for UrdfPlugin {
    fn build(&self, app: &mut App) {
        app.add_plugins((
            robot::RobotPlugin,
            mesh::MeshPlugin,
            kinematics::KinematicsPlugin,
        ));
    }
}
