use bevy::prelude::*;

use crate::{camera, keyboard, kinematics, manipulation, mesh, overlays, robot, ui};

/// Core URDF visualization plugin. Does NOT include physics — add
/// `UrdfPhysicsPlugin` separately if you want rapier colliders (it's
/// expensive in debug builds).
#[derive(Default)]
pub struct UrdfPlugin;

impl Plugin for UrdfPlugin {
    fn build(&self, app: &mut App) {
        app.add_plugins((
            robot::RobotPlugin,
            mesh::MeshPlugin,
            kinematics::KinematicsPlugin,
            camera::ArcballCameraPlugin,
            manipulation::ManipulationPlugin,
            ui::UrdfUiPlugin,
            keyboard::UrdfKeyboardPlugin,
            overlays::UrdfOverlaysPlugin,
        ));
    }
}
