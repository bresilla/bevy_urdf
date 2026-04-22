//! Franka Emika Panda — real 7-DOF arm with Collada visual meshes and
//! STL collision meshes. Exercises the package:// resolver + mesh loader
//! properly, unlike the toy sample.
//!
//! Run: `cargo run --example panda`

use std::path::PathBuf;

use bevy::prelude::*;
use big_space::prelude::*;
use bevy_urdf::{ArcballCamera, LoadRobot, PackageMap, UrdfPlugin};

fn main() {
    App::new()
        .add_plugins(DefaultPlugins.build().disable::<TransformPlugin>())
        .add_plugins(BigSpaceDefaultPlugins)
        .add_plugins(UrdfPlugin)
        .add_systems(Startup, setup)
        .run();
}

fn setup(mut commands: Commands, mut package_map: ResMut<PackageMap>) {
    let franka_root = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("urdf/oems/xacro_generated/franka_emika/franka_description");

    // URDF references `package://franka_description/...` — tell the
    // resolver that this package name maps to the folder we just cloned.
    package_map.insert("franka_description", franka_root.clone());

    let urdf_path = franka_root.join("robots/panda/panda.urdf");

    commands.spawn((
        DirectionalLight {
            illuminance: 10_000.0,
            shadows_enabled: true,
            ..default()
        },
        Transform::from_xyz(4.0, 8.0, 4.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));
    commands.insert_resource(bevy::light::GlobalAmbientLight {
        brightness: 400.0,
        ..default()
    });

    commands.spawn_big_space_default(|grid| {
        grid.spawn_spatial((
            Camera3d::default(),
            Transform::from_xyz(1.8, 1.2, 1.8).looking_at(Vec3::new(0.0, 0.4, 0.0), Vec3::Y),
            ArcballCamera {
                focus: Vec3::new(0.0, 0.4, 0.0),
                distance: 2.5,
                ..default()
            },
            FloatingOrigin,
        ));
        let robot_root = grid.spawn_spatial(()).id();
        grid.commands().write_message(LoadRobot {
            path: urdf_path,
            root: Some(robot_root),
        });
    });
}
