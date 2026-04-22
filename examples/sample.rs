//! Smoke test for `bevy_urdf`: load the urdf-viz sample URDF, render it
//! with an arcball camera inside a BigSpace grid.
//!
//! Run with: `cargo run --example sample`

use std::path::PathBuf;

use bevy::prelude::*;
use big_space::prelude::*;
use bevy_urdf::{ArcballCamera, LoadRobot, UrdfPlugin};

fn main() {
    App::new()
        // big_space owns transform propagation, so Bevy's default
        // TransformPlugin must be disabled — otherwise the two fight.
        .add_plugins(DefaultPlugins.build().disable::<TransformPlugin>())
        .add_plugins(BigSpaceDefaultPlugins)
        .add_plugins(UrdfPlugin)
        .add_systems(Startup, setup)
        .run();
}

fn setup(mut commands: Commands) {
    // Key light + ambient so PBR materials aren't black.
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

    // Everything must live inside ONE BigSpace. The camera carries
    // `FloatingOrigin` (big_space uses it as the reference point for
    // floating-origin translation — exactly one per BigSpace).
    let urdf_path = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("xtra/urdf-viz/sample.urdf");
    commands.spawn_big_space_default(|grid| {
        grid.spawn_spatial((
            Camera3d::default(),
            Transform::from_xyz(3.0, 2.0, 3.0).looking_at(Vec3::new(0.0, 0.4, 0.0), Vec3::Y),
            ArcballCamera {
                focus: Vec3::new(0.0, 0.4, 0.0),
                distance: 4.0,
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
