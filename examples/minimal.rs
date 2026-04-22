//! Bare-bones: Bevy + big_space + a camera + a cube. No UrdfPlugin.
//! If this doesn't respond either, the fault is in the Bevy/big_space
//! setup, not my plugin stack.

use bevy::prelude::*;
use big_space::prelude::*;

fn main() {
    App::new()
        .add_plugins(DefaultPlugins.build().disable::<TransformPlugin>())
        .add_plugins(BigSpaceDefaultPlugins)
        .add_systems(Startup, setup)
        .run();
}

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    commands.spawn((
        DirectionalLight::default(),
        Transform::from_xyz(4.0, 8.0, 4.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));
    // Everything — camera included — lives inside the BigSpace. The
    // camera carries `FloatingOrigin` so big_space knows which entity is
    // the reference point for its floating-origin translation.
    commands.spawn_big_space_default(|grid| {
        grid.spawn_spatial((
            Camera3d::default(),
            Transform::from_xyz(3.0, 2.0, 3.0).looking_at(Vec3::ZERO, Vec3::Y),
            FloatingOrigin,
        ));
        grid.spawn_spatial((
            Mesh3d(meshes.add(Cuboid::new(1.0, 1.0, 1.0))),
            MeshMaterial3d(materials.add(Color::srgb(0.8, 0.3, 0.3))),
        ));
    });
}
