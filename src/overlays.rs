//! Debug overlays controlled by `DisplayToggles`:
//! - `v` toggles visual / collision mesh visibility
//! - `f` toggles a gizmo triad per joint frame
//! - `l` toggles a gizmo triad per link frame
//! - `n` toggles world-space link-name labels
//!
//! The triad gizmos are drawn every frame with `bevy::Gizmos`; the name
//! labels are drawn via egui using a world→screen projection.

use bevy::asset::RenderAssetUsages;
use bevy::mesh::{Indices, Mesh, PrimitiveTopology};
use bevy::prelude::*;
use bevy_egui::{egui, EguiContexts, EguiPrimaryContextPass};

use crate::keyboard::DisplayToggles;
use crate::robot::{GeomKind, Robot, RobotLink, RobotRoot};

pub struct UrdfOverlaysPlugin;

impl Plugin for UrdfOverlaysPlugin {
    fn build(&self, app: &mut App) {
        // Gizmos and visibility can run in Update; the egui label overlay
        // has to run in the primary-context schedule (see ui.rs rationale).
        app.add_systems(Startup, spawn_world_grid)
            .add_systems(
                Update,
                (
                    apply_geom_visibility,
                    apply_world_grid_visibility,
                    draw_joint_frames,
                    draw_link_frames,
                    draw_world_axes,
                ),
            )
            .add_systems(EguiPrimaryContextPass, draw_link_name_labels);
    }
}

fn apply_geom_visibility(
    toggles: Res<DisplayToggles>,
    mut q: Query<(&GeomKind, &mut Visibility)>,
) {
    // Always runs — only a few hundred entities even on a dense robot, so
    // the cost is a rounding error and the always-on path rules out any
    // change-detection races on freshly-spawned geometry.
    for (kind, mut vis) in q.iter_mut() {
        let show = match kind {
            GeomKind::Visual => toggles.show_visual,
            GeomKind::Collision => toggles.show_collision,
        };
        let desired = if show {
            Visibility::Inherited
        } else {
            Visibility::Hidden
        };
        if *vis != desired {
            *vis = desired;
        }
    }
}

/// Draws a 3-axis triad (red/green/blue = X/Y/Z) at each joint's world
/// frame. Cheap: a few gizmos per joint.
fn draw_joint_frames(
    toggles: Res<DisplayToggles>,
    robots: Query<&Robot, With<RobotRoot>>,
    robot_tf: Query<&GlobalTransform, With<RobotRoot>>,
    mut gizmos: Gizmos,
) {
    if !toggles.show_joint_frames {
        return;
    }
    for robot in robots.iter() {
        let root_tf = robot_tf.single().copied().unwrap_or_default();
        let base = root_tf.compute_transform();
        for node in robot.chain.iter() {
            let Some(iso) = node.world_transform() else {
                continue;
            };
            let local = isometry_to_transform(&iso);
            let world = base * local;
            draw_triad(&mut gizmos, world, 0.08);
        }
    }
}

fn draw_link_frames(
    toggles: Res<DisplayToggles>,
    links: Query<&GlobalTransform, With<RobotLink>>,
    mut gizmos: Gizmos,
) {
    if !toggles.show_link_frames {
        return;
    }
    for tf in links.iter() {
        let t = tf.compute_transform();
        draw_triad(&mut gizmos, t, 0.06);
    }
}

/// Marker: one per sub-layer of the ground grid (minor lines / major
/// lines / dots). Separate entities so we can give each its own material
/// and toggle / restyle independently.
#[derive(Component, Copy, Clone, PartialEq, Eq)]
enum WorldGridKind {
    Minor,
    Major,
    Dots,
}

/// Grid tuning — lifted in spirit from astrocraft's `grid_overlay.rs`.
/// Not camera-adaptive (URDF viewers work at human scale, so a fixed
/// 20 m area with 0.25 m minor / 1 m major is fine).
const GRID_HALF: f32 = 10.0;
const GRID_MINOR: f32 = 0.25;
const GRID_MAJOR: f32 = 1.0;
const GRID_MINOR_THICK: f32 = 0.005;
const GRID_MAJOR_THICK: f32 = 0.012;
const GRID_DOT_RADIUS: f32 = 0.025;

fn spawn_world_grid(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    let minor_color = Color::linear_rgba(0.45, 0.48, 0.55, 0.18);
    let major_color = Color::linear_rgba(0.70, 0.75, 0.85, 0.55);
    let dot_color = Color::linear_rgba(0.95, 0.95, 1.00, 0.80);

    let mut make_mat = |c: Color| {
        materials.add(StandardMaterial {
            base_color: c,
            emissive: c.to_linear(),
            alpha_mode: AlphaMode::Blend,
            unlit: true,
            cull_mode: None,
            double_sided: true,
            ..default()
        })
    };

    let minor_mesh = meshes.add(build_grid_lines_mesh(
        GRID_HALF,
        GRID_MINOR,
        GRID_MAJOR,
        GRID_MINOR_THICK,
        /* skip_major */ true,
    ));
    let major_mesh = meshes.add(build_grid_lines_mesh(
        GRID_HALF,
        GRID_MAJOR,
        GRID_MAJOR,
        GRID_MAJOR_THICK,
        /* skip_major */ false,
    ));
    let dots_mesh = meshes.add(build_grid_dots_mesh(
        GRID_HALF,
        GRID_MAJOR,
        GRID_DOT_RADIUS,
    ));

    // Slight Y offsets to suppress z-fighting between the three layers.
    commands.spawn((
        WorldGridKind::Minor,
        Mesh3d(minor_mesh),
        MeshMaterial3d(make_mat(minor_color)),
        Transform::from_xyz(0.0, 0.000, 0.0),
        bevy::light::NotShadowCaster,
    ));
    commands.spawn((
        WorldGridKind::Major,
        Mesh3d(major_mesh),
        MeshMaterial3d(make_mat(major_color)),
        Transform::from_xyz(0.0, 0.001, 0.0),
        bevy::light::NotShadowCaster,
    ));
    commands.spawn((
        WorldGridKind::Dots,
        Mesh3d(dots_mesh),
        MeshMaterial3d(make_mat(dot_color)),
        Transform::from_xyz(0.0, 0.002, 0.0),
        bevy::light::NotShadowCaster,
    ));
}

fn apply_world_grid_visibility(
    toggles: Res<DisplayToggles>,
    mut q: Query<(&WorldGridKind, &mut Visibility)>,
) {
    let want = if toggles.show_world_grid {
        Visibility::Inherited
    } else {
        Visibility::Hidden
    };
    for (_kind, mut vis) in q.iter_mut() {
        if *vis != want {
            *vis = want;
        }
    }
}

/// Build a mesh of thin horizontal quads forming a grid of lines.
/// `spacing` is the line step; `major_every` defines the "major line"
/// boundary — when `skip_major` is true, lines on that boundary are
/// omitted (so a minor layer doesn't overlap the major layer).
fn build_grid_lines_mesh(
    half: f32,
    spacing: f32,
    major_every: f32,
    thickness: f32,
    skip_major: bool,
) -> Mesh {
    let mut positions: Vec<[f32; 3]> = Vec::new();
    let mut normals: Vec<[f32; 3]> = Vec::new();
    let mut indices: Vec<u32> = Vec::new();
    let mut next_index: u32 = 0;

    let lines = (half / spacing) as i32;
    // Lines along X (vary Z):
    for lz in -lines..=lines {
        let z = lz as f32 * spacing;
        if skip_major && is_major(z, major_every) {
            continue;
        }
        push_quad(
            &mut positions,
            &mut normals,
            &mut indices,
            &mut next_index,
            Vec3::new(-half, 0.0, z - thickness * 0.5),
            Vec3::new(half, 0.0, z + thickness * 0.5),
        );
    }
    // Lines along Z (vary X):
    for lx in -lines..=lines {
        let x = lx as f32 * spacing;
        if skip_major && is_major(x, major_every) {
            continue;
        }
        push_quad(
            &mut positions,
            &mut normals,
            &mut indices,
            &mut next_index,
            Vec3::new(x - thickness * 0.5, 0.0, -half),
            Vec3::new(x + thickness * 0.5, 0.0, half),
        );
    }
    finalize_mesh(positions, normals, indices)
}

fn build_grid_dots_mesh(half: f32, spacing: f32, radius: f32) -> Mesh {
    let mut positions: Vec<[f32; 3]> = Vec::new();
    let mut normals: Vec<[f32; 3]> = Vec::new();
    let mut indices: Vec<u32> = Vec::new();
    let mut next_index: u32 = 0;
    let lines = (half / spacing) as i32;
    for lx in -lines..=lines {
        for lz in -lines..=lines {
            let x = lx as f32 * spacing;
            let z = lz as f32 * spacing;
            push_disc(
                &mut positions,
                &mut normals,
                &mut indices,
                &mut next_index,
                Vec3::new(x, 0.0, z),
                radius,
                12,
            );
        }
    }
    finalize_mesh(positions, normals, indices)
}

fn is_major(coord: f32, major_every: f32) -> bool {
    let q = (coord / major_every).round();
    (coord - q * major_every).abs() < 1e-3
}

fn push_quad(
    positions: &mut Vec<[f32; 3]>,
    normals: &mut Vec<[f32; 3]>,
    indices: &mut Vec<u32>,
    next_index: &mut u32,
    min: Vec3,
    max: Vec3,
) {
    positions.push([min.x, 0.0, min.z]);
    positions.push([max.x, 0.0, min.z]);
    positions.push([max.x, 0.0, max.z]);
    positions.push([min.x, 0.0, max.z]);
    normals.extend([[0.0, 1.0, 0.0]; 4]);
    indices.extend_from_slice(&[
        *next_index,
        *next_index + 1,
        *next_index + 2,
        *next_index,
        *next_index + 2,
        *next_index + 3,
    ]);
    *next_index += 4;
}

fn push_disc(
    positions: &mut Vec<[f32; 3]>,
    normals: &mut Vec<[f32; 3]>,
    indices: &mut Vec<u32>,
    next_index: &mut u32,
    center: Vec3,
    radius: f32,
    segments: u32,
) {
    let center_index = *next_index;
    positions.push([center.x, 0.0, center.z]);
    normals.push([0.0, 1.0, 0.0]);
    *next_index += 1;
    for step in 0..segments {
        let angle = (step as f32 / segments as f32) * std::f32::consts::TAU;
        positions.push([
            center.x + angle.cos() * radius,
            0.0,
            center.z + angle.sin() * radius,
        ]);
        normals.push([0.0, 1.0, 0.0]);
        *next_index += 1;
    }
    for step in 0..segments {
        indices.extend_from_slice(&[
            center_index,
            center_index + 1 + step,
            center_index + 1 + ((step + 1) % segments),
        ]);
    }
}

fn finalize_mesh(
    positions: Vec<[f32; 3]>,
    normals: Vec<[f32; 3]>,
    indices: Vec<u32>,
) -> Mesh {
    let mut mesh = Mesh::new(
        PrimitiveTopology::TriangleList,
        RenderAssetUsages::default(),
    );
    mesh.insert_attribute(Mesh::ATTRIBUTE_POSITION, positions);
    mesh.insert_attribute(Mesh::ATTRIBUTE_NORMAL, normals);
    mesh.insert_indices(Indices::U32(indices));
    mesh
}

/// R/G/B triad at world origin, length 1 m — mirrors ROS rviz's world frame.
fn draw_world_axes(toggles: Res<DisplayToggles>, mut gizmos: Gizmos) {
    if !toggles.show_world_axes {
        return;
    }
    draw_triad(&mut gizmos, Transform::default(), 1.0);
}

fn draw_triad(gizmos: &mut Gizmos, tf: Transform, size: f32) {
    let origin = tf.translation;
    let x = tf.rotation * Vec3::X * size;
    let y = tf.rotation * Vec3::Y * size;
    let z = tf.rotation * Vec3::Z * size;
    gizmos.line(origin, origin + x, Color::srgb(1.0, 0.1, 0.1));
    gizmos.line(origin, origin + y, Color::srgb(0.1, 1.0, 0.1));
    gizmos.line(origin, origin + z, Color::srgb(0.1, 0.3, 1.0));
}

fn draw_link_name_labels(
    toggles: Res<DisplayToggles>,
    mut contexts: EguiContexts,
    links: Query<(&RobotLink, &GlobalTransform)>,
    cameras: Query<(&Camera, &GlobalTransform)>,
) {
    if !toggles.show_link_names {
        return;
    }
    let Ok(ctx) = contexts.ctx_mut() else {
        return;
    };
    let Ok((camera, cam_tf)) = cameras.single() else {
        return;
    };

    egui::Area::new(egui::Id::new("bevy_urdf_link_names"))
        .fixed_pos(egui::pos2(0.0, 0.0))
        .show(ctx, |ui| {
            let painter = ui.painter();
            for (link, tf) in links.iter() {
                let world = tf.translation();
                let Ok(ndc) = camera.world_to_viewport(cam_tf, world) else {
                    continue;
                };
                painter.text(
                    egui::pos2(ndc.x, ndc.y),
                    egui::Align2::CENTER_CENTER,
                    &link.name,
                    egui::FontId::proportional(12.0),
                    egui::Color32::WHITE,
                );
            }
        });
}

fn isometry_to_transform(iso: &k::nalgebra::Isometry3<f32>) -> Transform {
    let tr = &iso.translation.vector;
    let q = &iso.rotation;
    Transform {
        translation: Vec3::new(tr.x, tr.y, tr.z),
        rotation: Quat::from_xyzw(q.i, q.j, q.k, q.w),
        scale: Vec3::ONE,
    }
}
