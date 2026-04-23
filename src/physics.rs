//! Rapier3d (f64) physics integration.
//!
//! Each URDF link is mirrored as a kinematic-position-based rigid body
//! in the rapier world. Bevy's `GlobalTransform` (f32) is upcast to
//! `f64` each frame and pushed to the body so physics tracks the
//! FK-driven pose. Colliders are built directly from URDF geometry —
//! we previously used `rapier3d-urdf`, but that crate wraps only the
//! f32 rapier flavour, so the conversion is inlined here.

use std::collections::HashMap;

use bevy::prelude::*;
use rapier3d_f64::math::{Pose, Real, Vector};
use rapier3d_f64::prelude::*;

use crate::mesh::load_raw_mesh;
use crate::robot::{Robot, RobotLink, RobotRoot};
use crate::urdf::PackageMap;

pub struct UrdfPhysicsPlugin;

impl Plugin for UrdfPhysicsPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<PhysicsWorld>().add_systems(
            PostUpdate,
            (attach_link_bodies, drive_kinematic_bodies, step_physics)
                .chain()
                .after(bevy::transform::TransformSystems::Propagate),
        );
    }
}

#[derive(Resource)]
pub struct PhysicsWorld {
    pub pipeline: PhysicsPipeline,
    pub integration_parameters: IntegrationParameters,
    pub islands: IslandManager,
    pub broad_phase: BroadPhaseBvh,
    pub narrow_phase: NarrowPhase,
    pub bodies: RigidBodySet,
    pub colliders: ColliderSet,
    pub impulse_joints: ImpulseJointSet,
    pub multibody_joints: MultibodyJointSet,
    pub ccd_solver: CCDSolver,
    pub gravity: Vector,
}

impl Default for PhysicsWorld {
    fn default() -> Self {
        Self {
            pipeline: PhysicsPipeline::new(),
            integration_parameters: IntegrationParameters::default(),
            islands: IslandManager::new(),
            broad_phase: BroadPhaseBvh::new(),
            narrow_phase: NarrowPhase::new(),
            bodies: RigidBodySet::new(),
            colliders: ColliderSet::new(),
            impulse_joints: ImpulseJointSet::new(),
            multibody_joints: MultibodyJointSet::new(),
            ccd_solver: CCDSolver::new(),
            gravity: Vector::new(0.0, -9.81, 0.0),
        }
    }
}

#[derive(Component)]
pub struct LinkPhysicsBody {
    pub handle: RigidBodyHandle,
}

#[derive(Component)]
pub struct RobotPhysicsAttached;

fn attach_link_bodies(
    mut commands: Commands,
    mut world: ResMut<PhysicsWorld>,
    package_map: Res<PackageMap>,
    robots: Query<(Entity, &Robot), (With<RobotRoot>, Without<RobotPhysicsAttached>)>,
    q_links: Query<(Entity, &RobotLink, &GlobalTransform)>,
) {
    for (robot_entity, robot) in robots.iter() {
        let mut link_by_name: HashMap<String, (Entity, GlobalTransform)> = HashMap::new();
        for (e, l, tf) in q_links.iter() {
            link_by_name.insert(l.name.clone(), (e, *tf));
        }

        let urdf_links: HashMap<&str, &urdf_rs::Link> = robot
            .urdf
            .links
            .iter()
            .map(|l| (l.name.as_str(), l))
            .collect();

        let PhysicsWorld {
            bodies, colliders, ..
        } = &mut *world;

        for (name, (entity, tf)) in &link_by_name {
            let Some(urdf_link) = urdf_links.get(name.as_str()) else {
                continue;
            };
            if urdf_link.collision.is_empty() {
                continue;
            }

            let pose = transform_to_pose(&tf.compute_transform());
            let body = RigidBodyBuilder::kinematic_position_based()
                .pose(pose)
                .build();
            let body_handle = bodies.insert(body);

            for collision in &urdf_link.collision {
                let local = urdf_pose_to_pose(&collision.origin);
                if let Some(collider) = collider_from_urdf_geom(
                    &collision.geometry,
                    local,
                    &package_map,
                    robot.urdf_dir.as_path(),
                ) {
                    colliders.insert_with_parent(collider, body_handle, bodies);
                }
            }

            commands
                .entity(*entity)
                .insert(LinkPhysicsBody { handle: body_handle });
        }

        commands.entity(robot_entity).insert(RobotPhysicsAttached);
    }
}

fn drive_kinematic_bodies(
    mut world: ResMut<PhysicsWorld>,
    q: Query<(&LinkPhysicsBody, &GlobalTransform)>,
) {
    for (lpb, tf) in q.iter() {
        let Some(body) = world.bodies.get_mut(lpb.handle) else {
            continue;
        };
        let pose = transform_to_pose(&tf.compute_transform());
        body.set_next_kinematic_position(pose);
    }
}

fn step_physics(mut world: ResMut<PhysicsWorld>) {
    let PhysicsWorld {
        pipeline,
        integration_parameters,
        islands,
        broad_phase,
        narrow_phase,
        bodies,
        colliders,
        impulse_joints,
        multibody_joints,
        ccd_solver,
        gravity,
    } = &mut *world;
    pipeline.step(
        *gravity,
        integration_parameters,
        islands,
        broad_phase,
        narrow_phase,
        bodies,
        colliders,
        impulse_joints,
        multibody_joints,
        ccd_solver,
        &(),
        &(),
    );
}

/// Bevy `Transform` (f32) → rapier `Pose` (f64). glam DVec3 + DQuat are
/// used by `parry3d-f64`'s glamx backend; they have `::new` constructors
/// that accept f64 scalars, so we upcast on the spot.
fn transform_to_pose(t: &Transform) -> Pose {
    let tr = t.translation;
    let r = t.rotation;
    Pose::from_parts(
        Vector::new(tr.x as Real, tr.y as Real, tr.z as Real),
        Rotation::from_xyzw(r.x as Real, r.y as Real, r.z as Real, r.w as Real),
    )
}

fn urdf_pose_to_pose(pose: &urdf_rs::Pose) -> Pose {
    let translation = Vector::new(pose.xyz[0], pose.xyz[1], pose.xyz[2]);
    let rot = Rotation::from_euler(
        rapier3d_f64::glamx::EulerRot::XYZ,
        pose.rpy[0],
        pose.rpy[1],
        pose.rpy[2],
    );
    Pose::from_parts(translation, rot)
}

fn collider_from_urdf_geom(
    geom: &urdf_rs::Geometry,
    local: Pose,
    package_map: &PackageMap,
    urdf_dir: &std::path::Path,
) -> Option<Collider> {
    let builder = match geom {
        urdf_rs::Geometry::Box { size } => Some(ColliderBuilder::cuboid(
            size[0] * 0.5,
            size[1] * 0.5,
            size[2] * 0.5,
        )),
        urdf_rs::Geometry::Cylinder { radius, length } => {
            Some(ColliderBuilder::cylinder(*length * 0.5, *radius))
        }
        urdf_rs::Geometry::Capsule { radius, length } => {
            // URDF capsule extends along Z ± length/2.
            Some(ColliderBuilder::capsule_z(*length * 0.5, *radius))
        }
        urdf_rs::Geometry::Sphere { radius } => Some(ColliderBuilder::ball(*radius)),
        urdf_rs::Geometry::Mesh { filename, scale } => {
            let path = match package_map.resolve(filename, urdf_dir) {
                Ok(p) => p,
                Err(e) => {
                    warn!("bevy_urdf: mesh collider resolve failed for {filename}: {e}");
                    return None;
                }
            };
            let raw = match load_raw_mesh(&path) {
                Ok(m) => m,
                Err(e) => {
                    warn!("bevy_urdf: mesh collider load failed for {path:?}: {e}");
                    return None;
                }
            };
            let scale_vec = scale
                .as_ref()
                .map(|s| [s[0], s[1], s[2]])
                .unwrap_or([1.0, 1.0, 1.0]);
            let vertices: Vec<Vector> = raw
                .vertices
                .iter()
                .map(|v| {
                    Vector::new(
                        v[0] as Real * scale_vec[0],
                        v[1] as Real * scale_vec[1],
                        v[2] as Real * scale_vec[2],
                    )
                })
                .collect();
            let indices: Vec<[u32; 3]> = raw.faces.clone();
            if vertices.is_empty() || indices.is_empty() {
                warn!("bevy_urdf: mesh collider skipped (empty mesh {path:?})");
                return None;
            }
            Some(ColliderBuilder::trimesh(vertices, indices).ok()?)
        }
    };
    builder.map(|b| b.position(local).build())
}
