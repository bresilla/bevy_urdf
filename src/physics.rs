//! Rapier3d physics integration via `rapier3d-urdf`.
//!
//! Instead of hand-rolling URDF → collider conversion (which missed mesh
//! colliders and got joint axes wrong), we delegate to dimforge's
//! `rapier3d-urdf` crate. It builds rigid bodies + colliders + joints from
//! the URDF in one shot, including STL / OBJ / DAE trimesh colliders via
//! its `rapier3d-meshloader` backend.
//!
//! Because `bevy_urdf` drives kinematics from `k::Chain` (so IK has the
//! same solver urdf-viz uses), we keep the bodies as
//! `KinematicPositionBased` and update them from Bevy's `GlobalTransform`
//! each frame. Joints built by rapier3d-urdf are discarded — they'd fight
//! the IK-driven positions.

use std::collections::HashMap;

use bevy::prelude::*;
use rapier3d::math::{Pose, Vector};
use rapier3d::prelude::*;
use rapier3d_urdf::{UrdfLoaderOptions, UrdfRobot};

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

/// Marker put on a robot root once we've attached its rapier bodies so we
/// don't try to do it twice.
#[derive(Component)]
pub struct RobotPhysicsAttached;

fn attach_link_bodies(
    mut commands: Commands,
    mut world: ResMut<PhysicsWorld>,
    package_map: Res<PackageMap>,
    robots: Query<(Entity, &Robot), (With<RobotRoot>, Without<RobotPhysicsAttached>)>,
    q_links: Query<(Entity, &RobotLink)>,
) {
    for (robot_entity, robot) in robots.iter() {
        // Build an index of URDF-link-name → our link Entity, so we can
        // paste the rapier handles onto the right entities.
        let mut link_entity_by_name: HashMap<String, Entity> = HashMap::new();
        for (e, l) in q_links.iter() {
            link_entity_by_name.insert(l.name.clone(), e);
        }

        // rapier3d-urdf's mesh loader takes a single base directory. We
        // can't plug our `PackageMap` into it directly, so for robots
        // whose URDF uses `package://` we compute a common base. Fallback
        // is the URDF's own dir.
        let mesh_dir = resolve_mesh_base(&robot.urdf, &package_map, &robot.urdf_dir);

        let options = UrdfLoaderOptions {
            // Colliders only from <collision> — visual meshes become
            // Bevy-side renderables, not colliders.
            create_colliders_from_collision_shapes: true,
            create_colliders_from_visual_shapes: false,
            // We drive positions via FK, not physics. Fixed prevents
            // gravity / integrator churn; kinematic_position would also
            // work but Fixed is simpler while IK is the source of truth.
            make_roots_fixed: true,
            rigid_body_blueprint: RigidBodyBuilder::kinematic_position_based(),
            ..Default::default()
        };

        let urdf_robot = UrdfRobot::from_robot(&robot.urdf, options, &mesh_dir);

        // Insert bodies + colliders but skip joints. Joints would fight
        // the FK-driven transforms.
        let PhysicsWorld {
            bodies, colliders, ..
        } = &mut *world;
        for (i, urdf_link) in urdf_robot.links.into_iter().enumerate() {
            let mut body = urdf_link.body;
            // Force kinematic_position_based regardless of what the
            // loader picked (dynamic for non-root, fixed for root when
            // `make_roots_fixed`). We want unified FK-driven motion.
            body.set_body_type(rapier3d::dynamics::RigidBodyType::KinematicPositionBased, false);
            let body_handle = bodies.insert(body);
            for collider in urdf_link.colliders {
                colliders.insert_with_parent(collider, body_handle, bodies);
            }

            let urdf_name = &robot.urdf.links[i].name;
            if let Some(entity) = link_entity_by_name.get(urdf_name) {
                commands
                    .entity(*entity)
                    .insert(LinkPhysicsBody { handle: body_handle });
            }
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
        let t = tf.compute_transform();
        let pose = Pose::from_parts(t.translation, t.rotation);
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

/// rapier3d-urdf expects a single base directory; our `PackageMap` is
/// more flexible but its data isn't directly usable there. Heuristic:
/// if every mesh URI in the URDF uses the same `package://<pkg>/...`
/// prefix and that package is registered, return that package's root.
/// Otherwise fall back to the URDF file's own directory.
fn resolve_mesh_base(
    robot: &urdf_rs::Robot,
    package_map: &PackageMap,
    urdf_dir: &std::path::Path,
) -> std::path::PathBuf {
    let mut unique: Option<String> = None;
    for link in &robot.links {
        for collision in &link.collision {
            if let urdf_rs::Geometry::Mesh { filename, .. } = &collision.geometry {
                if let Some(pkg) = filename
                    .strip_prefix("package://")
                    .and_then(|s| s.split_once('/'))
                    .map(|(p, _)| p.to_string())
                {
                    match &unique {
                        None => unique = Some(pkg),
                        Some(existing) if existing == &pkg => {}
                        Some(_) => return urdf_dir.to_path_buf(),
                    }
                }
            }
        }
    }
    if let Some(pkg) = unique {
        if let Ok(p) = package_map.resolve(&format!("package://{pkg}/"), urdf_dir) {
            return p;
        }
    }
    urdf_dir.to_path_buf()
}
