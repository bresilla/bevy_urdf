//! Forward kinematics: drive link-entity `Transform`s from `k::Chain` joint
//! state.
//!
//! Mirrors `urdf-viz::viewer::Viewer::update` (viewer.rs:199) but uses the
//! `Vec<Isometry3>` that `update_transforms()` already returns, so we do
//! one pass over the chain per frame — no extra `RefCell` borrows inside
//! the inner loop (those were a suspected deadlock source).

use bevy::prelude::*;

use crate::robot::{Robot, RobotRoot};

pub struct KinematicsPlugin;

impl Plugin for KinematicsPlugin {
    fn build(&self, app: &mut App) {
        // FK runs in `PostUpdate`, BEFORE Bevy's transform propagation
        // so freshly-written link Transforms get pushed into their
        // GlobalTransform the same frame.
        app.add_systems(
            PostUpdate,
            sync_fk_to_transforms.before(bevy::transform::TransformSystems::Propagate),
        );
    }
}

fn sync_fk_to_transforms(
    robots: Query<&Robot, With<RobotRoot>>,
    mut transforms: Query<&mut Transform>,
) {
    for robot in robots.iter() {
        // Single call to update_transforms → returns world poses in the
        // same order as `chain.iter()`. We zip them together without
        // re-borrowing `node.joint()` per step.
        let world_poses = robot.chain.update_transforms();
        for (node, iso) in robot.chain.iter().zip(world_poses.iter()) {
            // Take the joint name by clone once and release the borrow —
            // the rest of the iteration does not touch `node` again, so
            // any internal RefCell guard is dropped before we hit the
            // ECS query.
            let joint_name = node.joint().name.clone();
            let Some(child_link_name) = robot.joint_to_child_link.get(&joint_name) else {
                continue;
            };
            let Some(entity) = robot.link_entities.get(child_link_name).copied() else {
                continue;
            };
            if let Ok(mut t) = transforms.get_mut(entity) {
                *t = isometry_to_transform(iso);
            }
        }
    }
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
