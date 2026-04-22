//! Mouse-driven IK manipulation.
//!
//! Ports the Shift+drag interaction from `urdf-viz::app::AppState` (app.rs
//! lines 887-917): mouse-delta in screen plane → world-plane delta →
//! updates an IK target isometry → `k::JacobianIkSolver` solves for joint
//! angles that reach it. On solver failure the previous joint state is
//! restored so a bad target can't leave the chain in a broken pose.
//!
//! Arms (IK chains) are enumerated once at load time: every leaf link
//! (link with no child joints) that is downstream of at least one movable
//! joint becomes a `SerialChain`. `UiState::active_arm` picks which one
//! drags with the mouse.

use std::collections::HashMap;

use bevy::input::mouse::AccumulatedMouseMotion;
use bevy::prelude::*;
use k::nalgebra as na;
use k::{InverseKinematicsSolver, JacobianIkSolver, SerialChain};

use crate::camera::ArcballCamera;
use crate::robot::{Robot, RobotRoot};
use crate::ui::UiState;

pub struct ManipulationPlugin;

impl Plugin for ManipulationPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<IkSolverRes>()
            .add_systems(
                Update,
                (
                    build_arms_after_spawn,
                    drag_ik,
                )
                    .chain(),
            );
    }
}

/// Arms = IK serial chains, built lazily after the Robot component is
/// inserted. Each arm ends at a leaf link (no children).
#[derive(Component)]
pub struct RobotArms {
    pub chains: Vec<SerialChain<f32>>,
    pub names: Vec<String>,
}

#[derive(Resource)]
struct IkSolverRes {
    solver: JacobianIkSolver<f32>,
}

impl Default for IkSolverRes {
    fn default() -> Self {
        // Same defaults as urdf-viz::app::AppState (app.rs:557-572):
        // allowable_error = 0.01 m, 0.01 rad; jacobian_mul = 0.5; 100 iter.
        Self {
            solver: JacobianIkSolver::new(0.01, 0.01, 0.5, 100),
        }
    }
}

fn build_arms_after_spawn(
    mut commands: Commands,
    robots: Query<(Entity, &Robot), (With<RobotRoot>, Without<RobotArms>)>,
) {
    for (entity, robot) in robots.iter() {
        // Identify leaf links: any link that never appears as a `parent`
        // of any joint. Leaves are candidate end-effectors.
        let parent_set: std::collections::HashSet<&str> = robot
            .urdf
            .joints
            .iter()
            .map(|j| j.parent.link.as_str())
            .collect();
        let child_to_joint: HashMap<&str, &str> = robot
            .urdf
            .joints
            .iter()
            .map(|j| (j.child.link.as_str(), j.name.as_str()))
            .collect();

        let mut chains = Vec::new();
        let mut names = Vec::new();
        for link in &robot.urdf.links {
            if parent_set.contains(link.name.as_str()) {
                continue;
            }
            // Find the k::Node for this link by walking up from the joint
            // whose child is this link.
            let Some(joint_name) = child_to_joint.get(link.name.as_str()) else {
                // Leaf that's ALSO the root? skip (no chain to solve).
                continue;
            };
            let Some(node) = robot.chain.find(joint_name) else {
                continue;
            };
            let serial = SerialChain::from_end(node);
            if serial.iter_joints().count() == 0 {
                continue;
            }
            chains.push(serial);
            names.push(link.name.clone());
        }

        if !chains.is_empty() {
            info!(
                "bevy_urdf: built {} IK arms: {:?}",
                chains.len(),
                names
            );
        }
        commands.entity(entity).insert(RobotArms { chains, names });
    }
}

/// Held across frames during a single Shift+drag gesture. `None` when the
/// user isn't dragging.
#[derive(Default)]
struct DragState {
    target: Option<na::Isometry3<f32>>,
    prev_positions: Option<Vec<f32>>,
}

fn drag_ik(
    buttons: Res<ButtonInput<MouseButton>>,
    keys: Res<ButtonInput<KeyCode>>,
    motion: Res<AccumulatedMouseMotion>,
    ui_state: Res<UiState>,
    solver_res: Res<IkSolverRes>,
    cameras: Query<&Transform, (With<Camera3d>, With<ArcballCamera>)>,
    arcballs: Query<&ArcballCamera>,
    robots: Query<(&Robot, &RobotArms), With<RobotRoot>>,
    mut drag: Local<DragState>,
) {
    let shift = keys.any_pressed([KeyCode::ShiftLeft, KeyCode::ShiftRight]);
    let lmb = buttons.pressed(MouseButton::Left);
    let drag_active = shift && lmb;

    if !drag_active {
        drag.target = None;
        drag.prev_positions = None;
        return;
    }

    let Ok((robot, arms)) = robots.single() else {
        return;
    };
    if arms.chains.is_empty() {
        return;
    }
    let arm_index = ui_state.active_arm.min(arms.chains.len() - 1);
    let arm = &arms.chains[arm_index];

    // Seed target = current end-effector pose on drag start.
    if drag.target.is_none() {
        robot.chain.update_transforms();
        let Some(end) = arm.iter().last() else {
            return;
        };
        let Some(iso) = end.world_transform() else {
            return;
        };
        drag.target = Some(iso);
        drag.prev_positions = Some(arm.joint_positions());
    }

    let Some(mut target) = drag.target.clone() else {
        return;
    };

    // Translate drag pixels → world-space delta using the camera's basis.
    // Scaling with arcball radius keeps drag sensitivity roughly constant
    // across zoom levels.
    let Ok(cam_tf) = cameras.single() else {
        return;
    };
    let arcball = arcballs.single().ok();
    let world_scale = arcball.map(|a| 0.003 * a.distance).unwrap_or(0.01);
    let right = cam_tf.rotation * Vec3::X;
    let up = cam_tf.rotation * Vec3::Y;
    let world_delta = right * (motion.delta.x * world_scale)
        + up * (-motion.delta.y * world_scale);

    target.translation.vector.x += world_delta.x;
    target.translation.vector.y += world_delta.y;
    target.translation.vector.z += world_delta.z;
    drag.target = Some(target.clone());

    // Snapshot positions before solve so we can revert on failure (matches
    // viewer.rs:908).
    let snapshot = arm.joint_positions();
    if let Err(e) = solver_res.solver.solve(arm, &target) {
        warn!("bevy_urdf: IK solve failed: {e}. reverting joint state.");
        if arm.set_joint_positions(&snapshot).is_err() {
            error!("bevy_urdf: IK revert failed — joint DOF mismatch?");
        }
    }
}
