//! Keyboard shortcuts ported from `urdf-viz::app::AppState::handle_key_press`
//! (app.rs:442-547).
//!
//! Kept in its own module so the controls stay co-located and the
//! manipulation / UI modules don't grow a dumping-ground of `just_pressed`
//! branches.

use bevy::prelude::*;
use rand::RngExt;

use crate::robot::{Robot, RobotRoot};
use crate::ui::UiState;

pub struct UrdfKeyboardPlugin;

impl Plugin for UrdfKeyboardPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<DisplayToggles>()
            .add_systems(Update, handle_keys);
    }
}

/// Rendering toggles driven by keyboard. Actual visibility mutation lives
/// in the overlay module (#10); this is just shared state the systems read.
#[derive(Resource, Debug, Clone)]
pub struct DisplayToggles {
    pub show_visual: bool,
    pub show_collision: bool,
    pub show_joint_frames: bool,
    pub show_link_frames: bool,
    pub show_link_names: bool,
    /// A 20×20 m ground grid with lines every 1 m. On by default because
    /// it anchors the eye when a URDF lands far from origin.
    pub show_world_grid: bool,
    /// R/G/B axis triad at world origin.
    pub show_world_axes: bool,
    /// Opacity multiplier applied to every visual geom's `base_color`.
    /// 1.0 = solid, 0.0 = invisible. Useful for peering inside the
    /// robot to see joint frames / collision shells.
    pub body_opacity: f32,
}

impl Default for DisplayToggles {
    fn default() -> Self {
        Self {
            show_visual: true,
            show_collision: false,
            show_joint_frames: false,
            show_link_frames: false,
            show_link_names: false,
            show_world_grid: true,
            // Off by default — even tucked into the grid corner the R/G/B
            // triad competes with the robot for the eye. Opt in via the
            // Overlays panel when you need it.
            show_world_axes: false,
            body_opacity: 1.0,
        }
    }
}

fn handle_keys(
    keys: Res<ButtonInput<KeyCode>>,
    mut ui_state: ResMut<UiState>,
    mut toggles: ResMut<DisplayToggles>,
    robots: Query<&Robot, With<RobotRoot>>,
) {
    let Ok(robot) = robots.single() else {
        return;
    };

    let dof = robot.chain.dof();
    if dof == 0 {
        return;
    }

    // o/p — cycle active joint (app.rs:469-487 "JointUp/Down").
    if keys.just_pressed(KeyCode::KeyO) && dof > 0 {
        ui_state.active_joint = (ui_state.active_joint + dof - 1) % dof;
    }
    if keys.just_pressed(KeyCode::KeyP) && dof > 0 {
        ui_state.active_joint = (ui_state.active_joint + 1) % dof;
    }

    // ,/. — cycle active arm. Only meaningful once the arms component is
    // populated; we clamp instead of wrapping because we don't see the arm
    // count from here. UI clamps again on render.
    if keys.just_pressed(KeyCode::Comma) {
        ui_state.active_arm = ui_state.active_arm.saturating_sub(1);
    }
    if keys.just_pressed(KeyCode::Period) {
        ui_state.active_arm = ui_state.active_arm.saturating_add(1);
    }

    // Arrows — nudge the active joint angle. urdf-viz uses 0.1 rad per tick
    // and holds down the key for continuous motion (app.rs:518-531).
    let nudge_step = 0.02;
    let mut delta = 0.0;
    if keys.pressed(KeyCode::ArrowUp) || keys.pressed(KeyCode::ArrowRight) {
        delta += nudge_step;
    }
    if keys.pressed(KeyCode::ArrowDown) || keys.pressed(KeyCode::ArrowLeft) {
        delta -= nudge_step;
    }
    if delta != 0.0 {
        let mut positions = robot.chain.joint_positions();
        if ui_state.active_joint < positions.len() {
            positions[ui_state.active_joint] += delta;
            robot.chain.set_joint_positions_clamped(&positions);
        }
    }

    // r — randomize all joints (app.rs:504 "R").
    if keys.just_pressed(KeyCode::KeyR) {
        let mut rng = rand::rng();
        let mut positions = robot.chain.joint_positions();
        // Use per-joint limits if present; otherwise ±π.
        let mut limits: Vec<Option<(f32, f32)>> = Vec::with_capacity(positions.len());
        for node in robot.chain.iter() {
            let j = node.joint();
            limits.push(j.limits.as_ref().map(|l| (l.min as f32, l.max as f32)));
        }
        for (i, p) in positions.iter_mut().enumerate() {
            let (lo, hi) = limits[i]
                .unwrap_or((-std::f32::consts::PI, std::f32::consts::PI));
            *p = rng.random_range(lo..=hi);
        }
        robot.chain.set_joint_positions_clamped(&positions);
    }

    // z — reset all joints to zero (app.rs:506 "Z").
    if keys.just_pressed(KeyCode::KeyZ) {
        let zeros = vec![0.0_f32; robot.chain.dof()];
        robot.chain.set_joint_positions_clamped(&zeros);
    }

    // v/f/n/l — display toggles (handled visually in #10, stored here so
    // the overlay module has a single source of truth).
    if keys.just_pressed(KeyCode::KeyV) {
        toggles.show_visual = !toggles.show_visual;
        toggles.show_collision = !toggles.show_collision;
    }
    if keys.just_pressed(KeyCode::KeyF) {
        toggles.show_joint_frames = !toggles.show_joint_frames;
    }
    if keys.just_pressed(KeyCode::KeyN) {
        toggles.show_link_names = !toggles.show_link_names;
    }
    if keys.just_pressed(KeyCode::KeyL) {
        toggles.show_link_frames = !toggles.show_link_frames;
    }
}
