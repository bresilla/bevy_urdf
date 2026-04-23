//! `bevy_egui` panels: joint sliders, arm selector, status line.
//!
//! Ports the UI half of `urdf-viz::viewer::App`: the side panel with one
//! slider per movable joint (respecting URDF limits), the arm selector
//! (cycled by `,`/`.` in urdf-viz), and a small status display.

use bevy::prelude::*;
use bevy_egui::{egui, EguiPlugin};

use crate::manipulation::RobotArms;
use crate::robot::Robot;

pub struct UrdfUiPlugin;

impl Plugin for UrdfUiPlugin {
    fn build(&self, app: &mut App) {
        // Only add EguiPlugin if the consumer hasn't already — users may
        // want their own egui config. `is_plugin_added` keeps us idempotent.
        if !app.is_plugin_added::<EguiPlugin>() {
            app.add_plugins(EguiPlugin::default());
        }
        app.init_resource::<UiState>();
        // No systems: consumers call `draw_joint_controls(ui, ...)` inside
        // their own egui window. This lets examples ship a single unified
        // panel instead of the library forcing a separate window.
    }
}

/// Selection / mode state shared between the UI and the manipulation
/// module. Mirrors urdf-viz's `AppState` cursor-position bookkeeping at the
/// UI layer: which arm is active, which joint is highlighted.
#[derive(Resource, Default)]
pub struct UiState {
    pub active_arm: usize,
    pub active_joint: usize,
}

/// Draw the IK arm selector + one slider per joint inside an already-open
/// `egui::Ui`. Host app is responsible for placing the window / panel.
pub fn draw_joint_controls(
    ui: &mut egui::Ui,
    robot: &Robot,
    arms: Option<&RobotArms>,
    ui_state: &mut UiState,
) {
    // Arm selector — drives `UiState::active_arm`, which the IK drag
    // system reads to pick which SerialChain to solve.
    if let Some(arms) = arms {
        if !arms.names.is_empty() {
            ui.label("IK Arm");
            let active_idx = ui_state.active_arm.min(arms.names.len() - 1);
            let current_label = arms.names[active_idx].clone();
            egui::ComboBox::from_id_salt("bevy_urdf_arm_selector")
                .selected_text(current_label)
                .show_ui(ui, |ui| {
                    for (i, name) in arms.names.iter().enumerate() {
                        ui.selectable_value(&mut ui_state.active_arm, i, name);
                    }
                });
            ui.separator();
        }
    }

    ui.label(egui::RichText::new("Joints").strong());
    ui.separator();

    // `joint_positions()` returns ONE entry per movable joint (fixed
    // joints are skipped). Walk `iter()` with the same `is_movable`
    // filter so names line up with positions — otherwise the first N
    // slider labels read "base_footprint", "imu_joint", etc. instead of
    // the actual wheel / arm joint names.
    let mut positions = robot.chain.joint_positions();
    let mut names: Vec<String> = Vec::with_capacity(positions.len());
    let mut limits: Vec<Option<(f32, f32)>> = Vec::with_capacity(positions.len());
    for node in robot.chain.iter() {
        let joint = node.joint();
        if !joint.is_movable() {
            continue;
        }
        names.push(joint.name.clone());
        limits.push(joint.limits.as_ref().map(|l| (l.min as f32, l.max as f32)));
    }

    egui::ScrollArea::vertical()
        .max_height(320.0)
        .show(ui, |ui| {
            for (i, (name, limit)) in names.iter().zip(limits.iter()).enumerate() {
                if i >= positions.len() {
                    continue;
                }
                let (lo, hi) = limit.unwrap_or((-std::f32::consts::PI, std::f32::consts::PI));
                ui.horizontal(|ui| {
                    if i == ui_state.active_joint {
                        ui.label(egui::RichText::new("▶").strong());
                    } else {
                        ui.label("  ");
                    }
                    let resp =
                        ui.add(egui::Slider::new(&mut positions[i], lo..=hi).text(name));
                    if resp.clicked() || resp.dragged() {
                        ui_state.active_joint = i;
                    }
                });
            }
        });

    // Push edits back to the chain.
    robot.chain.set_joint_positions_clamped(&positions);
}
