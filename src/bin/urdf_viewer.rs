//! URDF viewer binary. Curated catalog of arms / mobile / humanoid /
//! quadruped / gripper URDFs, plus a "Browse…" button for loading any
//! local URDF.
//!
//!   cargo run --bin urdf-viewer
//!   cargo run --bin urdf-viewer -- ur5
//!
//! Mouse: L+R drag orbit · middle drag pan · scroll zoom.
//! Shift+L-drag pulls the IK target · Ctrl+L-drag nudges the active joint.

use std::path::PathBuf;

use bevy::prelude::*;
use bevy_egui::{egui, EguiContexts, EguiPrimaryContextPass};
use bevy_urdf::{
    draw_joint_controls, ArcballCamera, LoadRobot, PackageMap, Robot, RobotArms, RobotRoot,
    UiState, UrdfPlugin,
};

#[derive(Clone, Copy, PartialEq, Eq)]
enum Category {
    Arm,
    Mobile,
    Humanoid,
    Quadruped,
    Gripper,
}

impl Category {
    fn label(self) -> &'static str {
        match self {
            Category::Arm => "Arms",
            Category::Mobile => "Mobile",
            Category::Humanoid => "Humanoids",
            Category::Quadruped => "Quadrupeds",
            Category::Gripper => "Grippers",
        }
    }
}

#[derive(Clone)]
struct RobotEntry {
    key: &'static str,
    label: &'static str,
    urdf: &'static str,
    packages: &'static [(&'static str, &'static str)],
    focus: Vec3,
    distance: f32,
    category: Category,
}

fn catalog() -> &'static [RobotEntry] {
    use std::sync::OnceLock;
    static CATALOG: OnceLock<Vec<RobotEntry>> = OnceLock::new();
    CATALOG.get_or_init(|| vec![
        RobotEntry { key: "panda", label: "Franka Panda", urdf: "urdf/oems/xacro_generated/franka_emika/franka_description/robots/panda/panda.urdf",
            packages: &[("franka_description", "urdf/oems/xacro_generated/franka_emika/franka_description")],
            focus: Vec3::new(0.0, 0.4, 0.0), distance: 2.5, category: Category::Arm },
        RobotEntry { key: "ur3", label: "Universal UR3", urdf: "urdf/matlab/ur_description/urdf/universalUR3.urdf",
            packages: &[("ur_description", "urdf/matlab/ur_description")],
            focus: Vec3::new(0.0, 0.3, 0.0), distance: 1.4, category: Category::Arm },
        RobotEntry { key: "ur5", label: "Universal UR5", urdf: "urdf/matlab/ur_description/urdf/universalUR5.urdf",
            packages: &[("ur_description", "urdf/matlab/ur_description")],
            focus: Vec3::new(0.0, 0.5, 0.0), distance: 1.8, category: Category::Arm },
        RobotEntry { key: "ur10", label: "Universal UR10", urdf: "urdf/matlab/ur_description/urdf/universalUR10.urdf",
            packages: &[("ur_description", "urdf/matlab/ur_description")],
            focus: Vec3::new(0.0, 0.7, 0.0), distance: 2.5, category: Category::Arm },
        RobotEntry { key: "iiwa", label: "KUKA iiwa14 (Drake)", urdf: "urdf/drake/iiwa_description/urdf/iiwa14_primitive_collision.urdf",
            packages: &[("drake", "urdf/drake/iiwa_description/manipulation_compat")],
            focus: Vec3::new(0.0, 0.6, 0.0), distance: 2.0, category: Category::Arm },
        RobotEntry { key: "iiwa_matlab", label: "KUKA iiwa14 (Matlab)", urdf: "urdf/matlab/iiwa_description/urdf/kukaIiwa14.urdf",
            packages: &[("iiwa_description", "urdf/matlab/iiwa_description")],
            focus: Vec3::new(0.0, 0.6, 0.0), distance: 2.0, category: Category::Arm },
        RobotEntry { key: "iiwa7", label: "KUKA iiwa7", urdf: "urdf/matlab/iiwa_description/urdf/kukaIiwa7.urdf",
            packages: &[("iiwa_description", "urdf/matlab/iiwa_description")],
            focus: Vec3::new(0.0, 0.5, 0.0), distance: 1.8, category: Category::Arm },
        RobotEntry { key: "abb", label: "ABB IRB 120", urdf: "urdf/matlab/abb_irb120_support/urdf/abbIrb120.urdf",
            packages: &[("abb_irb120_support", "urdf/matlab/abb_irb120_support")],
            focus: Vec3::new(0.0, 0.3, 0.0), distance: 1.4, category: Category::Arm },
        RobotEntry { key: "kinova", label: "Kinova Gen3", urdf: "urdf/matlab/kortex_description/urdf/kinovaGen3.urdf",
            packages: &[("JACO3_URDF_V10", "urdf/matlab/kortex_description")],
            focus: Vec3::new(0.0, 0.4, 0.0), distance: 1.6, category: Category::Arm },
        RobotEntry { key: "qarm", label: "Quanser QArm", urdf: "urdf/matlab/qarm_description/urdf/quanserQArm.urdf",
            packages: &[("QARM", "urdf/matlab/qarm_description")],
            focus: Vec3::new(0.0, 0.2, 0.0), distance: 0.8, category: Category::Arm },
        RobotEntry { key: "sawyer", label: "Rethink Sawyer", urdf: "urdf/matlab/sawyer_description/urdf/rethinkSawyer.urdf",
            packages: &[("sawyer_description", "urdf/matlab/sawyer_description")],
            focus: Vec3::new(0.0, 0.7, 0.0), distance: 2.5, category: Category::Arm },
        RobotEntry { key: "openmanip", label: "Robotis OpenManipulator", urdf: "urdf/matlab/open_manipulator_description/urdf/robotisOpenManipulator.urdf",
            packages: &[("open_manipulator_description", "urdf/matlab/open_manipulator_description")],
            focus: Vec3::new(0.0, 0.1, 0.0), distance: 0.6, category: Category::Arm },
        RobotEntry { key: "meca", label: "Mecademic Meca500", urdf: "urdf/matlab/mecademic_description/urdf/meca500r3.urdf",
            packages: &[("mecademic_description", "urdf/matlab/mecademic_description")],
            focus: Vec3::new(0.0, 0.2, 0.0), distance: 0.8, category: Category::Arm },
        RobotEntry { key: "fanuc", label: "Fanuc M16iB", urdf: "urdf/matlab/fanuc_m16ib_support/urdf/fanucM16ib.urdf",
            packages: &[("fanuc_m16ib_support", "urdf/matlab/fanuc_m16ib_support")],
            focus: Vec3::new(0.0, 0.6, 0.0), distance: 2.2, category: Category::Arm },
        RobotEntry { key: "pioneer3at", label: "Pioneer 3-AT", urdf: "urdf/matlab/amr_robots_description/urdf/amrPioneer3AT.urdf",
            packages: &[("amr_robots_description", "urdf/matlab/amr_robots_description")],
            focus: Vec3::new(0.0, 0.2, 0.0), distance: 1.4, category: Category::Mobile },
        RobotEntry { key: "pioneer3dx", label: "Pioneer 3-DX", urdf: "urdf/matlab/amr_robots_description/urdf/amrPioneer3DX.urdf",
            packages: &[("amr_robots_description", "urdf/matlab/amr_robots_description")],
            focus: Vec3::new(0.0, 0.2, 0.0), distance: 1.4, category: Category::Mobile },
        RobotEntry { key: "pioneerlx", label: "Pioneer LX", urdf: "urdf/matlab/amr_robots_description/urdf/amrPioneerLX.urdf",
            packages: &[("amr_robots_description", "urdf/matlab/amr_robots_description")],
            focus: Vec3::new(0.0, 0.2, 0.0), distance: 1.6, category: Category::Mobile },
        RobotEntry { key: "husky", label: "Clearpath Husky", urdf: "urdf/matlab/husky_description/urdf/clearpathHusky.urdf",
            packages: &[("husky_description", "urdf/matlab/husky_description")],
            focus: Vec3::new(0.0, 0.2, 0.0), distance: 1.8, category: Category::Mobile },
        RobotEntry { key: "jackal", label: "Clearpath Jackal", urdf: "urdf/matlab/jackal_description/urdf/clearpathJackal.urdf",
            packages: &[("jackal_description", "urdf/matlab/jackal_description")],
            focus: Vec3::new(0.0, 0.15, 0.0), distance: 1.2, category: Category::Mobile },
        RobotEntry { key: "tb3_burger", label: "TurtleBot3 Burger", urdf: "urdf/matlab/turtlebot3_description/urdf/robotisTurtleBot3Burger.urdf",
            packages: &[("turtlebot3_description", "urdf/matlab/turtlebot3_description")],
            focus: Vec3::new(0.0, 0.1, 0.0), distance: 0.7, category: Category::Mobile },
        RobotEntry { key: "tb3_waffle", label: "TurtleBot3 Waffle Pi", urdf: "urdf/matlab/turtlebot3_description/urdf/robotisTurtleBot3WafflePi.urdf",
            packages: &[("turtlebot3_description", "urdf/matlab/turtlebot3_description")],
            focus: Vec3::new(0.0, 0.1, 0.0), distance: 0.8, category: Category::Mobile },
        RobotEntry { key: "fetch", label: "Fetch Robotics Fetch", urdf: "urdf/robotics-toolbox/fetch_description/robots/fetch.urdf",
            packages: &[("fetch_description", "urdf/robotics-toolbox/fetch_description")],
            focus: Vec3::new(0.0, 0.6, 0.0), distance: 2.5, category: Category::Mobile },
        RobotEntry { key: "movo", label: "Kinova MOVO", urdf: "urdf/matlab/movo_description/urdf/kinovaMovo.urdf",
            packages: &[("movo_description", "urdf/matlab/movo_description")],
            focus: Vec3::new(0.0, 0.7, 0.0), distance: 2.8, category: Category::Mobile },
        RobotEntry { key: "pr2", label: "Willow Garage PR2", urdf: "urdf/matlab/pr2_description/robots/willowgaragePR2.urdf",
            packages: &[("pr2_description", "urdf/matlab/pr2_description")],
            focus: Vec3::new(0.0, 0.8, 0.0), distance: 3.2, category: Category::Mobile },
        RobotEntry { key: "robotis_op2", label: "Robotis OP2", urdf: "urdf/matlab/robotis_op_description/robots/robotisOP2.urdf",
            packages: &[("robotis_op_description", "urdf/matlab/robotis_op_description")],
            focus: Vec3::new(0.0, 0.2, 0.0), distance: 1.0, category: Category::Humanoid },
        RobotEntry { key: "atlas", label: "Boston Dynamics Atlas", urdf: "urdf/matlab/Atlas/urdf/atlas.urdf",
            packages: &[("Atlas", "urdf/matlab/Atlas")],
            focus: Vec3::new(0.0, 0.9, 0.0), distance: 3.0, category: Category::Humanoid },
        RobotEntry { key: "baxter", label: "Rethink Baxter", urdf: "urdf/matlab/baxter_description/urdf/rethinkBaxter.urdf",
            packages: &[("baxter_description", "urdf/matlab/baxter_description")],
            focus: Vec3::new(0.0, 1.1, 0.0), distance: 3.0, category: Category::Humanoid },
        RobotEntry { key: "anymal", label: "ANYmal B (best-effort DAE)", urdf: "urdf/oems/anymal_anybotics/anymal_b_simple_description/urdf/anymal.urdf",
            packages: &[("meshes", "urdf/oems/anymal_anybotics/anymal_b_simple_description/meshes")],
            focus: Vec3::new(0.0, 0.3, 0.0), distance: 2.0, category: Category::Quadruped },
        RobotEntry { key: "spot", label: "Boston Dynamics Spot", urdf: "urdf/oems/spot_boston_dynamics/spot_base_urdf/model.urdf",
            packages: &[],
            focus: Vec3::new(0.0, 0.3, 0.0), distance: 2.0, category: Category::Quadruped },
        RobotEntry { key: "robotiq_2f85", label: "Robotiq 2F-85", urdf: "urdf/matlab/robotiq2F85/urdf/robotiq2F85.urdf",
            // URDF asks for `package://robotiq_2f_85_gripper_visualization/...`
            // — NOT the folder name. Map the actual URI to the local dir.
            packages: &[("robotiq_2f_85_gripper_visualization", "urdf/matlab/robotiq2F85")],
            focus: Vec3::new(0.0, 0.05, 0.0), distance: 0.3, category: Category::Gripper },
        RobotEntry { key: "allegro", label: "Allegro Hand", urdf: "urdf/drake/allegro_hand_description/urdf/allegro_hand_description_right.urdf",
            packages: &[("drake", "urdf/drake/allegro_hand_description/manipulation_compat")],
            focus: Vec3::new(0.0, 0.1, 0.0), distance: 0.5, category: Category::Gripper },
    ])
}

fn find_entry(key: &str) -> Option<&'static RobotEntry> {
    catalog().iter().find(|e| e.key == key)
}

fn main() {
    let initial_key = std::env::args().nth(1).unwrap_or_else(|| "panda".into());
    let initial = find_entry(&initial_key)
        .map(|e| PendingLoad::Catalog(e.key.to_string()))
        .unwrap_or_else(|| PendingLoad::Catalog("panda".to_string()));

    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins(UrdfPlugin)
        .insert_resource(Selection {
            pending: Some(initial),
            current: None,
            custom_path: None,
            custom_label: None,
        })
        .insert_resource(LeftTab::Selection)
        .add_systems(Startup, setup)
        .add_systems(Update, apply_pending_load)
        .add_systems(EguiPrimaryContextPass, (setup_egui_style, draw_panel).chain())
        .run();
}

enum PendingLoad {
    Catalog(String),
    Custom(PathBuf),
}

#[derive(Resource)]
struct Selection {
    /// A load request that hasn't been applied yet. Consumed by
    /// `apply_pending_load` on the next Update tick.
    pending: Option<PendingLoad>,
    /// The most recently *applied* selection, used for the picker's
    /// displayed label. Survives `pending.take()`.
    current: Option<AppliedSelection>,
    custom_path: Option<PathBuf>,
    custom_label: Option<String>,
}

#[derive(Clone)]
enum AppliedSelection {
    Catalog(String),
    Custom(PathBuf),
}

#[derive(Resource, Default)]
struct CurrentRobot {
    root: Option<Entity>,
}

fn setup(mut commands: Commands) {
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
    commands.init_resource::<CurrentRobot>();

    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(3.0, 2.0, 3.0).looking_at(Vec3::new(0.0, 0.4, 0.0), Vec3::Y),
        ArcballCamera {
            focus: Vec3::new(0.0, 0.4, 0.0),
            distance: 3.0,
            ..default()
        },
    ));
}

fn setup_egui_style(mut contexts: EguiContexts, mut done: Local<bool>) {
    if *done {
        return;
    }
    let Ok(ctx) = contexts.ctx_mut() else {
        return;
    };
    // Simple translucent dark theme — matches the look the user had
    // before the gearbox-style port. No embedded font, no section cards.
    let mut visuals = egui::Visuals::dark();
    visuals.window_fill = egui::Color32::from_rgba_unmultiplied(24, 26, 32, 235);
    visuals.panel_fill = visuals.window_fill;
    visuals.window_corner_radius = 8.0.into();
    visuals.window_shadow = egui::epaint::Shadow {
        offset: [0, 4],
        blur: 16,
        spread: 0,
        color: egui::Color32::from_black_alpha(150),
    };
    for w in [
        &mut visuals.widgets.active,
        &mut visuals.widgets.hovered,
        &mut visuals.widgets.inactive,
        &mut visuals.widgets.open,
        &mut visuals.widgets.noninteractive,
    ] {
        w.corner_radius = 4.0.into();
    }
    visuals.selection.bg_fill = egui::Color32::from_rgb(70, 125, 200);
    ctx.set_visuals(visuals);

    let mut style = (*ctx.style()).clone();
    style.spacing.item_spacing = egui::vec2(8.0, 6.0);
    style.spacing.slider_width = 160.0;
    style.spacing.window_margin = egui::Margin::same(10);
    ctx.set_style(style);
    *done = true;
}

fn apply_pending_load(
    mut commands: Commands,
    mut selection: ResMut<Selection>,
    mut current: ResMut<CurrentRobot>,
    mut package_map: ResMut<PackageMap>,
    mut cameras: Query<&mut ArcballCamera>,
) {
    let Some(pending) = selection.pending.take() else {
        return;
    };

    let manifest_dir = PathBuf::from(env!("CARGO_MANIFEST_DIR"));

    let (urdf_path, focus, distance, label, applied) = match &pending {
        PendingLoad::Catalog(key) => {
            let entry = match find_entry(key) {
                Some(e) => e,
                None => {
                    error!("unknown robot key: {key}");
                    return;
                }
            };
            for (pkg, dir) in entry.packages {
                package_map.insert(*pkg, manifest_dir.join(dir));
            }
            for (pkg, dir) in entry.packages {
                if *pkg == "drake" {
                    let compat = manifest_dir.join(dir);
                    let real_pkg = compat.parent().unwrap().to_path_buf();
                    let _ = std::fs::create_dir_all(compat.join("manipulation/models"));
                    let link_name = real_pkg.file_name().unwrap().to_owned();
                    let link = compat.join("manipulation/models").join(&link_name);
                    if !link.exists() {
                        let _ = std::os::unix::fs::symlink(&real_pkg, &link);
                    }
                }
            }
            (
                manifest_dir.join(entry.urdf),
                entry.focus,
                entry.distance,
                entry.label.to_string(),
                AppliedSelection::Catalog(entry.key.to_string()),
            )
        }
        PendingLoad::Custom(p) => {
            install_package_map_for_custom(p, &mut package_map);
            selection.custom_path = Some(p.clone());
            selection.custom_label = p
                .file_name()
                .and_then(|n| n.to_str())
                .map(|s| s.to_string());
            (
                p.clone(),
                Vec3::new(0.0, 0.4, 0.0),
                2.5,
                p.display().to_string(),
                AppliedSelection::Custom(p.clone()),
            )
        }
    };

    if let Some(prev) = current.root.take() {
        commands.entity(prev).despawn();
    }
    if let Ok(mut cam) = cameras.single_mut() {
        cam.focus = focus;
        cam.distance = distance;
    }

    // Spawn a plain top-level entity as the robot root; the URDF loader
    // attaches `Robot` + spawns link children onto it.
    let robot_root = commands
        .spawn((
            Transform::default(),
            GlobalTransform::default(),
            Visibility::default(),
            InheritedVisibility::default(),
            ViewVisibility::default(),
        ))
        .id();
    commands.write_message(LoadRobot {
        path: urdf_path,
        root: Some(robot_root),
    });
    current.root = Some(robot_root);
    selection.current = Some(applied);
    info!("urdf-viewer: loading {label}");
}

fn install_package_map_for_custom(urdf_path: &std::path::Path, map: &mut PackageMap) {
    let text = match std::fs::read_to_string(urdf_path) {
        Ok(t) => t,
        Err(e) => {
            warn!("urdf-viewer: read {:?} failed: {e}", urdf_path);
            return;
        }
    };
    let urdf_dir = urdf_path
        .parent()
        .map(|p| p.to_path_buf())
        .unwrap_or_else(|| PathBuf::from("."));
    let candidates = [
        urdf_dir.clone(),
        urdf_dir.parent().map(|p| p.to_path_buf()).unwrap_or(urdf_dir.clone()),
        urdf_dir
            .parent()
            .and_then(|p| p.parent())
            .map(|p| p.to_path_buf())
            .unwrap_or(urdf_dir.clone()),
    ];

    let mut seen = std::collections::HashSet::new();
    for cap in text.split("package://").skip(1) {
        let Some(end) = cap.find('"').or_else(|| cap.find('\'')) else {
            continue;
        };
        let uri = &cap[..end];
        let Some((pkg, rest)) = uri.split_once('/') else {
            continue;
        };
        if !seen.insert(pkg.to_string()) {
            continue;
        }
        let mut best = None;
        for base in &candidates {
            if base.join(rest).exists() {
                best = Some(base.clone());
                break;
            }
        }
        if let Some(dir) = best {
            map.insert(pkg, dir);
        } else {
            map.insert(pkg, candidates[0].clone());
            warn!(
                "urdf-viewer: package `{pkg}` — couldn't confirm root; guessing {:?}",
                candidates[0]
            );
        }
    }
}

// ── Activity bar ported from gearbox-editor/src/float.rs ─────────────
// VS-Code-style left-edge button column. Clicking toggles a panel
// anchored next to the rail.

const SIDE_BTN_SIZE: f32 = 34.0;
const SIDE_BTN_GAP: f32 = 4.0;
const EDGE_GAP: f32 = 8.0;
const RAIL_PANEL_GAP: f32 = 6.0;
const ACCENT: egui::Color32 = egui::Color32::from_rgb(0x6D, 0xAE, 0xF5);
const BG_1_PANEL: egui::Color32 = egui::Color32::from_rgba_premultiplied(24, 26, 32, 235);
const BG_2_RAISED: egui::Color32 = egui::Color32::from_rgb(0x2D, 0x2D, 0x32);
const BORDER_SUBTLE: egui::Color32 = egui::Color32::from_rgb(0x0E, 0x0E, 0x10);
const TEXT_PRIMARY: egui::Color32 = egui::Color32::from_rgb(0xE6, 0xE6, 0xE8);
const TEXT_SECONDARY: egui::Color32 = egui::Color32::from_rgb(0x9A, 0x9A, 0xA2);

#[derive(Resource, Clone, Copy, PartialEq, Eq)]
enum LeftTab {
    None,
    Selection,
    Overlays,
    Joints,
    Controls,
}

/// Stand-alone square button anchored to a screen edge, one per
/// `slot`. Active = accent stroke + tinted fill (VS Code convention).
fn side_button(
    id: &'static str,
    ctx: &egui::Context,
    slot: u32,
    glyph: &str,
    tooltip: &str,
    is_active: bool,
    on_click: impl FnOnce(),
) {
    let slot_y = slot as f32 * (SIDE_BTN_SIZE + SIDE_BTN_GAP);
    egui::Area::new(egui::Id::new(id))
        .anchor(egui::Align2::LEFT_TOP, egui::vec2(EDGE_GAP, EDGE_GAP + slot_y))
        .interactable(true)
        .show(ctx, |ui| {
            let (rect, resp) = ui.allocate_exact_size(
                egui::vec2(SIDE_BTN_SIZE, SIDE_BTN_SIZE),
                egui::Sense::click(),
            );
            let bg = if is_active {
                // Tint BG_2_RAISED 25 % toward accent.
                let mix = |a: u8, b: u8| ((a as f32) * 0.75 + (b as f32) * 0.25).round() as u8;
                egui::Color32::from_rgb(
                    mix(BG_2_RAISED.r(), ACCENT.r()),
                    mix(BG_2_RAISED.g(), ACCENT.g()),
                    mix(BG_2_RAISED.b(), ACCENT.b()),
                )
            } else if resp.hovered() {
                BG_2_RAISED
            } else {
                BG_1_PANEL
            };
            let fg = if is_active { TEXT_PRIMARY } else { TEXT_SECONDARY };
            let stroke = if is_active { ACCENT } else { BORDER_SUBTLE };
            let p = ui.painter();
            p.rect_filled(rect, egui::CornerRadius::same(6), bg);
            p.rect_stroke(
                rect,
                egui::CornerRadius::same(6),
                egui::Stroke::new(1.0, stroke),
                egui::StrokeKind::Outside,
            );
            p.text(
                rect.center(),
                egui::Align2::CENTER_CENTER,
                glyph,
                egui::FontId::new(14.0, egui::FontFamily::Monospace),
                fg,
            );
            if resp.on_hover_text(tooltip).clicked() {
                on_click();
            }
        });
}

/// Fixed-size floating content panel anchored to the left edge,
/// sitting to the right of the button rail. No title bar — we paint
/// our own uppercase caption inside the frame.
fn floating_panel(
    ctx: &egui::Context,
    id: &'static str,
    title: &str,
    width: f32,
    height: f32,
    add_contents: impl FnOnce(&mut egui::Ui),
) {
    let side_inset = EDGE_GAP + SIDE_BTN_SIZE + RAIL_PANEL_GAP;
    let frame = egui::Frame {
        inner_margin: egui::Margin { left: 10, right: 10, top: 8, bottom: 10 },
        outer_margin: egui::Margin::ZERO,
        fill: BG_1_PANEL,
        stroke: egui::Stroke::new(1.0, BORDER_SUBTLE),
        corner_radius: egui::CornerRadius::same(8),
        shadow: egui::epaint::Shadow {
            offset: [0, 6],
            blur: 20,
            spread: 0,
            color: egui::Color32::from_black_alpha(120),
        },
    };
    egui::Window::new(title)
        .id(egui::Id::new(id))
        .title_bar(false)
        .resizable(false)
        .collapsible(false)
        .anchor(egui::Align2::LEFT_TOP, egui::vec2(side_inset, EDGE_GAP))
        .fixed_size([width, height])
        .frame(frame)
        .show(ctx, |ui| {
            ui.set_max_width(width - 20.0);
            // Uppercase caption + 1 px underline.
            let title_h = 22.0;
            let (rect, _) = ui.allocate_exact_size(
                egui::vec2(ui.available_width(), title_h),
                egui::Sense::hover(),
            );
            ui.painter().text(
                egui::pos2(rect.min.x, rect.center().y),
                egui::Align2::LEFT_CENTER,
                title.to_uppercase(),
                egui::FontId::new(15.0, egui::FontFamily::Proportional),
                ACCENT,
            );
            ui.painter().hline(
                rect.min.x..=rect.max.x,
                rect.max.y + 3.0,
                egui::Stroke::new(1.0, BORDER_SUBTLE),
            );
            ui.add_space(14.0);
            add_contents(ui);
        });
}

/// Tiny enum so the label logic can treat `PendingLoad` and
/// `AppliedSelection` uniformly.
enum PendingOrApplied {
    Catalog(String),
    Custom(PathBuf),
}

impl PendingOrApplied {
    fn pending(p: &PendingLoad) -> Self {
        match p {
            PendingLoad::Catalog(k) => Self::Catalog(k.clone()),
            PendingLoad::Custom(p) => Self::Custom(p.clone()),
        }
    }
    fn applied(a: &AppliedSelection) -> Self {
        match a {
            AppliedSelection::Catalog(k) => Self::Catalog(k.clone()),
            AppliedSelection::Custom(p) => Self::Custom(p.clone()),
        }
    }
}

fn draw_panel(
    mut contexts: EguiContexts,
    mut active: ResMut<LeftTab>,
    mut selection: ResMut<Selection>,
    mut ui_state: ResMut<UiState>,
    mut toggles: ResMut<bevy_urdf::keyboard::DisplayToggles>,
    robots: Query<&Robot, With<RobotRoot>>,
    arms_q: Query<&RobotArms, With<RobotRoot>>,
) {
    let Ok(ctx) = contexts.ctx_mut() else {
        return;
    };

    // --- Side buttons --------------------------------------------------
    let toggle = |active: &mut LeftTab, tab: LeftTab| {
        *active = if *active == tab { LeftTab::None } else { tab };
    };
    side_button(
        "side_selection",
        ctx,
        0,
        "S",
        "Selection",
        *active == LeftTab::Selection,
        || toggle(&mut active, LeftTab::Selection),
    );
    side_button(
        "side_overlays",
        ctx,
        1,
        "O",
        "Overlays",
        *active == LeftTab::Overlays,
        || toggle(&mut active, LeftTab::Overlays),
    );
    side_button(
        "side_joints",
        ctx,
        2,
        "J",
        "Joints",
        *active == LeftTab::Joints,
        || toggle(&mut active, LeftTab::Joints),
    );
    side_button(
        "side_controls",
        ctx,
        3,
        "?",
        "Controls",
        *active == LeftTab::Controls,
        || toggle(&mut active, LeftTab::Controls),
    );

    if *active == LeftTab::None {
        return;
    }

    // While a load is pending, show what's coming in; after it lands,
    // show the settled `current` selection so the dropdown label doesn't
    // flip to "?" the frame after `pending.take()`.
    let current_label: String = match selection
        .pending
        .as_ref()
        .map(PendingOrApplied::pending)
        .or_else(|| selection.current.as_ref().map(PendingOrApplied::applied))
    {
        Some(PendingOrApplied::Catalog(k)) => {
            find_entry(&k).map(|e| e.label).unwrap_or("?").to_string()
        }
        Some(PendingOrApplied::Custom(p)) => p
            .file_name()
            .and_then(|n| n.to_str())
            .unwrap_or("(custom)")
            .to_string(),
        None => "?".to_string(),
    };

    const PANEL_W: f32 = 320.0;
    let panel_h: f32 = 560.0;

    match *active {
        LeftTab::Selection => floating_panel(ctx, "panel_selection", "Selection", PANEL_W, panel_h, |ui| {
            ui.label("Robot");
            let remaining = ui.available_width();
            egui::ComboBox::from_id_salt("robot_picker")
                .selected_text(&current_label)
                .width(remaining)
                .show_ui(ui, |ui| {
                    egui::ScrollArea::vertical().max_height(360.0).show(ui, |ui| {
                        if let Some(label) = &selection.custom_label {
                            if ui.selectable_label(false, format!("(custom) {label}")).clicked() {
                                if let Some(p) = selection.custom_path.clone() {
                                    selection.pending = Some(PendingLoad::Custom(p));
                                }
                            }
                            ui.separator();
                        }
                        for cat in [
                            Category::Arm,
                            Category::Mobile,
                            Category::Humanoid,
                            Category::Quadruped,
                            Category::Gripper,
                        ] {
                            ui.label(
                                egui::RichText::new(cat.label())
                                    .strong()
                                    .color(egui::Color32::from_gray(180)),
                            );
                            for entry in catalog().iter().filter(|e| e.category == cat) {
                                if ui.selectable_label(false, entry.label).clicked() {
                                    selection.pending =
                                        Some(PendingLoad::Catalog(entry.key.to_string()));
                                }
                            }
                            ui.add_space(4.0);
                        }
                    });
                });

            ui.add_space(8.0);

            let remaining = ui.available_width();
            if ui
                .add_sized([remaining, 24.0], egui::Button::new("📁  Browse URDF…"))
                .clicked()
            {
                if let Some(path) = rfd::FileDialog::new()
                    .add_filter("URDF", &["urdf", "xml"])
                    .pick_file()
                {
                    selection.pending = Some(PendingLoad::Custom(path));
                }
            }

            if let Some(p) = &selection.custom_path {
                ui.add_space(4.0);
                ui.label(
                    egui::RichText::new(p.display().to_string())
                        .small()
                        .color(egui::Color32::from_gray(140)),
                );
            }
        }),

        LeftTab::Overlays => floating_panel(ctx, "panel_overlays", "Overlays", PANEL_W, panel_h, |ui| {
            ui.checkbox(&mut toggles.show_visual, "Visual meshes");
            ui.checkbox(&mut toggles.show_collision, "Collision meshes");
            ui.checkbox(&mut toggles.show_world_grid, "Ground grid");
            ui.checkbox(&mut toggles.show_world_axes, "World axes");
            ui.checkbox(&mut toggles.show_joint_frames, "Joint frames");
            ui.checkbox(&mut toggles.show_link_frames, "Link frames");
            ui.checkbox(&mut toggles.show_link_names, "Link names");
            ui.add_space(6.0);
            ui.label("Body opacity");
            ui.add(
                egui::Slider::new(&mut toggles.body_opacity, 0.0..=1.0)
                    .show_value(true)
                    .fixed_decimals(2),
            );
        }),

        LeftTab::Joints => floating_panel(ctx, "panel_joints", "Joints", PANEL_W, panel_h, |ui| {
            if let Ok(robot) = robots.single() {
                let arms = arms_q.single().ok();
                draw_joint_controls(ui, robot, arms, &mut ui_state);
            } else {
                ui.label("(loading…)");
            }
        }),

        LeftTab::Controls => floating_panel(ctx, "panel_controls", "Controls", PANEL_W, panel_h, |ui| {
            ui.label(
                egui::RichText::new(
                    "L+R drag    orbit\n\
                     Middle drag pan\n\
                     Scroll      zoom\n\
                     \n\
                     Shift+L     IK target drag\n\
                     Ctrl+L      joint drag\n\
                     \n\
                     o / p       cycle joint\n\
                     , / .       cycle arm\n\
                     arrows      nudge active joint\n\
                     r           randomize\n\
                     z           reset to zero\n\
                     v           visual ↔ collision\n\
                     f / l / n   joint / link frames · names",
                )
                .monospace()
                .small()
                .color(egui::Color32::from_gray(195)),
            );
        }),

        LeftTab::None => {}
    }
}
