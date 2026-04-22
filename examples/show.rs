//! Interactive URDF browser. Pass an initial robot name as the first CLI
//! arg (default: `panda`). A single floating panel lets you swap robots,
//! toggle overlays, and drive every joint.
//!
//!   cargo run --example show
//!   cargo run --example show -- ur5
//!
//! Mouse: L+R drag orbit · middle drag pan · scroll zoom
//! Shift+L-drag pulls the IK target · Ctrl+L-drag nudges active joint.

use std::path::PathBuf;

use bevy::prelude::*;
use bevy_egui::{egui, EguiContexts, EguiPrimaryContextPass};
use big_space::prelude::*;
use bevy_urdf::{
    draw_joint_controls, ArcballCamera, LoadRobot, PackageMap, Robot, RobotArms, RobotRoot,
    UiState, UrdfPlugin,
};

#[derive(Clone)]
struct RobotEntry {
    key: &'static str,
    label: &'static str,
    urdf: &'static str,
    packages: &'static [(&'static str, &'static str)],
    focus: Vec3,
    distance: f32,
}

fn catalog() -> &'static [RobotEntry] {
    use std::sync::OnceLock;
    static CATALOG: OnceLock<Vec<RobotEntry>> = OnceLock::new();
    CATALOG.get_or_init(|| vec![
        RobotEntry {
            key: "panda",
            label: "Franka Panda",
            urdf: "urdf/oems/xacro_generated/franka_emika/franka_description/robots/panda/panda.urdf",
            packages: &[("franka_description", "urdf/oems/xacro_generated/franka_emika/franka_description")],
            focus: Vec3::new(0.0, 0.4, 0.0),
            distance: 2.5,
        },
        RobotEntry {
            key: "ur3",
            label: "Universal UR3",
            urdf: "urdf/matlab/ur_description/urdf/universalUR3.urdf",
            packages: &[("ur_description", "urdf/matlab/ur_description")],
            focus: Vec3::new(0.0, 0.3, 0.0),
            distance: 1.4,
        },
        RobotEntry {
            key: "ur5",
            label: "Universal UR5",
            urdf: "urdf/matlab/ur_description/urdf/universalUR5.urdf",
            packages: &[("ur_description", "urdf/matlab/ur_description")],
            focus: Vec3::new(0.0, 0.5, 0.0),
            distance: 1.8,
        },
        RobotEntry {
            key: "ur10",
            label: "Universal UR10",
            urdf: "urdf/matlab/ur_description/urdf/universalUR10.urdf",
            packages: &[("ur_description", "urdf/matlab/ur_description")],
            focus: Vec3::new(0.0, 0.7, 0.0),
            distance: 2.5,
        },
        RobotEntry {
            key: "iiwa",
            label: "KUKA iiwa14 (Drake)",
            urdf: "urdf/drake/iiwa_description/urdf/iiwa14_primitive_collision.urdf",
            packages: &[("drake", "urdf/drake/iiwa_description/manipulation_compat")],
            focus: Vec3::new(0.0, 0.6, 0.0),
            distance: 2.0,
        },
        RobotEntry {
            key: "iiwa_matlab",
            label: "KUKA iiwa14 (Matlab)",
            urdf: "urdf/matlab/iiwa_description/urdf/kukaIiwa14.urdf",
            packages: &[("iiwa_description", "urdf/matlab/iiwa_description")],
            focus: Vec3::new(0.0, 0.6, 0.0),
            distance: 2.0,
        },
        RobotEntry {
            key: "iiwa7",
            label: "KUKA iiwa7",
            urdf: "urdf/matlab/iiwa_description/urdf/kukaIiwa7.urdf",
            packages: &[("iiwa_description", "urdf/matlab/iiwa_description")],
            focus: Vec3::new(0.0, 0.5, 0.0),
            distance: 1.8,
        },
        RobotEntry {
            key: "abb",
            label: "ABB IRB 120",
            urdf: "urdf/matlab/abb_irb120_support/urdf/abbIrb120.urdf",
            packages: &[("abb_irb120_support", "urdf/matlab/abb_irb120_support")],
            focus: Vec3::new(0.0, 0.3, 0.0),
            distance: 1.4,
        },
        RobotEntry {
            key: "kinova",
            label: "Kinova Gen3",
            urdf: "urdf/matlab/kortex_description/urdf/kinovaGen3.urdf",
            packages: &[("JACO3_URDF_V10", "urdf/matlab/kortex_description")],
            focus: Vec3::new(0.0, 0.4, 0.0),
            distance: 1.6,
        },
        RobotEntry {
            key: "qarm",
            label: "Quanser QArm",
            urdf: "urdf/matlab/qarm_description/urdf/quanserQArm.urdf",
            packages: &[("QARM", "urdf/matlab/qarm_description")],
            focus: Vec3::new(0.0, 0.2, 0.0),
            distance: 0.8,
        },
        RobotEntry {
            key: "sawyer",
            label: "Rethink Sawyer",
            urdf: "urdf/matlab/sawyer_description/urdf/rethinkSawyer.urdf",
            packages: &[("sawyer_description", "urdf/matlab/sawyer_description")],
            focus: Vec3::new(0.0, 0.7, 0.0),
            distance: 2.5,
        },
        RobotEntry {
            key: "openmanip",
            label: "Robotis OpenManipulator",
            urdf: "urdf/matlab/open_manipulator_description/urdf/robotisOpenManipulator.urdf",
            packages: &[("open_manipulator_description", "urdf/matlab/open_manipulator_description")],
            focus: Vec3::new(0.0, 0.1, 0.0),
            distance: 0.6,
        },
        RobotEntry {
            key: "meca",
            label: "Mecademic Meca500",
            urdf: "urdf/matlab/mecademic_description/urdf/meca500r3.urdf",
            packages: &[("mecademic_description", "urdf/matlab/mecademic_description")],
            focus: Vec3::new(0.0, 0.2, 0.0),
            distance: 0.8,
        },
        RobotEntry {
            key: "fanuc",
            label: "Fanuc M16iB",
            urdf: "urdf/matlab/fanuc_m16ib_support/urdf/fanucM16ib.urdf",
            packages: &[("fanuc_m16ib_support", "urdf/matlab/fanuc_m16ib_support")],
            focus: Vec3::new(0.0, 0.6, 0.0),
            distance: 2.2,
        },
        RobotEntry {
            key: "anymal",
            label: "ANYmal B (best-effort)",
            urdf: "urdf/oems/anymal_anybotics/anymal_b_simple_description/urdf/anymal.urdf",
            packages: &[("meshes", "urdf/oems/anymal_anybotics/anymal_b_simple_description/meshes")],
            focus: Vec3::new(0.0, 0.3, 0.0),
            distance: 2.0,
        },
    ])
}

fn find_entry(key: &str) -> &'static RobotEntry {
    catalog().iter().find(|e| e.key == key).unwrap_or(&catalog()[0])
}

fn main() {
    let initial_key = std::env::args().nth(1).unwrap_or_else(|| "panda".into());

    App::new()
        .add_plugins(DefaultPlugins.build().disable::<TransformPlugin>())
        .add_plugins(BigSpaceDefaultPlugins)
        .add_plugins(UrdfPlugin)
        .insert_resource(SelectedRobot {
            key: find_entry(&initial_key).key.to_string(),
            applied: false,
        })
        .add_systems(Startup, setup)
        .add_systems(Update, swap_robot_if_changed)
        .add_systems(EguiPrimaryContextPass, (setup_egui_style, draw_control_panel).chain())
        .run();
}

#[derive(Resource)]
struct SelectedRobot {
    key: String,
    applied: bool,
}

#[derive(Resource, Default)]
struct CurrentRobot {
    root: Option<Entity>,
}

#[derive(Resource)]
struct WorldGrid(Entity);

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

    let mut grid_entity = None;
    commands.spawn_big_space_default(|grid| {
        grid.spawn_spatial((
            Camera3d::default(),
            Transform::from_xyz(3.0, 2.0, 3.0).looking_at(Vec3::new(0.0, 0.4, 0.0), Vec3::Y),
            ArcballCamera {
                focus: Vec3::new(0.0, 0.4, 0.0),
                distance: 3.0,
                ..default()
            },
            FloatingOrigin,
        ));
        grid_entity = Some(grid.id());
    });

    if let Some(e) = grid_entity {
        commands.insert_resource(WorldGrid(e));
    }
}

fn setup_egui_style(mut contexts: EguiContexts, mut done: Local<bool>) {
    if *done {
        return;
    }
    let Ok(ctx) = contexts.ctx_mut() else {
        return;
    };
    // Dark, slightly-rounded theme — Blender-ish panel colors.
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
    visuals.widgets.active.corner_radius = 4.0.into();
    visuals.widgets.hovered.corner_radius = 4.0.into();
    visuals.widgets.inactive.corner_radius = 4.0.into();
    visuals.widgets.open.corner_radius = 4.0.into();
    visuals.widgets.noninteractive.corner_radius = 4.0.into();
    visuals.selection.bg_fill = egui::Color32::from_rgb(70, 125, 200);
    ctx.set_visuals(visuals);

    let mut style = (*ctx.style()).clone();
    style.spacing.item_spacing = egui::vec2(8.0, 6.0);
    style.spacing.slider_width = 160.0;
    style.spacing.window_margin = egui::Margin::same(10);
    ctx.set_style(style);

    *done = true;
}

fn swap_robot_if_changed(
    mut commands: Commands,
    mut selected: ResMut<SelectedRobot>,
    mut current: ResMut<CurrentRobot>,
    mut package_map: ResMut<PackageMap>,
    world_grid: Option<Res<WorldGrid>>,
    mut cameras: Query<&mut ArcballCamera>,
) {
    if selected.applied {
        return;
    }
    let Some(world_grid) = world_grid else {
        return;
    };
    let entry = find_entry(&selected.key);
    let manifest_dir = PathBuf::from(env!("CARGO_MANIFEST_DIR"));

    for (pkg, dir) in entry.packages {
        package_map.insert(*pkg, manifest_dir.join(dir));
    }
    for (pkg, dir) in entry.packages {
        if *pkg == "drake" {
            let compat = manifest_dir.join(dir);
            let real_iiwa = compat.parent().unwrap().to_path_buf();
            let _ = std::fs::create_dir_all(compat.join("manipulation/models"));
            let link = compat.join("manipulation/models/iiwa_description");
            if !link.exists() {
                let _ = std::os::unix::fs::symlink(&real_iiwa, &link);
            }
        }
    }

    if let Some(prev) = current.root.take() {
        commands.entity(prev).despawn();
    }
    if let Ok(mut cam) = cameras.single_mut() {
        cam.focus = entry.focus;
        cam.distance = entry.distance;
    }

    let robot_root = commands
        .spawn((BigSpatialBundle::default(), ChildOf(world_grid.0)))
        .id();
    commands.write_message(LoadRobot {
        path: manifest_dir.join(entry.urdf),
        root: Some(robot_root),
    });
    current.root = Some(robot_root);
    selected.applied = true;
    info!("show: loading {}", entry.label);
}

fn draw_control_panel(
    mut contexts: EguiContexts,
    mut selected: ResMut<SelectedRobot>,
    mut ui_state: ResMut<UiState>,
    mut toggles: ResMut<bevy_urdf::keyboard::DisplayToggles>,
    robots: Query<&Robot, With<RobotRoot>>,
    arms_q: Query<&RobotArms, With<RobotRoot>>,
) {
    let Ok(ctx) = contexts.ctx_mut() else {
        return;
    };

    egui::Window::new("bevy_urdf")
        .default_width(300.0)
        .default_pos([14.0, 14.0])
        .resizable(true)
        .collapsible(true)
        .show(ctx, |ui| {
            // ── Robot picker ─────────────────────────────────────────
            ui.horizontal(|ui| {
                ui.label("Robot");
                let current_label = find_entry(&selected.key).label;
                let mut picked = selected.key.clone();
                egui::ComboBox::from_id_salt("show_robot_picker")
                    .selected_text(current_label)
                    .width(180.0)
                    .show_ui(ui, |ui| {
                        for entry in catalog() {
                            ui.selectable_value(
                                &mut picked,
                                entry.key.to_string(),
                                entry.label,
                            );
                        }
                    });
                if picked != selected.key {
                    selected.key = picked;
                    selected.applied = false;
                }
            });

            ui.separator();

            // ── Overlays ────────────────────────────────────────────
            egui::CollapsingHeader::new("Overlays")
                .default_open(false)
                .show(ui, |ui| {
                    ui.checkbox(&mut toggles.show_visual, "Visual meshes");
                    ui.checkbox(&mut toggles.show_collision, "Collision meshes");
                    ui.checkbox(&mut toggles.show_world_grid, "Ground grid");
                    ui.checkbox(&mut toggles.show_world_axes, "World axes");
                    ui.checkbox(&mut toggles.show_joint_frames, "Joint frames");
                    ui.checkbox(&mut toggles.show_link_frames, "Link frames");
                    ui.checkbox(&mut toggles.show_link_names, "Link names");
                });

            ui.separator();

            // ── Joints + IK ─────────────────────────────────────────
            if let Ok(robot) = robots.single() {
                let arms = arms_q.single().ok();
                draw_joint_controls(ui, robot, arms, &mut ui_state);
            } else {
                ui.label("(loading…)");
            }

            ui.separator();
            ui.label(
                egui::RichText::new(
                    "L+R drag orbit · Middle drag pan · Scroll zoom\n\
                     Shift+L drag → IK · Ctrl+L drag → joint",
                )
                .small()
                .color(egui::Color32::from_gray(160)),
            );
        });
}
