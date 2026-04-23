use std::collections::HashMap;
use std::path::PathBuf;

use bevy::prelude::*;

use crate::mesh::bevy_meshes_from_urdf_geometry;
use crate::urdf::{self, PackageMap, UrdfError};

/// Root entity of a spawned robot. Carries the `k::Chain`, the parsed URDF,
/// and lookup tables that let the FK system drive link entities from chain
/// nodes.
#[derive(Component)]
pub struct Robot {
    pub chain: k::Chain<f32>,
    pub urdf: urdf_rs::Robot,
    /// URDF file directory — needed to resolve relative mesh paths on reload.
    pub urdf_dir: PathBuf,
    /// joint-name → URDF child-link name. Built from `urdf.joints`; lets
    /// FK map `k::Node` (keyed by joint name) → link entity.
    pub joint_to_child_link: HashMap<String, String>,
    /// URDF-link-name → spawned entity. The root link maps to the robot
    /// root entity (which has identity transform — it IS the base).
    pub link_entities: HashMap<String, Entity>,
}

/// Marker: the transform root the FK system writes into. All link entities
/// are descendants of this entity. Separate from the `Robot` component so
/// tests / headless setups can spawn just the transform skeleton.
#[derive(Component, Default)]
pub struct RobotRoot;

/// One per URDF link. `link_index` indexes into `k::Chain::iter_links()`.
#[derive(Component, Debug, Clone)]
pub struct RobotLink {
    pub link_index: usize,
    pub name: String,
}

/// One per URDF joint. Attached to the *child link* entity of the joint
/// (matches k's convention: a link's joint is what attaches it to its parent).
#[derive(Component, Debug, Clone)]
pub struct RobotJoint {
    pub joint_index: usize,
    pub name: String,
    pub kind: JointKind,
    pub limits: Option<(f32, f32)>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum JointKind {
    Fixed,
    Revolute,
    Continuous,
    Prismatic,
    Floating,
    Planar,
}

impl JointKind {
    fn from_urdf(j: &urdf_rs::JointType) -> Self {
        match j {
            urdf_rs::JointType::Fixed => Self::Fixed,
            urdf_rs::JointType::Revolute => Self::Revolute,
            urdf_rs::JointType::Continuous => Self::Continuous,
            urdf_rs::JointType::Prismatic => Self::Prismatic,
            urdf_rs::JointType::Floating => Self::Floating,
            urdf_rs::JointType::Planar => Self::Planar,
            // urdf-rs may expose additional variants in future; default to
            // Fixed so the scene still loads.
            _ => Self::Fixed,
        }
    }
}

/// Distinguishes a visual-geometry child from a collision-geometry child of
/// a link. The `collision` entity is `Visibility::Hidden` by default; the
/// `v` key in milestone #10 will swap it.
#[derive(Component, Debug, Clone, Copy, PartialEq, Eq)]
pub enum GeomKind {
    Visual,
    Collision,
}

/// Snapshot of the material's RGB at spawn. Used by the opacity slider so
/// we can reapply `alpha = body_opacity` without losing the original
/// colour when the user slides back to 1.0.
#[derive(Component, Debug, Clone, Copy)]
pub struct GeomBaseColor(pub Color);

/// Command: load a URDF from disk and spawn it as a robot entity.
///
/// Emitted by the user; handled by `robot::handle_load_requests`. The
/// command owns a root entity (optional — caller may pre-create one to
/// position the robot via `Transform`).
///
/// Bevy 0.18 renamed `Event` → `Message`.
#[derive(Message)]
pub struct LoadRobot {
    pub path: PathBuf,
    /// If `Some`, the loader attaches components to this existing entity
    /// (useful when the caller has already set the robot's Transform).
    /// If `None`, a new entity is spawned at the world origin.
    pub root: Option<Entity>,
}

/// Marker for robots whose `Robot` component has been attached but whose
/// link/joint children haven't been spawned yet.
#[derive(Component, Default)]
pub struct RobotNeedsSpawn;

pub struct RobotPlugin;

impl Plugin for RobotPlugin {
    fn build(&self, app: &mut App) {
        app.add_message::<LoadRobot>()
            .init_resource::<PackageMap>()
            .add_systems(
                Update,
                (
                    handle_load_requests,
                    apply_urdf_axis_convention,
                    spawn_robot_children,
                )
                    .chain(),
            );
    }
}

/// URDF/ROS uses Z-up; Bevy is Y-up. Without this rotation every robot
/// lies flat on the ground. Applied exactly once, on the frame the
/// `Robot` component is added, pre-multiplying the root entity's
/// rotation so any translation the caller set survives.
fn apply_urdf_axis_convention(mut q: Query<&mut Transform, Added<Robot>>) {
    let fix = Quat::from_rotation_x(-std::f32::consts::FRAC_PI_2);
    for mut tf in q.iter_mut() {
        tf.rotation = fix * tf.rotation;
    }
}

fn handle_load_requests(
    mut commands: Commands,
    mut msgs: MessageReader<LoadRobot>,
) {
    for msg in msgs.read() {
        let loaded = match urdf::load_from_file(&msg.path) {
            Ok(l) => l,
            Err(e) => {
                error!("bevy_urdf: failed to load {:?}: {e}", msg.path);
                continue;
            }
        };
        // If the caller pre-spawned a root entity (e.g. to set its
        // Transform), we attach our components there. Otherwise we
        // spawn a bare top-level entity at the world origin.
        let entity = match msg.root {
            Some(e) => e,
            None => commands
                .spawn((
                    Transform::default(),
                    GlobalTransform::default(),
                    Visibility::default(),
                    InheritedVisibility::default(),
                    ViewVisibility::default(),
                ))
                .id(),
        };
        let joint_to_child_link: HashMap<String, String> = loaded
            .robot
            .joints
            .iter()
            .map(|j| (j.name.clone(), j.child.link.clone()))
            .collect();

        commands.entity(entity).insert((
            RobotRoot,
            Robot {
                chain: loaded.chain,
                urdf: loaded.robot,
                urdf_dir: loaded.urdf_dir,
                joint_to_child_link,
                link_entities: HashMap::new(),
            },
            RobotNeedsSpawn,
        ));
        info!("bevy_urdf: loaded URDF from {:?}", msg.path);
    }
}

/// Walk a freshly-loaded robot and spawn an entity per URDF link, each
/// with visual + collision geometry children.
fn spawn_robot_children(
    mut commands: Commands,
    package_map: Res<PackageMap>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut images: ResMut<Assets<bevy::image::Image>>,
    mut robots: Query<(Entity, &mut Robot), With<RobotNeedsSpawn>>,
) {
    for (root_entity, mut robot) in robots.iter_mut() {
        let urdf_dir = robot.urdf_dir.clone();
        let resolve = |uri: &str| package_map.resolve(uri, &urdf_dir);

        // Preflight: pre-resolve URDF material colors by name. URDFs
        // usually declare the colours at robot-root level
        // (<material name="red"><color .../></material>) and then visual
        // blocks carry name-only refs (<material name="red"/>). We also
        // scan every visual — some URDFs (turtlebot3, roboticsl arm
        // examples) inline the <color> on the first use of a name and
        // just reference it by name thereafter.
        let mut named_materials: std::collections::HashMap<String, Color> =
            std::collections::HashMap::new();
        for m in &robot.urdf.materials {
            if let Some(c) = &m.color {
                named_materials.insert(m.name.clone(), urdf_color_to_bevy(*c.rgba));
            }
        }
        for link in &robot.urdf.links {
            for v in &link.visual {
                if let Some(m) = &v.material {
                    if let Some(c) = &m.color {
                        named_materials
                            .entry(m.name.clone())
                            .or_insert_with(|| urdf_color_to_bevy(*c.rgba));
                    }
                }
            }
        }

        // The root link is any link that isn't referenced as a `child` of
        // some joint. That link rides directly on the robot root entity.
        let child_links: std::collections::HashSet<&str> = robot
            .urdf
            .joints
            .iter()
            .map(|j| j.child.link.as_str())
            .collect();

        let mut link_entities: HashMap<String, Entity> = HashMap::new();

        for (link_index, link) in robot.urdf.links.iter().enumerate() {
            let is_root = !child_links.contains(link.name.as_str());
            let link_entity = if is_root {
                // Mount root-link geometry under the robot root entity so
                // FK can leave the robot root at identity while still
                // rendering the base.
                commands
                    .entity(root_entity)
                    .insert(RobotLink {
                        link_index,
                        name: link.name.clone(),
                    });
                root_entity
            } else {
                commands
                    .spawn((
                        RobotLink {
                            link_index,
                            name: link.name.clone(),
                        },
                        Transform::default(),
                        GlobalTransform::default(),
                        Visibility::default(),
                        InheritedVisibility::default(),
                        ViewVisibility::default(),
                        ChildOf(root_entity),
                    ))
                    .id()
            };
            link_entities.insert(link.name.clone(), link_entity);

            for visual in &link.visual {
                let resolved_color = visual.material.as_ref().and_then(|m| {
                    m.color
                        .as_ref()
                        .map(|c| urdf_color_to_bevy(*c.rgba))
                        .or_else(|| named_materials.get(m.name.as_str()).copied())
                });
                spawn_geometry(
                    &mut commands,
                    link_entity,
                    GeomKind::Visual,
                    &visual.geometry,
                    &visual.origin,
                    resolved_color,
                    &mut meshes,
                    &mut materials,
                    &mut images,
                    &resolve,
                );
            }
            for collision in &link.collision {
                spawn_geometry(
                    &mut commands,
                    link_entity,
                    GeomKind::Collision,
                    &collision.geometry,
                    &collision.origin,
                    None,
                    &mut meshes,
                    &mut materials,
                    &mut images,
                    &resolve,
                );
            }
        }

        // Attach RobotJoint markers *by joint name* — we map to link
        // entities in a later system once we have the joint → child-link
        // relationship. For now, just record joints on the root for FK
        // use; per-link joint components are set when FK is wired.
        for (joint_index, joint) in robot.urdf.joints.iter().enumerate() {
            // We'll refine placement in milestone #5. Recording here keeps
            // the data close to the robot spawn and lets FK look it up.
            let limits = match joint.joint_type {
                urdf_rs::JointType::Revolute | urdf_rs::JointType::Prismatic => {
                    Some((joint.limit.lower as f32, joint.limit.upper as f32))
                }
                _ => None,
            };
            commands.spawn((
                RobotJoint {
                    joint_index,
                    name: joint.name.clone(),
                    kind: JointKind::from_urdf(&joint.joint_type),
                    limits,
                },
                ChildOf(root_entity),
            ));
        }

        robot.link_entities = link_entities;
        commands.entity(root_entity).remove::<RobotNeedsSpawn>();
    }
}

#[allow(clippy::too_many_arguments)]
fn spawn_geometry(
    commands: &mut Commands,
    parent: Entity,
    kind: GeomKind,
    geometry: &urdf_rs::Geometry,
    origin: &urdf_rs::Pose,
    color: Option<Color>,
    meshes: &mut Assets<Mesh>,
    materials: &mut Assets<StandardMaterial>,
    images: &mut Assets<bevy::image::Image>,
    resolve: &impl Fn(&str) -> Result<PathBuf, UrdfError>,
) {
    let sub_meshes = match bevy_meshes_from_urdf_geometry(geometry, resolve, meshes) {
        Ok(h) => h,
        Err(e) => {
            error!("bevy_urdf: skipping geometry for {kind:?}: {e}");
            return;
        }
    };
    let fallback_color = Color::srgb(0.7, 0.7, 0.75);
    let visibility = match kind {
        GeomKind::Visual => Visibility::default(),
        GeomKind::Collision => Visibility::Hidden,
    };

    // Priority: mesh-file material (per-sub-mesh diffuse from DAE / OBJ)
    // > URDF color > grey fallback. In practice URDF materials are
    // usually named references to a single flat colour (e.g. Anymal's
    // `anymal_material` = 0.7 grey), while the source DAE carries richer
    // per-sub-mesh liveries. Letting the mesh colour win matches what
    // users see in rviz / Gazebo.
    for sub in sub_meshes {
        let resolved = sub.material_color.or(color).unwrap_or(fallback_color);
        let texture_handle = sub
            .diffuse_texture
            .as_ref()
            .and_then(|p| load_texture(p, images));
        let mat = materials.add(StandardMaterial {
            // When a texture is present, keep base_color WHITE so the
            // texture reads at full fidelity. Without a texture, paint
            // the resolved flat colour.
            base_color: if texture_handle.is_some() {
                Color::WHITE
            } else {
                resolved
            },
            base_color_texture: texture_handle,
            ..default()
        });
        let transform = urdf_pose_to_transform(origin, sub.scale, geometry);
        commands.spawn((
            Mesh3d(sub.handle),
            MeshMaterial3d(mat),
            transform,
            GlobalTransform::default(),
            visibility,
            InheritedVisibility::default(),
            ViewVisibility::default(),
            kind,
            GeomBaseColor(resolved),
            ChildOf(parent),
        ));
    }
}

/// Read an image file from disk and register it as a Bevy `Image`
/// asset. Returns `None` on any failure so a busted texture path doesn't
/// fail the whole robot load (the material falls back to a flat colour).
fn load_texture(path: &std::path::Path, images: &mut Assets<bevy::image::Image>) -> Option<Handle<bevy::image::Image>> {
    let bytes = match std::fs::read(path) {
        Ok(b) => b,
        Err(e) => {
            warn!("bevy_urdf: texture read failed for {path:?}: {e}");
            return None;
        }
    };
    let ext = path
        .extension()
        .and_then(|s| s.to_str())
        .unwrap_or("")
        .to_ascii_lowercase();
    let image = bevy::image::Image::from_buffer(
        &bytes,
        bevy::image::ImageType::Extension(&ext),
        bevy::image::CompressedImageFormats::empty(),
        /* is_srgb */ true,
        bevy::image::ImageSampler::Default,
        bevy::asset::RenderAssetUsages::default(),
    );
    match image {
        Ok(img) => Some(images.add(img)),
        Err(e) => {
            warn!("bevy_urdf: texture decode failed for {path:?}: {e}");
            None
        }
    }
}

fn urdf_color_to_bevy(rgba: [f64; 4]) -> Color {
    Color::srgba(rgba[0] as f32, rgba[1] as f32, rgba[2] as f32, rgba[3] as f32)
}

/// Build a Bevy `Transform` from a URDF `<origin>` + the geometry's own axis
/// quirks.
///
/// URDF convention: cylinders and capsules are axis-aligned along **+Z**.
/// Bevy's `primitives::Cylinder` / `Capsule3d` are along **+Y**. We bake a
/// -90° rotation around X into the transform so the geometry is oriented
/// correctly within the link frame — avoids baking it into the mesh, which
/// would break mesh reuse.
fn urdf_pose_to_transform(
    origin: &urdf_rs::Pose,
    scale: Vec3,
    geometry: &urdf_rs::Geometry,
) -> Transform {
    let translation = Vec3::new(
        origin.xyz[0] as f32,
        origin.xyz[1] as f32,
        origin.xyz[2] as f32,
    );
    let rpy = origin.rpy;
    let euler = Quat::from_euler(
        EulerRot::XYZ,
        rpy[0] as f32,
        rpy[1] as f32,
        rpy[2] as f32,
    );

    // Bevy cylinder/capsule axis is +Y; URDF is +Z. Rotate the local frame
    // to align them.
    let axis_fix = match geometry {
        urdf_rs::Geometry::Cylinder { .. } | urdf_rs::Geometry::Capsule { .. } => {
            Quat::from_rotation_x(std::f32::consts::FRAC_PI_2)
        }
        _ => Quat::IDENTITY,
    };

    Transform {
        translation,
        rotation: euler * axis_fix,
        scale,
    }
}
