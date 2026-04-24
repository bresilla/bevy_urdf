//! URDF geometry → `bevy::render::mesh::Mesh`.
//!
//! Covers the four primitives (Box / Cylinder / Capsule / Sphere) plus
//! STL / OBJ / DAE file meshes via `mesh-loader` (same loader that
//! `urdf-viz` uses).

use std::path::Path;

use bevy::asset::RenderAssetUsages;
use bevy::math::primitives;
use bevy::prelude::*;
use bevy::mesh::{Indices, Mesh, PrimitiveTopology, VertexAttributeValues};
use mesh_loader::Loader;

use crate::urdf::UrdfError;

pub struct MeshPlugin;

impl Plugin for MeshPlugin {
    fn build(&self, _app: &mut App) {
        // Mesh conversion is synchronous; nothing to register yet.
    }
}

/// One Bevy-ready sub-mesh produced from a URDF geometry block.
pub struct SubMesh {
    pub handle: Handle<Mesh>,
    pub scale: Vec3,
    /// Diffuse color parsed from the source file's material (DAE / OBJ)
    /// when present. Used as the fallback when the URDF doesn't supply
    /// a color for the visual.
    pub material_color: Option<Color>,
    /// Absolute path to the diffuse-map texture referenced by the mesh
    /// file's material (PNG / JPG). `None` when the material only has
    /// a flat color or no texture info at all.
    pub diffuse_texture: Option<std::path::PathBuf>,
}

/// Convert a URDF geometry block into one or more Bevy meshes.
///
/// Primitives always produce a single entry; mesh files can produce
/// multiple entries when the source DAE / OBJ has several sub-meshes
/// (one per material) — we spawn one entity per sub-mesh so nothing
/// gets lost to an over-eager merge.
pub fn bevy_meshes_from_urdf_geometry(
    geometry: &urdf_rs::Geometry,
    resolve: impl Fn(&str) -> Result<std::path::PathBuf, UrdfError>,
    meshes: &mut Assets<Mesh>,
) -> Result<Vec<SubMesh>, UrdfError> {
    match geometry {
        urdf_rs::Geometry::Box { size } => {
            let [x, y, z] = [size[0] as f32, size[1] as f32, size[2] as f32];
            Ok(vec![SubMesh {
                handle: meshes.add(Mesh::from(primitives::Cuboid::new(x, y, z))),
                scale: Vec3::ONE,
                material_color: None,
                diffuse_texture: None,
            }])
        }
        urdf_rs::Geometry::Cylinder { radius, length } => Ok(vec![SubMesh {
            handle: meshes.add(Mesh::from(primitives::Cylinder::new(
                *radius as f32,
                *length as f32,
            ))),
            scale: Vec3::ONE,
            material_color: None,
            diffuse_texture: None,
        }]),
        urdf_rs::Geometry::Capsule { radius, length } => Ok(vec![SubMesh {
            handle: meshes.add(Mesh::from(primitives::Capsule3d::new(
                *radius as f32,
                *length as f32,
            ))),
            scale: Vec3::ONE,
            material_color: None,
            diffuse_texture: None,
        }]),
        urdf_rs::Geometry::Sphere { radius } => Ok(vec![SubMesh {
            handle: meshes.add(Mesh::from(primitives::Sphere::new(*radius as f32))),
            scale: Vec3::ONE,
            material_color: None,
            diffuse_texture: None,
        }]),
        urdf_rs::Geometry::Mesh { filename, scale } => {
            let path = resolve(filename)?;
            let scale_vec = scale
                .as_ref()
                .map(|s| Vec3::new(s[0] as f32, s[1] as f32, s[2] as f32))
                .unwrap_or(Vec3::ONE);
            // Bake non-uniform scale into the vertices so normals stay
            // correct. Transform.scale would apply the same factor to
            // positions and normals, which distorts lighting on wheels
            // and other squished/stretched meshes (tread grooves looked
            // smeared on Robotti's tires).
            let bake = (scale_vec - Vec3::ONE).abs().max_element() > 1e-6;
            Ok(load_mesh_file_all(&path)?
                .into_iter()
                .map(|(mut mesh, mat_color, tex)| {
                    if bake {
                        apply_scale_to_mesh(&mut mesh, scale_vec);
                    }
                    SubMesh {
                        handle: meshes.add(mesh),
                        scale: if bake { Vec3::ONE } else { scale_vec },
                        material_color: mat_color,
                        diffuse_texture: tex,
                    }
                })
                .collect())
        }
    }
}

/// Load every sub-mesh in a file as a separate Bevy `Mesh`, paired with
/// the diffuse colour AND the diffuse-texture path parsed from the
/// corresponding material. Collada / Wavefront assign materials by
/// position in `scene.materials`, so we `zip` the two in the same
/// order `rapier3d-meshloader` does.
fn load_mesh_file_all(
    path: &Path,
) -> Result<Vec<(Mesh, Option<Color>, Option<std::path::PathBuf>)>, UrdfError> {
    let (meshes, materials) = load_raw_scene(path)?;
    let mesh_dir = path.parent().map(|p| p.to_path_buf());
    let mut out = Vec::with_capacity(meshes.len());
    for (i, m) in meshes.into_iter().enumerate() {
        let mat_ref = materials.get(i);
        let color = mat_ref
            .and_then(|mat| mat.color.diffuse)
            .map(color4_to_bevy);
        // Resolve texture paths relative to the mesh file so they work
        // regardless of where the process was launched from.
        let tex = mat_ref.and_then(|mat| mat.texture.diffuse.as_ref()).map(|p| {
            if p.is_absolute() {
                p.clone()
            } else {
                mesh_dir.as_ref().map(|d| d.join(p)).unwrap_or_else(|| p.clone())
            }
        });
        out.push((mesh_loader_to_bevy(m), color, tex));
    }
    Ok(out)
}

fn color4_to_bevy(c: [f32; 4]) -> Color {
    Color::srgba(c[0], c[1], c[2], c[3])
}

/// Multiply positions by `scale` and compensate normals with the
/// inverse-transpose factor (per-component reciprocal for a diagonal
/// scale). Re-normalises after so the result is unit-length.
fn apply_scale_to_mesh(mesh: &mut Mesh, scale: Vec3) {
    if let Some(VertexAttributeValues::Float32x3(positions)) =
        mesh.attribute_mut(Mesh::ATTRIBUTE_POSITION)
    {
        for p in positions.iter_mut() {
            p[0] *= scale.x;
            p[1] *= scale.y;
            p[2] *= scale.z;
        }
    }
    let inv = Vec3::new(1.0 / scale.x, 1.0 / scale.y, 1.0 / scale.z);
    if let Some(VertexAttributeValues::Float32x3(normals)) =
        mesh.attribute_mut(Mesh::ATTRIBUTE_NORMAL)
    {
        for n in normals.iter_mut() {
            let v = Vec3::new(n[0] * inv.x, n[1] * inv.y, n[2] * inv.z).normalize_or_zero();
            *n = [v.x, v.y, v.z];
        }
    }
}

/// Raw-mesh variant for callers that need the vertex/index arrays (e.g.
/// rapier trimesh colliders). Returns all sub-meshes in load order.
pub fn load_raw_mesh(path: &Path) -> Result<mesh_loader::Mesh, UrdfError> {
    // Merge everything here — for collision, a single trimesh is all a
    // collider wants, and we don't care about material boundaries.
    let scene = Loader::default()
        .stl_parse_color(true)
        .merge_meshes(true)
        .load(path)
        .map_err(|e| UrdfError::Mesh {
            path: path.to_path_buf(),
            message: e.to_string(),
        })?;
    scene.meshes.into_iter().next().ok_or_else(|| UrdfError::Mesh {
        path: path.to_path_buf(),
        message: "file contained no meshes".to_string(),
    })
}

fn load_raw_scene(
    path: &Path,
) -> Result<(Vec<mesh_loader::Mesh>, Vec<mesh_loader::Material>), UrdfError> {
    let scene = Loader::default()
        .stl_parse_color(true)
        .load(path)
        .map_err(|e| UrdfError::Mesh {
            path: path.to_path_buf(),
            message: e.to_string(),
        })?;
    if scene.meshes.is_empty() {
        return Err(UrdfError::Mesh {
            path: path.to_path_buf(),
            message: "file contained no meshes".to_string(),
        });
    }

    // Note: we deliberately do NOT honour `<up_axis>` from Collada.
    // URDFs authored against a specific mesh set assume the raw vertex
    // coordinates match what the URDF expects; rotating here breaks any
    // URDF that was written against Y-up meshes (most of the ROS robot
    // zoo — movo, kinova, etc.). Gazebo/rviz behave the same way.
    Ok((scene.meshes, scene.materials))
}

fn mesh_loader_to_bevy(src: mesh_loader::Mesh) -> Mesh {
    let positions: Vec<[f32; 3]> = src.vertices.iter().map(|v| [v[0], v[1], v[2]]).collect();
    let indices: Vec<u32> = src.faces.iter().flat_map(|f| [f[0], f[1], f[2]]).collect();

    let mut mesh = Mesh::new(
        PrimitiveTopology::TriangleList,
        RenderAssetUsages::default(),
    );
    mesh.insert_attribute(Mesh::ATTRIBUTE_POSITION, positions);

    if !src.normals.is_empty() && src.normals.len() == src.vertices.len() {
        let normals: Vec<[f32; 3]> = src.normals.iter().map(|n| [n[0], n[1], n[2]]).collect();
        mesh.insert_attribute(Mesh::ATTRIBUTE_NORMAL, normals);
    } else {
        // No supplied normals — compute smooth ones from indexed faces.
        // `compute_flat_normals` would panic on indexed meshes; smooth
        // is the right call for PBR-friendly shading on .obj/.stl files
        // that ship without normals.
        mesh.insert_indices(Indices::U32(indices.clone()));
        mesh.compute_smooth_normals();
        return mesh;
    }

    if !src.texcoords[0].is_empty() && src.texcoords[0].len() == src.vertices.len() {
        let uvs: Vec<[f32; 2]> = src.texcoords[0].iter().map(|uv| [uv[0], uv[1]]).collect();
        mesh.insert_attribute(Mesh::ATTRIBUTE_UV_0, uvs);
    }

    mesh.insert_indices(Indices::U32(indices));
    mesh
}
