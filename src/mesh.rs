//! URDF geometry → `bevy::render::mesh::Mesh`.
//!
//! Covers the four primitives (Box / Cylinder / Capsule / Sphere) plus
//! STL / OBJ / DAE file meshes via `mesh-loader` (same loader that
//! `urdf-viz` uses).

use std::path::Path;

use bevy::asset::RenderAssetUsages;
use bevy::math::primitives;
use bevy::prelude::*;
use bevy::mesh::{Indices, Mesh, PrimitiveTopology};
use mesh_loader::Loader;

use crate::urdf::UrdfError;

pub struct MeshPlugin;

impl Plugin for MeshPlugin {
    fn build(&self, _app: &mut App) {
        // Mesh conversion is synchronous; nothing to register yet.
    }
}

/// Convert a URDF geometry block into one or more Bevy meshes.
///
/// Returns a list of `(handle, scale)` pairs. Primitives always produce a
/// single entry; mesh files can produce multiple entries when the source
/// DAE / OBJ has several sub-meshes (one per material) — we spawn one
/// entity per sub-mesh so nothing gets lost to an over-eager merge (this
/// was the cause of the "chopped" UR5 rendering).
pub fn bevy_meshes_from_urdf_geometry(
    geometry: &urdf_rs::Geometry,
    resolve: impl Fn(&str) -> Result<std::path::PathBuf, UrdfError>,
    meshes: &mut Assets<Mesh>,
) -> Result<Vec<(Handle<Mesh>, Vec3)>, UrdfError> {
    match geometry {
        urdf_rs::Geometry::Box { size } => {
            let [x, y, z] = [size[0] as f32, size[1] as f32, size[2] as f32];
            let mesh = Mesh::from(primitives::Cuboid::new(x, y, z));
            Ok(vec![(meshes.add(mesh), Vec3::ONE)])
        }
        urdf_rs::Geometry::Cylinder { radius, length } => {
            // URDF cylinders have their axis along +Z; Bevy's Cylinder
            // primitive has its axis along +Y. Caller must rotate.
            let mesh = Mesh::from(primitives::Cylinder::new(*radius as f32, *length as f32));
            Ok(vec![(meshes.add(mesh), Vec3::ONE)])
        }
        urdf_rs::Geometry::Capsule { radius, length } => {
            // URDF capsule: total length along Z. Bevy's Capsule3d takes
            // cylinder half_length and is Y-up — same axis quirk as above.
            let mesh = Mesh::from(primitives::Capsule3d::new(
                *radius as f32,
                *length as f32,
            ));
            Ok(vec![(meshes.add(mesh), Vec3::ONE)])
        }
        urdf_rs::Geometry::Sphere { radius } => {
            let mesh = Mesh::from(primitives::Sphere::new(*radius as f32));
            Ok(vec![(meshes.add(mesh), Vec3::ONE)])
        }
        urdf_rs::Geometry::Mesh { filename, scale } => {
            let path = resolve(filename)?;
            let scale_vec = scale
                .as_ref()
                .map(|s| Vec3::new(s[0] as f32, s[1] as f32, s[2] as f32))
                .unwrap_or(Vec3::ONE);
            let sub_meshes = load_mesh_file_all(&path)?;
            Ok(sub_meshes
                .into_iter()
                .map(|m| (meshes.add(m), scale_vec))
                .collect())
        }
    }
}

/// Load every sub-mesh in a file as a separate Bevy `Mesh`. No merging —
/// multi-material DAEs ship as several meshes in one file (one per
/// material); merging them loses per-group attributes and (in practice)
/// dropped geometry on the UR5 and Panda.
fn load_mesh_file_all(path: &Path) -> Result<Vec<Mesh>, UrdfError> {
    let scenes = load_raw_scene(path)?;
    Ok(scenes.into_iter().map(mesh_loader_to_bevy).collect())
}

/// Raw-mesh variant for callers that need the vertex/index arrays (e.g.
/// rapier trimesh colliders). Returns all sub-meshes in load order.
pub fn load_raw_mesh(path: &Path) -> Result<mesh_loader::Mesh, UrdfError> {
    // Merge everything here — for collision, a single trimesh is all a
    // collider wants, and we don't care about material boundaries.
    let scene = Loader::default()
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

fn load_raw_scene(path: &Path) -> Result<Vec<mesh_loader::Mesh>, UrdfError> {
    let scene = Loader::default()
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
    Ok(scene.meshes)
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
