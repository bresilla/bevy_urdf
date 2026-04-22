use std::{
    collections::HashMap,
    path::{Path, PathBuf},
};

use bevy::prelude::*;
use thiserror::Error;

/// Maps `package://<name>/...` URIs to a filesystem root.
///
/// URDF meshes are almost always referenced via `package://robot_description/meshes/foo.stl`
/// (a ROS convention). The consuming app supplies the mapping; we never guess.
#[derive(Clone, Debug, Default, Resource)]
pub struct PackageMap(HashMap<String, PathBuf>);

impl PackageMap {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn with(mut self, package: impl Into<String>, root: impl Into<PathBuf>) -> Self {
        self.0.insert(package.into(), root.into());
        self
    }

    pub fn insert(&mut self, package: impl Into<String>, root: impl Into<PathBuf>) {
        self.0.insert(package.into(), root.into());
    }

    /// Resolve a URDF mesh URI (either `package://...` or a plain path) into a
    /// concrete absolute path. `urdf_dir` is the directory of the URDF file,
    /// used to resolve relative paths.
    pub fn resolve(&self, uri: &str, urdf_dir: &Path) -> Result<PathBuf, UrdfError> {
        if let Some(stripped) = uri.strip_prefix("package://") {
            let (pkg, rest) = stripped
                .split_once('/')
                .ok_or_else(|| UrdfError::BadPackageUri(uri.to_string()))?;
            let root = self
                .0
                .get(pkg)
                .ok_or_else(|| UrdfError::UnknownPackage(pkg.to_string()))?;
            Ok(root.join(rest))
        } else if let Some(stripped) = uri.strip_prefix("file://") {
            Ok(PathBuf::from(stripped))
        } else {
            let p = PathBuf::from(uri);
            if p.is_absolute() {
                Ok(p)
            } else {
                Ok(urdf_dir.join(p))
            }
        }
    }
}

#[derive(Debug, Error)]
pub enum UrdfError {
    #[error("failed to read URDF: {0}")]
    Io(#[from] std::io::Error),
    #[error("urdf-rs parse error: {0}")]
    Parse(#[from] urdf_rs::UrdfError),
    #[error("k::Chain build error: {0}")]
    Chain(String),
    #[error("package URI has no package name: {0}")]
    BadPackageUri(String),
    #[error("unknown URDF package: {0} (not registered in PackageMap)")]
    UnknownPackage(String),
    #[error("mesh load error ({path}): {message}")]
    Mesh { path: PathBuf, message: String },
}

/// Parsed URDF + its `k::Chain` + the directory of the source file (so we can
/// resolve relative mesh paths later).
pub struct LoadedUrdf {
    pub robot: urdf_rs::Robot,
    pub chain: k::Chain<f32>,
    pub urdf_dir: PathBuf,
}

/// Load a URDF from disk. Mesh files are not loaded here — they're resolved
/// later when we spawn link entities.
pub fn load_from_file(path: &Path) -> Result<LoadedUrdf, UrdfError> {
    let text = std::fs::read_to_string(path)?;
    let robot = urdf_rs::read_from_string(&text)?;
    let chain = k::Chain::<f32>::from(&robot);
    let urdf_dir = path
        .parent()
        .map(|p| p.to_path_buf())
        .unwrap_or_else(|| PathBuf::from("."));
    Ok(LoadedUrdf {
        robot,
        chain,
        urdf_dir,
    })
}
