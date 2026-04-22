//! Helpers for placing a robot root under a `big_space::BigSpace` grid.
//!
//! Using a floating-origin reference means link `Transform`s stay in `f32`
//! precision even when the robot lives far from world origin. This module
//! only provides thin helpers; the robot loader doesn't *require* big_space
//! to be used — the user decides.

/// Returns true if `big_space` support is enabled for this build. Reserved
/// for a future `no-big-space` feature flag.
pub fn enabled() -> bool {
    true
}
