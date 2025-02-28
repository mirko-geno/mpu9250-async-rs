//! Euler Angle Calculations
//!
//! Euler angles represent orientation using three angles:
//! - Psi (ψ): Yaw angle (rotation around Z)
//! - Theta (θ): Pitch angle (rotation around Y)
//! - Phi (φ): Roll angle (rotation around X)

use crate::quaternion::Quaternion;

/// Orientation represented as Euler angles in radians.
///
/// The angles describe rotations in this order:
/// 1. Roll (φ): Rotation around X axis
/// 2. Pitch (θ): Rotation around Y axis
/// 3. Yaw (ψ): Rotation around Z axis
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Euler {
    /// Yaw angle (ψ) in radians (-π to π)
    pub psi: f64,
    /// Pitch angle (θ) in radians (-π/2 to π/2)
    pub theta: f64,
    /// Roll angle (φ) in radians (-π to π)
    pub phi: f64,
}

impl From<Quaternion> for Euler {
    /// Converts a quaternion to Euler angles.
    ///
    /// The conversion uses these formulas:
    /// - ψ = atan2(2(qx*qy - qw*qz), 2(qw² + qx²) - 1)
    /// - θ = -asin(2(qx*qz + qw*qy))
    /// - φ = atan2(2(qy*qz - qw*qx), 2(qw² + qz²) - 1)
    fn from(q: Quaternion) -> Self {
        Self {
            psi: libm::atan2(
                (2.0 * q.x * q.y - 2.0 * q.w * q.z) as f64,
                (2.0 * q.w * q.w + 2.0 * q.x * q.x - 1.0) as f64,
            ),
            theta: -libm::asin((2.0 * q.x * q.z + 2.0 * q.w * q.y) as f64),
            phi: libm::atan2(
                (2.0 * q.y * q.z - 2.0 * q.w * q.x) as f64,
                (2.0 * q.w * q.w + 2.0 * q.z * q.z - 1.0) as f64,
            ),
        }
    }
}
