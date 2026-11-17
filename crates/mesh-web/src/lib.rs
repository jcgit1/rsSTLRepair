//! Compile-only placeholder for the future Axum-based repair web service.
//!
//! Public web hosting is explicitly deferred until the desktop and CLI engines
//! pass Milestone 5 reliability and security requirements.

use mesh_core::RepairProfile;

/// Configuration placeholder for the future web repair service.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ServiceLimits {
    pub max_upload_bytes: u64,
    pub default_profile: RepairProfile,
}

impl Default for ServiceLimits {
    fn default() -> Self {
        Self {
            max_upload_bytes: 64 * 1024 * 1024,
            default_profile: RepairProfile::Conservative,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn default_limits_use_conservative_profile() {
        let limits = ServiceLimits::default();
        assert_eq!(limits.default_profile, RepairProfile::Conservative);
    }
}
