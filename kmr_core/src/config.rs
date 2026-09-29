//! The configuration of the KMR architecture.
//!
//! There's two main configuration types:
//! - The [`internal_config`]: unavailable to the API user.
//! - The [`user_config`]: easily accessible through the [`crate::robot::Robot`]
//!   builder pattern.
pub(crate) mod internal_config;
pub(crate) mod user_config;
