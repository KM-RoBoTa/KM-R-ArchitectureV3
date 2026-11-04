//! The engine's log macros: `tracing`'s with `std`, no-ops without it
//! (SRS NFR-7: no `alloc`, which `tracing` needs without `std`).

#[cfg(feature = "std")]
pub(crate) use tracing::{error, info, trace, warn};

/// Expands to nothing: no logger without `std`.
#[cfg(not(feature = "std"))]
macro_rules! silent {
    ($($ignored:tt)*) => {{}};
}

#[cfg(not(feature = "std"))]
pub(crate) use {silent as error, silent as info, silent as trace, silent as warn};
