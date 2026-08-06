#![forbid(unsafe_op_in_unsafe_fn)]
// The exported BYOND ABI keeps the existing ten-argument pathfinder signature.
#![allow(clippy::too_many_arguments)]

pub mod navmap_pathfinder;

#[cfg(all(not(target_pointer_width = "32"), not(feature = "allow_non_32bit")))]
compile_error!(
    "Compiling for non-32bit is not allowed without enabling the `allow_non_32bit` feature."
);

use meowtonin::byond_fn;

#[byond_fn]
#[allow(dead_code)]
fn get_version() -> String {
    env!("CARGO_PKG_VERSION").to_owned()
}
