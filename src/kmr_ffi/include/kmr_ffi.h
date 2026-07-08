/* kmr_ffi.h — C ABI for the kmr robot control library.
 *
 * Link against libkmr_ffi (cdylib: libkmr_ffi.so / .dylib / kmr_ffi.dll,
 * or staticlib: libkmr_ffi.a). Built from the kmr_ffi crate.
 *
 * The Rust `kmr_core` internals are not exposed; only the surface below is.
 */
#ifndef KMR_FFI_H
#define KMR_FFI_H

#include <stddef.h> /* size_t */

#ifdef __cplusplus
extern "C" {
#endif

/* Result codes. Mirrors the Rust-side error set. */
typedef enum KmrStatus {
    KMR_OK = 0,           /* success */
    KMR_OUT_OF_RANGE = 1, /* joint index out of range */
    KMR_NULL_POINTER = 2  /* a required pointer argument was null */
} KmrStatus;

/* Opaque robot handle. Internals are owned by the library; never dereference.
 * A handle is valid only inside the callback that received it. */
typedef struct KmrRobotState KmrRobotState;

/* Controller callback. Receives a borrowed robot handle valid only until the
 * callback returns. */
typedef void (*KmrController)(KmrRobotState *robot);

/* Register a controller callback and run the control loop once.
 * Blocks until the loop returns. */
void kmr_run(KmrController controller);

/* Write `value` to desired joint `index` through a robot handle.
 * Returns KMR_OK, KMR_OUT_OF_RANGE, or KMR_NULL_POINTER. */
KmrStatus kmr_robot_set_q_at(KmrRobotState *robot, float value, size_t index);

#ifdef __cplusplus
}
#endif

#endif /* KMR_FFI_H */
