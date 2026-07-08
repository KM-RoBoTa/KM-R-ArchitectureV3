/* main.c — drive the kmr control loop from C through the FFI shim.
 *
 * Build (against the precompiled shared lib, from the workspace root):
 *   cc src/kmr_ffi/examples/main.c \
 *      -I src/kmr_ffi/include \
 *      -L target/release -lkmr_ffi \
 *      -o /tmp/kmr_c_demo
 *   LD_LIBRARY_PATH=target/release /tmp/kmr_c_demo
 *
 * See src/kmr_ffi/README.md for static-linking and full details.
 */
#include <stdio.h>
#include "kmr_ffi.h"

/* Called once by kmr_run with a borrowed robot handle. */
static void my_controller(KmrRobotState *robot) {
    KmrStatus s = kmr_robot_set_q_at(robot, 1.5f, 2);
    printf("set joint 2 = 1.5 -> status %d\n", s); /* expect 0 (KMR_OK) */

    s = kmr_robot_set_q_at(robot, 9.0f, 99);
    printf("set joint 99       -> status %d\n", s); /* expect 1 (OUT_OF_RANGE) */
}

int main(void) {
    kmr_run(my_controller);
    return 0;
}
