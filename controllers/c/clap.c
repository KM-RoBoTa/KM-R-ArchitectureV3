/* The clap controller of src/main.rs: once home, a clap runs both snakes' sines for 2 s.
 * Rust keeps the builder, the buses, the actuators and the HOME ramp (examples/clap_c.rs). */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "kmr.h"

#define HOME_TIME 3.0   /* s, the time HOME takes to ramp to the centre; claps are heard after it */
#define TIME_LIMIT 60.0 /* s, the run shuts down after this */
#define AX_LEN 12       /* the AX-12A snake: the URDF's first actuators */
#define RX_LEN (KMR_N - AX_LEN) /* the RX-28 snake: every actuator after them */
#define CENTER 0.0f     /* rad: raw 512 on both models */
#define AMPLITUDE 0.15f /* rad, the RX-28 sine; the AX-12A sine has twice it */
#define PEAK 1000       /* a microphone sample louder than this is a clap */
#define MOVE_FOR 2.0    /* s, one period of the 0.5 Hz sine */
#define TAU 6.283185307179586

/* When the last clap's motion started: the context, mirrored as `Clap` in examples/clap_c.rs. */
typedef struct {
    double since;
    bool clapped;
} clap;

static float sine(float amplitude, double t) {
    return CENTER + amplitude * (float)sin(TAU * 0.5 * t);
}

/* The microphone's loudest sample this tick; 0 without a microphone or a sample. */
static int loudest(const kmr_sensors *sensors) {
    size_t len = 0, k;
    int peak = 0;
    const kmr_mic_reading *mic = kmr_mic(sensors, 0);
    const int16_t *samples;
    if (mic == NULL) {
        return 0;
    }
    samples = kmr_mic_samples(mic, &len);
    for (k = 0; k < len; ++k) {
        int a = abs(samples[k]);
        if (a > peak) {
            peak = a;
        }
    }
    return peak;
}

void move_on_clap_c(const kmr_time *time, kmr_state *state, const kmr_sensors *sensors, void *ctx) {
    static const size_t AX[AX_LEN] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11};
    size_t RX[RX_LEN];
    float ax[AX_LEN], rx[RX_LEN];
    clap *c = ctx;
    double now = kmr_elapsed(time), t;
    bool moving;
    int peak;
    size_t k;

    if (c == NULL) {
        kmr_shutdown(state, "move_on_clap_c needs kmr_api::c!(move_on_clap_c, with = Clap::default())");
        return;
    }
    if (now > TIME_LIMIT) {
        kmr_shutdown(state, "time limit");
    }
    if (now < HOME_TIME) {
        return;
    }
    /* A peak heard while moving is ignored (the servos are loud): one clap, one motion. */
    moving = c->clapped && now - c->since < MOVE_FOR;
    peak = loudest(sensors);
    if (!moving && peak > PEAK) {
        printf("clap at %.2f s: peak %d > %d, both snakes move for %.0f s\n", now, peak, PEAK,
               MOVE_FOR);
        c->since = now;
        c->clapped = true;
        moving = true;
    }

    t = moving ? now - c->since : 0.0;
    for (k = 0; k < AX_LEN; ++k) {
        ax[k] = moving ? sine(2.0f * AMPLITUDE, t) : CENTER;
    }
    for (k = 0; k < RX_LEN; ++k) {
        RX[k] = AX_LEN + k;
        rx[k] = moving ? sine(AMPLITUDE, t) : CENTER;
    }
    if (kmr_group_set_all_q(state, AX, AX_LEN, ax) != KMR_OK) {
        kmr_shutdown(state, "AX-12A group write failed");
    }
    if (kmr_group_set_all_q(state, RX, RX_LEN, rx) != KMR_OK) {
        kmr_shutdown(state, "RX-28 group write failed");
    }
}
