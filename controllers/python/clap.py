# The clap controller of src/main.rs: once home, a clap runs both snakes' sines for 2 s.
# Edit this file while the robot runs: the change is live on the next tick.
from math import sin, tau

HOME_TIME = 3.0  # s, the time HOME takes to ramp to the centre; claps are heard after it
TIME_LIMIT = 60.0  # s, the run shuts down after this
AX = range(12)  # the AX-12A snake: the URDF's first actuators
RX = range(12, N)  # the RX-28 snake: every actuator after them
CENTER = 0.0  # rad: raw 512 on both models
AMPLITUDE = 0.15  # rad, the RX-28 sine; the AX-12A sine has twice it
PEAK = 1000  # a microphone sample louder than this is a clap
MOVE_FOR = 2.0  # s, one period of the 0.5 Hz sine


def sine(amplitude, t):
    return CENTER + amplitude * sin(tau * 0.5 * t)


def each_tick(time, state, sensors, ctx):
    now = time.elapsed()
    if now > TIME_LIMIT:
        shutdown("time limit")
    if now < HOME_TIME:
        return
    # A peak heard while moving is ignored (the servos are loud): one clap, one motion.
    moving = ctx.since is not None and now - ctx.since < MOVE_FOR
    mic = sensors.mic(0)
    loudest = max(map(abs, mic.samples()), default=0) if mic else 0
    if not moving and loudest > PEAK:
        print(f"clap at {now:.2f} s: peak {loudest} > {PEAK}, both snakes move for {MOVE_FOR} s")
        ctx.since, moving = now, True

    t = now - ctx.since if moving else 0.0
    ax = sine(2 * AMPLITUDE, t) if moving else CENTER
    rx = sine(AMPLITUDE, t) if moving else CENTER
    state.group(AX).set_all_q([ax] * len(AX))
    state.group(RX).set_all_q([rx] * len(RX))

