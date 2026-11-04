# Moves every actuator by 0.1 rad a tick, from the newest sample.


def step(read, write, time):
    if read.samples == 0:
        # No sample to move from: no hardware. Shut down.
        return 1
    for i in range(len(read.q[0])):
        write.set_q(i, read.q[0][i] + 0.1)
    return 0
