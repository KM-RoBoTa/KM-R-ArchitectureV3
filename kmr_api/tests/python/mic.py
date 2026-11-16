# Moves actuator 0 once the microphone hears a sample louder than 1000.


def each_tick(sensors, state):
    mic = sensors.mic(0)
    if mic and max(map(abs, mic.samples()), default=0) > 1000:
        state.set_q_at(0.1, 0)
