# Writes, then raises: nothing is applied.


def each_tick(state):
    state.set_all_q([0.1] * N)
    raise ValueError("boom")
