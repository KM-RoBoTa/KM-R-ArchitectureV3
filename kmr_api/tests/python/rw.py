# Reads the state, keeps a count in ctx, writes three actuators.


def each_tick(time, ctx, state):
    ctx.ticks = (ctx.ticks or 0) + 1
    assert time.tick() == ctx.ticks - 1, (time.tick(), ctx.ticks)
    assert time.dt() == 0.001
    assert len(state.q()) == N
    assert state.q_at(1) == 10.0
    try:
        state.q_at(N)
        raise AssertionError("an index past N reads")
    except StateError:
        pass
    state.set_q_at(0.25, 1)
    state.group([2, 0]).set_all_q([0.5, -0.5])
