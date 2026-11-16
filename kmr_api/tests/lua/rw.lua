-- Reads the state, keeps a count in ctx, writes three actuators.

function each_tick(time, ctx, state)
    ctx.ticks = (ctx.ticks or 0) + 1
    assert(time:tick() == ctx.ticks - 1, time:tick())
    assert(time:dt() == 0.001)
    assert(#state:q() == N)
    assert(state:q_at(1) == 10.0)
    local ok, err = pcall(state.q_at, state, N)
    assert(not ok and StateError(err), "an index past N reads")
    state:set_q_at(0.25, 1)
    state:group({2, 0}):set_all_q({0.5, -0.5})
end
