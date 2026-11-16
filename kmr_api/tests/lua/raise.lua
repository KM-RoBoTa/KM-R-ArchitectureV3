-- Writes, then raises: nothing is applied.

function each_tick(state)
    local all = {}
    for i = 1, N do all[i] = 0.1 end
    state:set_all_q(all)
    error("boom")
end
