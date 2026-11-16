-- The clap controller of src/main.rs: once home, a clap runs both snakes' sines for 2 s.
-- Edit this file while the robot runs: the change is live on the next tick.
-- Actuator indices are 0-based, as in Rust and in the URDF.

local HOME_TIME = 3.0 -- s, the time HOME takes to ramp to the centre; claps are heard after it
local TIME_LIMIT = 60.0 -- s, the run shuts down after this
local CENTER = 0.0 -- rad: raw 512 on both models
local AMPLITUDE = 0.15 -- rad, the RX-28 sine; the AX-12A sine has twice it
local PEAK = 1000 -- a microphone sample louder than this is a clap
local MOVE_FOR = 2.0 -- s, one period of the 0.5 Hz sine

-- `n` copies of `v`.
local function rep(v, n)
    local t = {}
    for i = 1, n do t[i] = v end
    return t
end

-- Actuators `from` to `to`, both included.
local function range(from, to)
    local t = {}
    for i = from, to do t[#t + 1] = i end
    return t
end

local AX = range(0, 11) -- the AX-12A snake: the URDF's first actuators
local RX = range(12, N - 1) -- the RX-28 snake: every actuator after them

local function sine(amplitude, t)
    return CENTER + amplitude * math.sin(2 * math.pi * 0.5 * t)
end

function each_tick(time, state, sensors, ctx)
    local now = time:elapsed()
    if now > TIME_LIMIT then shutdown("time limit") end
    if now < HOME_TIME then return end
    -- A peak heard while moving is ignored (the servos are loud): one clap, one motion.
    local moving = ctx.since ~= nil and now - ctx.since < MOVE_FOR
    local loudest = 0
    local mic = sensors:mic(0)
    if mic then
        for _, s in ipairs(mic:samples()) do loudest = math.max(loudest, math.abs(s)) end
    end
    if not moving and loudest > PEAK then
        print(string.format("clap at %.2f s: peak %d > %d, both snakes move for %.1f s",
            now, loudest, PEAK, MOVE_FOR))
        ctx.since, moving = now, true
    end

    local t = moving and now - ctx.since or 0.0
    local ax = moving and sine(2 * AMPLITUDE, t) or CENTER
    local rx = moving and sine(AMPLITUDE, t) or CENTER
    state:group(AX):set_all_q(rep(ax, #AX))
    state:group(RX):set_all_q(rep(rx, #RX))
end
