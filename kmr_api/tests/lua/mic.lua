-- Moves actuator 0 once the microphone hears a sample louder than 1000.

function each_tick(sensors, state)
    local mic = sensors:mic(0)
    if not mic then return end
    for _, s in ipairs(mic:samples()) do
        if math.abs(s) > 1000 then
            state:set_q_at(0.1, 0)
            return
        end
    end
end
