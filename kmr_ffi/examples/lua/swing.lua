-- Moves every actuator by 0.1 rad a tick, from the newest sample. Tables are 1-based.

function step(read, write, time)
  if read.samples == 0 then
    -- No sample to move from: no hardware. Shut down.
    return 1
  end
  for i = 1, #read.q[1] do
    write.set_q(i, read.q[1][i] + 0.1)
  end
  return 0
end
