-- Buoy calibration command sequencer for EdgeTX 2.11+ Lua Mixes.
-- It only emits command values on CH11-CH16. Existing CH1-CH10 controls
-- remain standard model mixes and retain their normal fail-safe behavior.

local SCALE = 1024
local CMD_IDLE = -307       -- about 1350 us: outside all command windows
local CMD_START = -614      -- about 1200 us
local CMD_STEP = 0          -- 1500 us
local CMD_STOP = 614        -- about 1800 us
local PULSE_TICKS = 18      -- start/stop command duration; EdgeTX getTime() ticks are 10 ms
local NEUTRAL_INTERVAL_TICKS = 300 -- 3 seconds idle after each full motor run

local active = false
local started = false
local stopAt = nil
local completeAt = nil
local command = CMD_IDLE
local actionWasPressed = false
local nextStepAt = nil
local pulseAt = nil
local pulseTicks = PULSE_TICKS
local issuedCommandWasStep = false
local finalStepIssued = false
local mode = 0
local maxPower = 30
local pulseDurationMs = 10000
local durationSource = -1024 -- CH16 low = 10 seconds; high = 20 seconds
local power = 30
local testIndex = 0
local issuedIndex = 0
local issuedPower = 30
local lastTestAt = nil

-- G9 is reserved as the menu-trigger input. The one-time tool sets it high;
-- this mixer consumes the rising edge and clears it so it can be used again.
local ACTION_G_INDEX = 8 -- zero-based EdgeTX index for G9

local function clearMenuAction()
  if model and model.setGlobalVariable then
    model.setGlobalVariable(ACTION_G_INDEX, 0, -1024)
  end
end

local function elapsed(now, previousTick)
  return (now - previousTick) % 4294967296
end

local function clamp(value, minimum, maximum)
  if value < minimum then return minimum end
  if value > maximum then return maximum end
  return value
end

local function microsecondsToSource(us)
  return clamp(math.floor(((us - 1500) * 2.048) + 0.5), -1024, 1024)
end

local function percentToSource(percent)
  return clamp(math.floor((percent * 20.48) - 1024 + 0.5), -1024, 1024)
end

local function finishStop(now)
  active = false
  started = false
  stopAt = nil
  completeAt = nil
  pulseAt = nil
  pulseTicks = PULSE_TICKS
  issuedCommandWasStep = false
  finalStepIssued = false
  nextStepAt = nil
  command = CMD_IDLE
end

local function init()
  -- Do not clear G9 here: the one-time Cal Trigger tool sets it immediately
  -- before this mixer restarts. run() consumes and clears that handoff edge.
  actionWasPressed = false
end

local function run(Action, Arm, Mode, MaxPwr, Duration)
  local now = getTime()
  local actionPressed = (Action or 0) > 512
  local armHigh = (Arm or 0) > 512
  local actionEdge = actionPressed and not actionWasPressed
  actionWasPressed = actionPressed

  if actionEdge then clearMenuAction() end

  if not active then
    if actionEdge and not armHigh then
      active = true
      started = false
      mode = (Mode or 0) >= 1 and 1 or 0
      -- An older saved model can supply zero for a newly added VALUE input.
      -- Lua treats zero as truthy, so `MaxPwr or 30` would silently cap the
      -- run at 10% after clamping. Use the documented 30% default instead.
      local requestedMaxPower = tonumber(MaxPwr) or 0
      if requestedMaxPower < 30 then requestedMaxPower = 30 end
      maxPower = clamp(math.floor(requestedMaxPower / 10 + 0.5) * 10, 30, 100)
      pulseDurationMs = (Duration or 0) >= 1 and 20000 or 10000
      durationSource = pulseDurationMs == 20000 and SCALE or -SCALE
      power = 30
      testIndex = 0
      command = CMD_START
      pulseAt = now
      pulseTicks = PULSE_TICKS
      issuedCommandWasStep = false
      finalStepIssued = false
      lastTestAt = nil
    else
      command = CMD_IDLE
    end
  elseif stopAt then
    if elapsed(now, stopAt) >= PULSE_TICKS then
      finishStop(now)
    else
      command = CMD_STOP
    end
  elseif actionEdge then
    -- A fresh press after START is an immediate explicit abort request.
    command = CMD_STOP
    stopAt = now
  elseif started and not armHigh then
    -- SE low is the buoy's hardware-control abort gesture.
    command = CMD_STOP
    stopAt = now
  elseif pulseAt then
    -- Keep CH12 at STEP for the complete motor run. The buoy latches its
    -- rising edge, runs the requested duration, then this script supplies
    -- the full neutral coast interval before sending another step.
    if elapsed(now, pulseAt) >= pulseTicks then
      command = CMD_IDLE
      pulseAt = nil
      pulseTicks = PULSE_TICKS
      if issuedCommandWasStep then
        issuedCommandWasStep = false
        if finalStepIssued then
          completeAt = now + NEUTRAL_INTERVAL_TICKS
        else
          nextStepAt = now + NEUTRAL_INTERVAL_TICKS
        end
      end
    end
  elseif completeAt then
    if elapsed(now, completeAt) < 2147483648 then finishStop(now) end
    command = CMD_IDLE
  elseif not started then
    command = CMD_IDLE
    if armHigh then
      started = true
      nextStepAt = now + 100 -- allow the buoy to pass the SE arming interlock
    end
  elseif nextStepAt and elapsed(now, nextStepAt) < 2147483648 then
    command = CMD_STEP
    pulseAt = now
    pulseTicks = math.floor(pulseDurationMs / 10)
    issuedCommandWasStep = true
    lastTestAt = now
    issuedIndex = testIndex
    issuedPower = power
    nextStepAt = nil
    if testIndex == 5 then
      if power >= maxPower then
        finalStepIssued = true
      else
        finalStepIssued = false
        testIndex = 0
        power = power + 10
      end
    else
      finalStepIssued = false
      testIndex = testIndex + 1
    end
  else
    command = CMD_IDLE
  end

  local marker = active and SCALE or -SCALE
  local selectedMode = mode == 1 and SCALE or -SCALE
  local indexUs = 1000 + ((command == CMD_STEP and issuedIndex or testIndex) * 200)
  local powerPercent = command == CMD_STEP and issuedPower or ((started and lastTestAt ~= nil) and power or maxPower)
  if not active then marker = -SCALE end
  return marker, command, selectedMode, microsecondsToSource(indexUs), percentToSource(powerPercent), durationSource
end

return {
  input = {
    { "Action", SOURCE }, -- Select G9; trigger with the calgo tool in SYS > Tools.
    { "Arm", SOURCE },    -- Assign the buoy arm switch source (SE).
    { "Mode", VALUE, 0, 1, 0 }, -- 0=mixer response; 1=individual motor/compass.
    { "MaxPwr", VALUE, 30, 100, 30 }, -- Start at 30%; highest test power is configurable.
    { "Duration", VALUE, 0, 1, 0 }, -- 0=10 seconds; 1=20 seconds for a large pool.
  },
  output = { "CalEn", "CalCmd", "CalMode", "CalIdx", "CalPwr", "CalDur" },
  run = run,
  init = init,
}
