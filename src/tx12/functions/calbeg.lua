-- Announce the calibration start requested by the Cal Trigger one-time tool.
-- Install as /SCRIPTS/FUNCTIONS/calbeg.lua and run via a Special Function
-- assigned to ON with Repeat = ON. The one-time tool signals through SHM slot 0.

local SIGNAL_SLOT = 0
local SOUND_FILE = "/SOUNDS/en/calbeg.wav"

local function checkForCalibrationStart()
  if not getShmVar or not setShmVar then return end

  local request = getShmVar(SIGNAL_SLOT)
  if request and request > 0 then
    -- Clear before playback so this announcement can happen only once per start.
    setShmVar(SIGNAL_SLOT, 0)
    if playFile then playFile(SOUND_FILE) end
  end
end

local function init()
  -- Discard stale requests left behind by a radio restart.
  if setShmVar then setShmVar(SIGNAL_SLOT, 0) end
end

return {
  init = init,
  run = checkForCalibrationStart,
  background = checkForCalibrationStart,
}
