-- toolName = TNS|Mixer Cal|TNE
-- TX12 multi-page launcher for the mixcal.lua calibration sequence.
-- One-time tools suspend other Lua scripts, so this UI closes immediately
-- after handing the start request to G9. The mixer then resumes and runs.

local ACTION_G_INDEX = 8 -- zero-based EdgeTX index for G9
local page = 1
local pageCount = 3
local statusMessage = nil

local function drawLine(y, text, flags)
  lcd.drawText(2, y, text, flags or 0)
end

local function drawPage()
  lcd.clear()
  lcd.drawScreenTitle("MIXER CAL", page, pageCount)

  if page == 1 then
    drawLine(14, "Buoy mix calibration")
    drawLine(26, "Start the mixcal test")
    drawLine(42, "NEXT: see test plan")
  elseif page == 2 then
    drawLine(13, "TEST PLAN")
    drawLine(24, "6 directions / level")
    drawLine(34, "30%+; 10s pulses")
    drawLine(46, "3s neutral pause")
  elseif statusMessage then
    drawLine(19, statusMessage)
    drawLine(31, "SE low = disarmed")
    drawLine(43, "SB manual; ENTER retry")
  else
    drawLine(12, "PREFLIGHT")
    drawLine(21, "SB Manual; SE low")
    drawLine(30, "CH1/2/4 centered")
    drawLine(39, "CH3 min; no task active")
    drawLine(48, "Abort run: lower SE")
  end

  if page < pageCount then
    drawLine(57, "< PREV       NEXT >")
  else
    drawLine(57, "< PREV   ENTER START")
  end
end

local function sendStartRequest()
  if not model or not model.setGlobalVariable or not getFieldInfo or not getValue then
    lcd.clear()
    lcd.drawScreenTitle("MIXER CAL", 1, 1)
    statusMessage = "RADIO API UNAVAILABLE"
    drawLine(22, statusMessage)
    drawLine(35, "Check EdgeTX Lua API")
    return 0
  end

  local seField = getFieldInfo("se")
  local seValue = seField and seField.id and getValue(seField.id) or nil
  if seValue == nil then
    statusMessage = "CANNOT READ SE SWITCH"
    lcd.clear()
    lcd.drawScreenTitle("MIXER CAL", 1, 1)
    drawLine(22, statusMessage)
    drawLine(35, "No request sent")
    return 0
  elseif seValue > 512 then
    statusMessage = "DISARM FIRST (SE LOW)"
    lcd.clear()
    lcd.drawScreenTitle("MIXER CAL", 1, 1)
    drawLine(22, statusMessage)
    drawLine(35, "No request sent")
    drawLine(47, "SB manual; ENTER retry")
    return 0
  end

  statusMessage = nil

  -- Function scripts are suspended while this one-time tool is open. Leave
  -- a mailbox event for calbeg.lua to consume after this tool exits.
  if setShmVar then setShmVar(0, 1) end

  -- mixcal.lua consumes this high value on its first run after this tool exits.
  model.setGlobalVariable(ACTION_G_INDEX, 0, 1024)
  playTone(1200, 120, 0)
  return 1
end

local function run(event)
  if event == EVT_VIRTUAL_NEXT and page < pageCount then
    page = page + 1
    statusMessage = nil
  elseif event == EVT_VIRTUAL_PREV and page > 1 then
    page = page - 1
    statusMessage = nil
  elseif event == EVT_VIRTUAL_ENTER and page == pageCount then
    return sendStartRequest()
  elseif event == EVT_VIRTUAL_EXIT then
    return 1
  end

  drawPage()
  return 0
end

return { run = run }
