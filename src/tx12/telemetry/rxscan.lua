-- Buoy TX12 operational telemetry pages for EdgeTX B&W radios.
-- Copy to /SCRIPTS/TELEMETRY/rxscan.lua (filename must be <= 6 chars).
-- Read-only: controls remain assigned to the model mixes and buoy firmware.

local SENSOR_SLOTS = 32
local sensors = {}
local page = 1
local PAGE_TITLES = {"STATUS", "NAV", "POWER/RADIO", "CAL"}
local homeSource = "WAIT"
local homeLatitude, homeLongitude, homeDistance, homeBearing, homeLastUpdate
local bearingLastUpdate
local buoyStatus = {}
local calibrationStatus = {}
local statusLastUpdate, calibrationLastUpdate
local crsfFrameCount = 0
local passthroughCount = 0
local textFrameCount = 0
local buoyFrameCount = 0
local lastText = "NONE"
local lastFrameType = "--"
local lastPassthroughType = "--"
local calibrationWasActive = false
local calibrationSawStep = false
local calibrationSawAbort = false
local calibrationWasCompleted = false
local calibrationView = 1

local function word(data, first)
  return data[first] + data[first + 1] * 256 + data[first + 2] * 65536 + data[first + 3] * 16777216
end

local function parseHomeText(text)
  local source, latitude, longitude = string.match(text, "^HOME%s+(%u+)%s*([%+%-%.%d]*)%s*([%+%-%.%d]*)")
  if not source then return end
  homeSource, homeLatitude, homeLongitude = source, tonumber(latitude), tonumber(longitude)
  if source == "NONE" then
    homeLatitude, homeLongitude, homeDistance, homeBearing = nil, nil, nil, nil
  end
  homeLastUpdate = getTime and getTime() or 0
end

local function parseBuoyText(message)
  if string.sub(message, 1, 3) == "B1," then
    local values = {}
    for field in string.gmatch(string.sub(message, 4), "[^,]+") do
      values[#values + 1] = tonumber(field)
    end
    if #values ~= 13 then return end
    buoyStatus = {armed=values[1], assist=values[2], mission=values[3],
      percent=values[4], waypoint=values[5], count=values[6],
      session=values[7], audio=values[8], heading=values[9],
      distance=values[10], fix=values[11], home=values[12], block=values[13]}
    statusLastUpdate = getTime and getTime() or 0
    buoyFrameCount = buoyFrameCount + 1
    homeSource = ({[0]="NONE", [1]="MANUAL", [2]="MISSION"})[values[12]] or "WAIT"
    homeDistance = values[10] >= 0 and values[10] or nil
    homeLastUpdate = statusLastUpdate
  elseif string.sub(message, 1, 4) == "BC1," then
    local values = {}
    for field in string.gmatch(string.sub(message, 5), "[^,]+") do
      values[#values + 1] = tonumber(field)
    end
    if #values ~= 5 then return end
    calibrationStatus = {state=values[1], step=values[2], power=values[3],
      max=values[4], duration=values[5]}
    calibrationLastUpdate = getTime and getTime() or 0
  end
end

local function statusLive(lastUpdate)
  return lastUpdate and getTime and getTime() - lastUpdate < 600
end

local function pollHomeTelemetry()
  if not crossfireTelemetryPop then return end
  for _ = 1, 12 do
    local frameType, data = crossfireTelemetryPop()
    if frameType == nil then break end
    crsfFrameCount = crsfFrameCount + 1
    lastFrameType = string.format("%02X", frameType)
    -- EdgeTX's Lua queue removes the CRSF address, length and command.
    -- For 0x80, data[1] is the passthrough subtype (F0/F1/F2).
    if frameType == 0x80 and data and #data >= 3 then
      passthroughCount = passthroughCount + 1
      lastPassthroughType = string.format("%02X", data[1])
      if data[1] == 0xF0 and #data >= 7 then
        local appId = data[2] + data[3] * 256
        if appId == 0x5004 then
          local packed = word(data, 4)
          local encodedDistance = packed % 4096
          homeDistance = math.floor(encodedDistance / 4) * (10 ^ (encodedDistance % 4))
          homeBearing = math.floor(packed / 33554432) * 3
          bearingLastUpdate = getTime and getTime() or 0
          homeLastUpdate = getTime and getTime() or 0
        end
      elseif data[1] == 0xF1 and #data >= 3 then
        local message = ""
        for index = 3, #data do
          if data[index] == 0 then break end
          message = message .. string.char(data[index])
        end
        textFrameCount = textFrameCount + 1
        lastText = string.sub(message, 1, 17)
        parseHomeText(message)
        parseBuoyText(message)
      end
    end
  end
end

local function refreshSensors()
  sensors = {}
  if not model or not model.getSensor then return end
  for index = 0, SENSOR_SLOTS - 1 do
    local sensor = model.getSensor(index)
    if sensor and sensor.name and sensor.name ~= "" then sensors[#sensors + 1] = sensor end
  end
end

local function clipped(value, length)
  value = tostring(value or "")
  if #value > length then return string.sub(value, 1, length) end
  return value
end

local function formatValue(value, precision)
  if value == nil then return "--" end
  if type(value) == "number" then
    precision = math.max(0, math.min(2, tonumber(precision) or 0))
    return string.format("%." .. precision .. "f", value)
  end
  if type(value) == "table" then
    if value.lat and value.lon then return string.format("%.3f %.3f", value.lat, value.lon) end
    if #value > 0 then return string.format("%d CELLS", #value) end
    return "TABLE"
  end
  return clipped(value, 8)
end

local function readSensor(sensor)
  if getSourceValue then
    local value, current, fresh = getSourceValue(sensor.name)
    if value == nil then return "--", "WAIT" end
    if fresh then return formatValue(value, sensor.prec), "LIVE" end
    if current then return formatValue(value, sensor.prec), "OLD" end
    return "--", "LOST"
  end
  local value = getValue and getValue(sensor.name) or nil
  if value == nil then return "--", "WAIT" end
  return formatValue(value, sensor.prec), "VAL"
end

local function findSensor(names)
  for _, wanted in ipairs(names) do
    for _, sensor in ipairs(sensors) do
      if string.upper(sensor.name) == wanted then return sensor end
    end
  end
  return nil
end

local function findBatteryPercentSensor()
  local sensor = findSensor({"BAT%", "BATT%", "BATT_PCT", "BAT_PCT", "BATPCT", "BATTERY%", "BATTERY_PCT", "%"})
  if sensor then return sensor end
  -- EdgeTX/ExpressLRS releases expose the MAVLink remaining value under
  -- slightly different names. Accept a discovered battery sensor that
  -- explicitly identifies itself as a percent/remaining reading.
  for _, candidate in ipairs(sensors) do
    local name = string.upper(candidate.name or "")
    if string.find(name, "BAT", 1, true) and
       (string.find(name, "%", 1, true) or string.find(name, "PCT", 1, true) or
        string.find(name, "PERCENT", 1, true) or string.find(name, "REMAIN", 1, true)) then
      return candidate
    end
  end
  return nil
end

local function metric(names, suffix)
  local sensor = findSensor(names)
  if not sensor then return "--", "WAIT" end
  local value, freshness = readSensor(sensor)
  if freshness == "WAIT" or freshness == "LOST" then return "--", freshness end
  if suffix and value ~= "--" then value = value .. suffix end
  return value, freshness
end

local function missionState()
  return ({[0]="IDLE", [1]="STARTING", [2]="MOVING", [3]="COLLECT",
           [4]="RETURN HOME", [5]="STOPPED", [6]="COMPLETE", [7]="HOME",
           [8]="BLOCKED"})[buoyStatus.mission] or "UNKNOWN"
end

local function controlModeLabel()
  return ({[0]="DIRECT", [1]="ASSIST", [2]="NO HEADING",
           [3]="MISSION"})[buoyStatus.assist] or "UNKNOWN"
end

local function missionBlock()
  return ({[0]="NONE", [1]="ARM CYCLE", [2]="POWER/STICKS", [3]="WIFI OWNER",
           [4]="NOT READY", [5]="GPS QUALITY", [6]="BOUNDARY", [7]="AT HOME",
           [8]="MAG CAL", [9]="MODE CYCLE", [10]="V4 MISSION", [11]="NO SD",
           [12]="SESSION BUSY", [13]="ROUTE INVALID", [14]="ARM NOT HIGH",
           [15]="SERVICE BUSY", [16]="NO SESSION"})[buoyStatus.block] or tostring(buoyStatus.block)
end

local function drawStatusPage()
  local live = statusLive(statusLastUpdate)
  if live then
    lcd.drawText(0, 10, buoyStatus.armed == 1 and "ARMED" or "DISARMED")
    lcd.drawText(0, 19, "CONTROL " .. clipped(controlModeLabel(), 12))
    lcd.drawText(0, 28, "MISSION " .. clipped(missionState(), 12))
    if buoyStatus.mission > 0 and buoyStatus.mission < 5 then
      lcd.drawText(0, 37, "PROGRESS " .. buoyStatus.percent .. "%  WP " ..
        buoyStatus.waypoint .. "/" .. buoyStatus.count)
    else
      lcd.drawText(0, 37, "MISSION NOT RUNNING")
    end
    local session = buoyStatus.session == 0 and "OFF" or
      (buoyStatus.audio == 1 and "AUDIO + DATA" or "DATA ONLY")
    lcd.drawText(0, 46, "SESSION " .. session)
    if buoyStatus.block and buoyStatus.block > 0 then
      lcd.drawText(0, 55, "BLOCK " .. clipped(missionBlock(), 15))
    else
      lcd.drawText(0, 55, "HOME " .. (homeSource == "NONE" and "NOT SET" or homeSource))
    end
  else
    local quality = metric({"RQLY", "TQLY"}, "%")
    lcd.drawText(0, 10, "NO BUOY STATUS PACKET")
    lcd.drawText(0, 19, "RADIO LINK " .. quality)
    lcd.drawText(0, 28, "TEXT " .. textFrameCount .. " B1 " .. buoyFrameCount)
    lcd.drawText(0, 37, "PASS " .. passthroughCount .. " SUB " .. lastPassthroughType)
    lcd.drawText(0, 46, "CRSF " .. crsfFrameCount .. " TYPE " .. lastFrameType)
    lcd.drawText(0, 55, "LAST " .. clipped(lastText, 16))
  end
end

local function drawNavigationPage()
  local live = statusLive(statusLastUpdate)
  local gps = live and (buoyStatus.fix == 1 and "FIX" or "NO FIX") or "UNKNOWN"
  lcd.drawText(0, 10, "GPS " .. gps)
  local source = not statusLive(homeLastUpdate) and "UNKNOWN" or
    (homeSource == "NONE" and "NOT SET" or homeSource)
  lcd.drawText(0, 19, "HOME " .. source)
  lcd.drawText(0, 28, "DIST " .. (statusLive(homeLastUpdate) and homeDistance and
    (homeDistance .. " m") or "--"))
  local heading = live and buoyStatus.heading or -1
  if heading < 0 then
    local yaw = metric({"YAW"})
    local radians = tonumber(yaw)
    if radians then heading = math.floor((radians * 180 / math.pi) % 360 + 0.5) end
  end
  lcd.drawText(0, 37, "COMPASS " .. (heading >= 0 and (heading .. " deg") or "--"))
  lcd.drawText(0, 46, "HOME BEARING " ..
    (statusLive(bearingLastUpdate) and homeBearing and (homeBearing .. " deg") or "--"))
  local speed = metric({"SPD", "GSPD", "GPS_SPEED"})
  local sats = metric({"SATS", "SAT"})
  lcd.drawText(0, 55, "SPD " .. speed .. " SAT " .. sats)
end

local function drawPowerLinkPage()
  local voltage = metric({"RXVT", "RXV", "RXVOLT", "RXVOLTAGE", "VFAS", "VOLT"})
  local volts = tonumber(voltage)
  lcd.drawText(0, 10, "PACK " .. voltage .. " V" ..
    (volts and volts < 9.6 and "  LOW" or ""))
  local batteryPercentSensor = findBatteryPercentSensor()
  local percent = "--"
  if batteryPercentSensor then
    percent = readSensor(batteryPercentSensor)
  end
  local percentSuffix = "%"
  if percent == "--" or string.find(percent, "%%") then percentSuffix = "" end
  lcd.drawText(0, 19, "BATTERY " .. percent .. percentSuffix)
  if volts and volts < 9.6 then
    lcd.drawText(0, 28, "CHECK 3S POWER INPUT")
  else
    lcd.drawText(0, 28, "3S ESTIMATE FROM V")
  end
  local quality = metric({"RQLY", "TQLY"})
  local rssi = metric({"1RSS", "2RSS", "RSSI", "TRSS"})
  local snr = metric({"RSNR", "TSNR"})
  lcd.drawText(0, 37, "LINK QUALITY " .. quality .. "%")
  lcd.drawText(0, 46, "RX RSSI " .. rssi .. " dBm")
  lcd.drawText(0, 55, "SNR " .. snr .. " dB")
end

local function readMixSource(sourceName)
  if getFieldInfo and getValue then
    local field = getFieldInfo(sourceName)
    if field and field.id then return getValue(field.id) end
  end
  if getValue then return getValue(sourceName) end
  return nil
end

local function readChannelPulse(channel)
  local value = readMixSource("ch" .. channel)
  if value == nil then return nil end
  -- EdgeTX channel sources are normalized to -1024..1024; display as the
  -- equivalent receiver pulse so it can be compared with the buoy inputs.
  return math.floor(1500 + value * 500 / 1024 + 0.5)
end

local function localCalibrationOutputs()
  local names = {"CalEn", "CalCmd", "CalMode", "CalIdx", "CalPwr", "CalDur"}
  local values = {}
  for index, name in ipairs(names) do values[index] = readMixSource(name) end
  local complete = true
  for _, value in ipairs(values) do if value == nil then complete = false end end
  if complete then return values end

  -- Companion and some EdgeTX builds expose Lua mix outputs by slot name.
  -- mixcal should be installed in the first available Lua Mixes slot.
  for slot = 1, 7 do
    local candidate = {}
    for output = 1, 6 do
      local suffix = string.char(string.byte("a") + output - 1)
      candidate[output] = readMixSource("LUA" .. slot .. suffix)
    end
    if candidate[1] and math.abs(candidate[1]) > 900 and
       candidate[3] and math.abs(candidate[3]) > 900 and
       candidate[6] and math.abs(candidate[6]) > 900 then
      return candidate
    end
  end
  return nil
end

local function drawCalibrationPage()
  local ch11, ch12, ch13 = readChannelPulse(11), readChannelPulse(12), readChannelPulse(13)
  local ch14, ch15, ch16 = readChannelPulse(14), readChannelPulse(15), readChannelPulse(16)
  local outputs = localCalibrationOutputs()
  local stateValue, stateFresh = metric({"CAL_STAT"})
  local stepValue = metric({"CAL_STEP"})
  local powerTelemetry, powerFresh = metric({"CAL_PWR"})
  local stateCode = tonumber(stateValue)
  if statusLive(calibrationLastUpdate) then
    stateCode = calibrationStatus.state
    stepValue = tostring(calibrationStatus.step)
    powerTelemetry = tostring(calibrationStatus.power)
    stateFresh = "LIVE"
  end
  local remoteState = ({[1]="ACTIVE", [2]="STOPPED", [3]="COMPLETE",
    [4]="BLOCKED", [5]="SD ERROR"})[stateCode]

  local marker, command, modeValue, indexValue, powerValue, durationValue
  local active, step, power, duration
  if outputs then
    marker, command, modeValue = outputs[1], outputs[2], outputs[3]
    indexValue, powerValue, durationValue = outputs[4], outputs[5], outputs[6]
    active = marker > 512
    local testIndex = math.floor(math.max(0, math.min(5, (indexValue + 1024) / 409.6)) + 0.5)
    step = testIndex + 1
    power = math.floor(math.max(0, math.min(100, (powerValue + 1024) / 20.48)) / 10 + 0.5) * 10
    duration = durationValue >= 0 and 20 or 10
  else
    step = tonumber(stepValue)
    power = tonumber(powerTelemetry)
  end

  local directions = modeValue and modeValue >= 0
    and {"FL FWD", "FL REV", "FR FWD", "FR REV", "REAR FWD", "REAR REV"}
    or {"FWD", "REV", "RIGHT", "LEFT", "YAW CW", "YAW CCW"}

  if outputs and active and not calibrationWasActive then
    calibrationSawStep = false
    calibrationSawAbort = false
    calibrationWasCompleted = false
  end
  if outputs and active and math.abs(command) < 100 then calibrationSawStep = true end
  if outputs and active and command > 400 then calibrationSawAbort = true end
  if outputs and not active and calibrationWasActive then
    calibrationWasCompleted = calibrationSawStep and not calibrationSawAbort
  end
  if outputs then calibrationWasActive = active end

  local anyChannels = ch11 or ch12 or ch13 or ch14 or ch15 or ch16
  local txPhase = "NO CH DATA"
  if ch11 and ch11 < 1500 then txPhase = "IDLE / WAIT TRIGGER"
  elseif ch12 and ch12 >= 1170 and ch12 <= 1230 then txPhase = "START CMD"
  elseif ch12 and ch12 >= 1770 then txPhase = "STOP CMD"
  elseif ch12 and ch12 >= 1470 and ch12 <= 1530 then txPhase = "STEP CMD (5/10s)"
  elseif ch11 then txPhase = "ENABLED / WAIT"
  elseif anyChannels then txPhase = "PARTIAL CH DATA" end

  local state = remoteState or (outputs and "IDLE" or (anyChannels and "TX ONLY" or "NO CH DATA"))
  if outputs and active then
    if calibrationSawAbort then state = "STOPPING"
    elseif calibrationSawStep then state = "RUNNING"
    elseif command < -450 then state = "PREPARING"
    else state = "READY / ARM SE" end
  elseif outputs and calibrationWasCompleted then
    state = "COMPLETE"
  elseif outputs and calibrationSawAbort then
    state = "STOPPED"
  end

  if calibrationView == 1 then
    lcd.drawText(0, 8, clipped("TX " .. txPhase, 21))
    local buoyLine = "BUOY: " .. (remoteState or "CHECK WIFI LOG")
    if remoteState and statusLive(calibrationLastUpdate) then
      buoyLine = buoyLine .. " " .. tostring(calibrationStatus.step) ..
        " " .. tostring(calibrationStatus.power) .. "%"
    end
    lcd.drawText(0, 16, clipped(buoyLine, 21))
    local function row(y, channel, label, pulse, decoded)
      lcd.drawText(0, y, string.format("%d %-4s %s %s", channel, label,
        pulse and tostring(pulse) or "----", decoded or ""))
    end
    local commandLabel = ch12 and ch12 >= 1770 and "STOP" or
      (ch12 and ch12 >= 1470 and ch12 <= 1530 and "STEP" or
      (ch12 and ch12 >= 1170 and ch12 <= 1230 and "START" or
      (ch12 and ch12 >= 1320 and ch12 <= 1380 and "IDLE" or "OTHER")))
    local index = ch14 and math.floor(math.max(0, math.min(5, (ch14 - 1000) / 200)) + 0.5) or nil
    lcd.drawText(0, 24, "EN " .. (ch11 and (ch11 >= 1500 and "ON" or "OFF") or "--") ..
      " CMD " .. commandLabel)
    row(32, 13, "MODE", ch13, ch13 and (ch13 >= 1500 and "MOTOR" or "MIX") or "")
    row(40, 14, "STEP", ch14, index and (tostring(index + 1) .. "/6") or "")
    row(48, 15, "POWER", ch15, ch15 and (tostring(math.floor(math.max(0, math.min(100, (ch15 - 1000) / 10)) + 0.5)) .. "%") or "")
    row(56, 16, "DUR", ch16, ch16 and (ch16 >= 1500 and "10s" or "5s") or "")
  elseif calibrationView == 2 then
    lcd.drawText(0, 8, "EXPECTED PULSE 1/2")
    lcd.drawText(0, 16, "ENTER: next view")
    lcd.drawText(0, 24, "11 EN 1000/2000")
    lcd.drawText(0, 32, "12 START 1200 IDLE1350")
    lcd.drawText(0, 40, "12 STEP1500 STOP1800")
    lcd.drawText(0, 48, "13 MIX1000 MOTOR2000")
    lcd.drawText(0, 56, "PULSE 5 OR 10 SEC")
  else
    lcd.drawText(0, 8, "EXPECTED PULSE 2/2")
    lcd.drawText(0, 16, "ENTER: actual view")
    lcd.drawText(0, 24, "14 STEP 1000..2000")
    lcd.drawText(0, 32, "15 POWER 1100=10%")
    lcd.drawText(0, 40, "   +100us per +10%")
    lcd.drawText(0, 48, "16 DUR 1000=5s")
    lcd.drawText(0, 56, "       2000=10s")
  end
end

local function handleEvent(event, pages)
  local nextPage = (EVT_VIRTUAL_NEXT_PAGE and event == EVT_VIRTUAL_NEXT_PAGE) or
    (EVT_PLUS_BREAK and event == EVT_PLUS_BREAK)
  local previousPage = (EVT_VIRTUAL_PREV_PAGE and event == EVT_VIRTUAL_PREV_PAGE) or
    (EVT_MINUS_BREAK and event == EVT_MINUS_BREAK)
  if nextPage then page = page % pages + 1
  elseif previousPage then page = (page - 2) % pages + 1 end
end

local function run(event)
  pollHomeTelemetry()
  refreshSensors()
  local pageCount = 4
  handleEvent(event, pageCount)
  if page == 4 and EVT_ENTER_BREAK and event == EVT_ENTER_BREAK then
    calibrationView = calibrationView % 3 + 1
  end
  lcd.clear()
  lcd.drawText(0, 0, "BUOY " .. page .. "/" .. pageCount .. " " .. PAGE_TITLES[page])
  if page == 1 then drawStatusPage()
  elseif page == 2 then drawNavigationPage()
  elseif page == 3 then drawPowerLinkPage()
  elseif page == 4 then drawCalibrationPage()
  end
end

local function init()
  refreshSensors()
end

return { init = init, run = run }
