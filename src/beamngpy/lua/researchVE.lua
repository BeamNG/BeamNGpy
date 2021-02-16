-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}
local logTag = 'ResearchVE'

local socket = require('libs/luasocket/socket.socket')
local rcom = require('utils/researchCommunication')

local sensorHandlers = {}

local server = nil
local clients = nil

local port = nil

local conSleep = 60

local _log = log
local function log(level, message)
  _log(level, logTag, message)
end

M.startConnection = function()
  if server == nil then
    server = rcom.openServer(0)
    local ip = nil
    ip, port = server:getsockname()
    local set = rcom.newSet()
    set:insert(server)
    server = set
    clients = rcom.newSet()
  end
  local cmd = 'extensions.hook("onVehicleConnectionReady", ' .. tostring(obj:getID()) .. ', ' .. tostring(port) .. ')'
  obj:queueGameEngineLua(cmd)
end

M.onDebugDraw = function()
  if server ~= nil then
    if conSleep <= 0 then
      conSleep = 60
      local newClients = rcom.checkForClients(server)
      for i = 1, #newClients do
        clients:insert(newClients[i])
        local ip, clientPort = newClients[i]:getsockname()
        log('I', 'Accepted new vehicle client: ' .. tostring(ip) .. '/' .. tostring(clientPort))
      end
    else
      conSleep = conSleep - 1
    end
  else
    return
  end

  while rcom.checkMessages(M, clients) do end
end

local function getVehicleState()
  local vehicleState = {
    pos = obj:getPosition(),
    dir = obj:getDirectionVector(),
    up = obj:getDirectionVectorUp(),
    vel = obj:getVelocity(),
    front = obj:getFrontPosition()
  }
  vehicleState['pos'] = {
    vehicleState['pos'].x,
    vehicleState['pos'].y,
    vehicleState['pos'].z
  }

  vehicleState['dir'] = {
    vehicleState['dir'].x,
    vehicleState['dir'].y,
    vehicleState['dir'].z
  }

  vehicleState['up'] = {
    vehicleState['up'].x,
    vehicleState['up'].y,
    vehicleState['up'].z
  }

  vehicleState['vel'] = {
    vehicleState['vel'].x,
    vehicleState['vel'].y,
    vehicleState['vel'].z
  }

  vehicleState['front'] = {
    vehicleState['front'].x,
    vehicleState['front'].y,
    vehicleState['front'].z
  }

  return vehicleState
end

M.requestVehicleInfo = function()
  local info = {}
  info['port'] = port
  -- TODO: Add more info down the line, port's enough for now

  local cmd = 'extensions.hook("onVehicleInfoReady", ' .. tostring(obj:getID()) .. ', ' .. serialize(info) .. ')'
  obj:queueGameEngineLua(cmd)
end

-- Handlers

M.handleHello = function(skt, msg)
  local resp = {type = 'Hello', protocolVersion = rcom.protocolVersion}
  rcom.sendMessage(skt, resp)
end

local function submitInput(inputs, key)
  local val = inputs[key]
  if val ~= nil then
    input.event(key, val, 1)
  end
end

M.handleControl = function(skt, msg)
  submitInput(msg, 'throttle')
  submitInput(msg, 'steering')
  submitInput(msg, 'brake')
  submitInput(msg, 'parkingbrake')
  submitInput(msg, 'clutch')

  local gear = msg['gear']
  if gear ~= nil then
    drivetrain.shiftToGear(gear)
  end

  rcom.sendACK(skt, 'Controlled')
end

M.handleSetShiftMode = function(skt, msg)
  drivetrain.setShifterMode(msg['mode'])
  rcom.sendACK(skt, 'ShiftModeSet')
end

sensorHandlers.GForces = function(msg)
  local resp = {type='GForces'}

  resp['gx'] = sensors.gx
  resp['gx2'] = sensors.gx2
  resp['gx_smooth_max'] = sensors.gxSmoothMax
  resp['gy'] = sensors.gy
  resp['gy2'] = sensors.gy2
  resp['gz'] = sensors.gz
  resp['gz2'] = sensors.gz2

  return resp
end

sensorHandlers.Electrics = function(msg)
  local resp = {type = 'Electrics'}
  resp['values'] = electrics.values
  return resp
end

sensorHandlers.Damage = function(msg)
  local resp = {type = 'Damage'}
  resp['damage_ext'] = beamstate.damageExt
  resp['deform_group_damage'] = beamstate.deformGroupDamage
  resp['lowpressure'] = beamstate.lowpressure
  resp['damage'] = beamstate.damage
  resp['part_damage'] = beamstate.getPartDamageData()
  return resp
end

sensorHandlers.State = function(msg)
  local resp = {type = 'VehicleUpdate'}
  local vehicleState = getVehicleState()
  resp['state'] = vehicleState
  return resp
end

sensorHandlers.IMU = function(msg)
  local name = msg['name']
  local imu = imu.getIMU(name)
  return {
    name = imu.name,
    aX = imu.aX,
    aY = imu.aY,
    aZ = imu.aZ,
    gX = imu.gX,
    gY = imu.gY,
    gZ = imu.gZ
  }
end

local function getSensorData(request)
  local response, sensor_type, handler

  sensor_type = request['type']
  handler = sensorHandlers[sensor_type]
  if handler ~= nil then
    response = handler(request)
    return response
  end

  return nil
end

M.handleSensorRequest = function(skt, msg)
  local request, sensorData, data
  sensorData = {}
  request = msg['sensors']
  for k, v in pairs(request) do
    data = getSensorData(v)
    if data == nil then
      log('E', 'Could not get data for sensor: ' .. k)
    end
    sensorData[k] = data
  end

  local response = {type = 'SensorData', data = sensorData}
  rcom.sendMessage(skt, response)
end

M.handleSetColor = function(skt, msg)
  local cmd = 'Point4F(' .. msg['r'] .. ', ' .. msg['g'] .. ', ' .. msg['b'] .. ', ' .. msg['a'] .. ')'
  cmd = 'be:getObjectByID(' .. obj:getID() .. '):setColor(' .. cmd .. ')'
  obj:queueGameEngineLua(cmd)
  rcom.sendACK(skt, 'ColorSet')
end

M.handleSetAiMode = function(skt, msg)
  ai.setMode(msg['mode'])
  ai.stateChanged()
  rcom.sendACK(skt, 'AiModeSet')
end

M.handleSetAiLine = function(skt, msg)
  local nodes = msg['line']
  local fauxPath = {}
  local cling = msg['cling']
  local z = 0
  local speedList = {}
  for idx, n in ipairs(nodes) do
    local pos = vec3(n['pos'][1], n['pos'][2], 10000)
    if cling then
      z = obj:getSurfaceHeightBelow(pos:toFloat3())
    else
      z = n['pos'][3]
    end
    pos.z = z
    local fauxNode = {
      pos = pos,
      radius = 0,
      radiusOrig = 0,
    }
    table.insert(speedList, n['speed'])
    table.insert(fauxPath, fauxNode)
  end

  local arg = {
    script = fauxPath,
    wpSpeeds = speedList
  }

  ai.driveUsingPath(arg)
  ai.stateChanged()
  rcom.sendACK(skt, 'AiLineSet')
end

M.handleSetAiScript = function(skt, msg)
  local script = msg['script']
  local cling = msg['cling']

  if cling then
    for i, v in ipairs(script) do
      local pos = vec3(v.x, v.y, v.z)
      pos.z = obj:getSurfaceHeightBelow(pos:toFloat3())
      v.z = pos.z
    end
  end

  ai.startFollowing(script, 0, 0, 'never')

  ai.stateChanged()
  rcom.sendACK(skt, 'AiScriptSet')
end

M.handleSetAiSpeed = function(skt, msg)
  ai.setSpeedMode(msg['mode'])
  ai.setSpeed(msg['speed'])
  ai.stateChanged()
  rcom.sendACK(skt, 'AiSpeedSet')
end

M.handleSetAiTarget = function(skt, msg)
  local targetName = msg['target']
  obj:queueGameEngineLua('scenetree.findObjectById(' .. obj:getID() .. '):queueLuaCommand("ai.setTargetObjectID(" .. scenetree.findObject(\'' .. targetName .. '\'):getID() .. ")")')
  rcom.sendACK(skt, 'AiTargetSet')
end

M.handleSetAiWaypoint = function(skt, msg)
  local targetName = msg['target']
  ai.setTarget(targetName)
  ai.stateChanged()
  rcom.sendACK(skt, 'AiWaypointSet')
end

M.handleSetAiSpan = function(skt, msg)
  if msg['span'] then
    ai.spanMap(0)
  else
    ai.setMode('disabled')
  end
  ai.stateChanged()
  rcom.sendACK(skt, 'AiSpanSet')
end

M.handleSetAiAggression = function(skt, msg)
  local aggr = msg['aggression']
  ai.setAggression(aggr)
  ai.stateChanged()
  rcom.sendACK(skt, 'AiAggressionSet')
end

M.handleSetDriveInLane = function(skt, msg)
  ai.driveInLane(msg['lane'])
  ai.stateChanged()
  rcom.sendACK(skt, 'AiDriveInLaneSet')
end

M.handleSetLights = function(skt, msg)
  local leftSignal = msg['leftSignal']
  local rightSignal = msg['rightSignal']
  local hazardSignal = msg['hazardSignal']
  local fogLights = msg['fogLights']
  local headLights = msg['headLights']
  local lightBar = msg['lightBar']

  local state = electrics.values

  if headLights ~= nil and state.lights_state ~= headLights then
    electrics.setLightsState(headLights)
  end

  if hazardSignal ~= nil then 
    if hazardSignal == true then
      hazardSignal = 1
    end
    if hazardSignal == false then
      hazardSignal = 0
    end
    if state.hazard_enabled ~= hazardSignal then
      leftSignal = nil
      rightSignal = nil
      electrics.toggle_warn_signal()
    end
  end

  if leftSignal ~= nil then
    if leftSignal == true then
      leftSignal = 1
    end
    if leftSignal == false then
      leftSignal = 0
    end
    if state.signal_left_input ~= leftSignal then
      electrics.toggle_left_signal()
    end
  end

  if rightSignal ~= nil then
    if rightSignal == true then
      rightSignal = 1
    end
    if rightSignal == false then
      rightSignal = 0 
    end
    if state.signal_right_input ~= rightSignal then
      electrics.toggle_right_signal()
    end
  end

  if fogLights ~= nil and state.fog ~= fogLights then
    electrics.set_fog_lights(fogLights)
  end

  if lightBar ~= nil then
    if state.lightbar ~= lightBar then
      electrics.set_lightbar_signal(lightBar)
    end
  end

  rcom.sendACK(skt, 'LightsSet')
end

M.handleQueueLuaCommandVE = function(skt, msg)
  local func, loading_err = load(msg.chunk)
  if func then 
    local status, err = pcall(func)
    if not status then
      log('E', 'execution error: "' .. err .. '"')
    end
  else
    log('E', 'compilation error in: "' .. msg.chunk .. '"')
  end
  rcom.sendACK(skt, 'ExecutedLuaChunkVE')
end

M.handleAddIMUPosition = function(skt, msg)
  local name = msg['name']
  local pos = msg['pos']
  pos = vec3(pos[1], pos[2], pos[3])
  local debug = msg['debug']

  if imu == nil then
    extensions.load('imu')
  end

  imu.addIMU(name, pos, debug)
  rcom.sendACK(skt, 'IMUPositionAdded')
end

M.handleAddIMUNode = function(skt, msg)
  local name = msg['name']
  local node = msg['node']
  local debug = msg['debug']

  if imu == nil then
    extensions.load('imu')
  end

  imu.addIMUAtNode(name, node, debug)
  rcom.sendACK(skt, 'IMUNodeAdded')
end

M.handleRemoveIMU = function(skt, msg)
  local imu = imu.removeIMU(msg['name'])
  if imu ~= nil then
    rcom.sendACK(skt, 'IMURemoved')
  else
    rcom.sendBNGValueError('Unknown IMU: ' .. tostring(msg['name']))
  end
end

M.handleApplyVSLSettingsFromJSON = function(skt, msg)
  extensions.vehicleStatsLogger.applySettingsFromJSON(msg['fileName'])
  rcom.sendACK(skt, 'AppliedVSLSettings')
end

M.handleWriteVSLSettingsToJSON = function(skt, msg)
  extensions.vehicleStatsLogger.writeSettingsToJSON(msg['fileName'])
  rcom.sendACK(skt, 'WroteVSLSettingsToJSON')
end

M.handleStartVSLLogging = function(skt, msg)
  extensions.vehicleStatsLogger.settings.outputDir = msg['outputDir']
  extensions.vehicleStatsLogger.startLogging()
  rcom.sendACK(skt, 'StartedVSLLogging')
end

M.handleStopVSLLogging = function(skt, msg)
  extensions.vehicleStatsLogger.stopLogging()
  rcom.sendACK(skt, 'StoppedVSLLogging')
end

return M
