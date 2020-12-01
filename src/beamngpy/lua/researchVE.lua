-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}
local logTag = 'ResearchVE'

local socket = require('libs/luasocket/socket.socket')
local rcom = require('utils/researchCommunication')

local sensorHandlers = {}

local skt = nil
local clients = {}

local host = nil
local port = nil

local conSleep = 60

local _log = log
local function log(level, message)
  _log(level, logTag, message)
end

local function checkMessage()
  local message, err = rcom.readMessage(clients)
  if err ~= nil then
    skt = nil
    clients = {}
    conSleep = 60
    return
  end

  if message ~= nil then
    local msgType = message['type']
    if msgType ~= nil then
      msgType = 'handle' .. msgType
      local handler = M[msgType]
      if handler ~= nil then
        handler(message)
      else
        extensions.hook('onSocketMessage', skt, message)
      end
    end
  end
end

local function connect()
  log('I', 'Trying to connect to: ' .. host .. ':' .. tostring(port))
  skt = socket.connect(host, port)
  if skt ~= nil then
    log('I', 'Connected!')
    table.insert(clients, skt)
  else
    log('I', 'Could not connect...')
  end
end

M.startConnecting = function(targetHost, targetPort)
  host = targetHost
  port = targetPort
end

M.onDebugDraw = function()
  if port ~= nil then
    if skt == nil then
      if conSleep <= 0 then
        conSleep = 60
        connect()
      else
        conSleep = conSleep - 1
      end
    end
  end

  if skt == nil then
    return
  end

  checkMessage()
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

-- Handlers

local function submitInput(inputs, key)
  local val = inputs[key]
  if val ~= nil then
    input.event(key, val, 1)
  end
end

M.handleControl = function(msg)
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

M.handleSetShiftMode = function(msg)
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

M.handleGetPartConfig = function(msg)
  local cfg = partmgmt.getConfig()
  local resp = {type = 'PartConfig', config = cfg}
  rcom.sendMessage(skt, resp)
end

M.handleGetPartOptions = function(msg)
  local options = v.data.slotMap
  local resp = {type = 'PartOptions', options = options}
  rcom.sendMessage(skt, resp)
end

M.handleSetPartConfig = function(msg)
  local cfg = msg['config']
  skt = nil
  log('I', 'Setting part config.')
  partmgmt.setConfig(cfg)
end

M.handleSensorRequest = function(msg)
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

M.handleSetColor = function(msg)
  local cmd = 'Point4F(' .. msg['r'] .. ', ' .. msg['g'] .. ', ' .. msg['b'] .. ', ' .. msg['a'] .. ')'
  cmd = 'be:getObjectByID(' .. obj:getID() .. '):setColor(' .. cmd .. ')'
  obj:queueGameEngineLua(cmd)
  rcom.sendACK(skt, 'ColorSet')
end

M.handleSetAiMode = function(msg)
  ai.setMode(msg['mode'])
  ai.stateChanged()
  rcom.sendACK(skt, 'AiModeSet')
end

M.handleSetAiLine = function(msg)
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

M.handleSetAiScript = function(msg)
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

M.handleSetAiSpeed = function(msg)
  ai.setSpeedMode(msg['mode'])
  ai.setSpeed(msg['speed'])
  ai.stateChanged()
  rcom.sendACK(skt, 'AiSpeedSet')
end

M.handleSetAiTarget = function(msg)
  local targetName = msg['target']
  obj:queueGameEngineLua('scenetree.findObjectById(' .. obj:getID() .. '):queueLuaCommand("ai.setTargetObjectID(" .. scenetree.findObject(\'' .. targetName .. '\'):getID() .. ")")')
  rcom.sendACK(skt, 'AiTargetSet')
end

M.handleSetAiWaypoint = function(msg)
  local targetName = msg['target']
  ai.setTarget(targetName)
  ai.stateChanged()
  rcom.sendACK(skt, 'AiWaypointSet')
end

M.handleSetAiSpan = function(msg)
  if msg['span'] then
    ai.spanMap(0)
  else
    ai.setMode('disabled')
  end
  ai.stateChanged()
  rcom.sendACK(skt, 'AiSpanSet')
end

M.handleSetAiAggression = function(msg)
  local aggr = msg['aggression']
  ai.setAggression(aggr)
  ai.stateChanged()
  rcom.sendACK(skt, 'AiAggressionSet')
end

M.handleSetDriveInLane = function(msg)
  ai.driveInLane(msg['lane'])
  ai.stateChanged()
  rcom.sendACK(skt, 'AiDriveInLaneSet')
end

M.handleSetLights = function(msg)
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

M.handleQueueLuaCommandVE = function(msg)
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

return M
