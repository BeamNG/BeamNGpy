-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}
local logTag = 'ResearchGE'

local mp = require('libs/lua-MessagePack/MessagePack')
local socket = require('libs/luasocket/socket.socket')
local rcom = require('utils/researchCommunication')

local scenariosLoader = require('scenario/scenariosLoader')
local scenarioHelper = require('scenario/scenariohelper')

local procPrimitives = require('util/trackBuilder/proceduralPrimitives')

local jbeamIO = require('jbeam/io')

local host = '127.0.0.1'
local port = 64256

local gameState = 'menu'

local quitRequested = false

local conSleep = 1
local stepsLeft = 0
local stepACK = false

local blocking = nil
local waiting = nil

local spawnPending = nil

local vehicleInfoPending = 0
local vehicleInfo = nil

local frameDelayTimer = -1
local frameDelayFunc = nil

local sensors = {}

local lidars = {}

local objectCount = 1

local debugLines = {}

local vehicleColors = {}

local server = nil
local clients = nil

local debugObjects = { spheres = {},
                       dynamicSpheres = {},
                       polylines = {},
                       cylinders = {},
                       triangles = {},
                       rectangles ={},
                       text = {},
                       squarePrisms = {}
                      }
local debugObjectCounter = {sphereNum = 0,
                            dynamicSphereNum = 0,
                            lineNum = 0,
                            cylinderNum = 0,
                            triangleNum = 0,
                            rectangleNum = 0,
                            textNum = 0,
                            prismNum = 0
                          }

local _log = log
local function log(level, message)
  _log(level, logTag, message)
end

local function addDynamicDebugSphere(getSpec)
  debugObjectCounter.dynamicSphereNum = debugObjectCounter.dynamicSphereNum + 1
  table.insert(debugObjects.dynamicSpheres, {getSpec = getSpec})
  return debugObjectCounter.dynamicSphereNum
end

local function generateVehicleColor(vid)
  local color = ColorI(math.ceil(255 * math.random()), math.ceil(255 * math.random()), math.ceil(255 * math.random()), 255)
  vehicleColors[vid] = color
end

local function block(reason, skt)
  blocking = reason
  waiting = skt
end

local function stopBlocking()
  blocking = nil
  waiting = nil
end

M.onPreRender = function(dt)
  if quitRequested then
    shutdown(0)
  end

  if frameDelayTimer > 0 then
    frameDelayTimer = frameDelayTimer - 1
    if frameDelayTimer == 0 then
      frameDelayFunc()
      frameDelayFunc = nil
      frameDelayTimer = 0
    else
      return
    end
  end

  if blocking ~= nil then
    if blocking == 'returnMainMenu' then
      if next(core_gamestate.state) == nil then
        rcom.sendACK(waiting, 'ScenarioStopped')
        stopBlocking()
        goto continue
      end
    end

    if blocking == 'step' then
      stepsLeft = stepsLeft - 1
      if stepsLeft == 0 then
        rcom.sendACK(waiting, 'Stepped')
        stopBlocking()
        goto continue
      end
    end

    return
  end

  ::continue::

  if server ~= nil then
    if conSleep <= 0 then
      conSleep = 1
      local newClients = rcom.checkForClients(server)
      for i = 1, #newClients do
        clients:insert(newClients[i])
        local ip, clientPort = newClients[i]:getsockname()
        log('I', 'Accepted new client: ' .. tostring(ip) .. '/' .. tostring(clientPort))
      end
    else
      conSleep = conSleep - dt
    end
  else
    return
  end

  while rcom.checkMessages(M, clients) do end
end

M.onScenarioUIReady = function(state)
  if state == 'start' and blocking == 'loadScenario' then
    rcom.sendACK(waiting, 'MapLoaded')
    stopBlocking()
  end
end

M.onCountdownEnded = function()
  if blocking == 'startScenario' then
    rcom.sendACK(waiting, 'ScenarioStarted')
    stopBlocking()
  end
end

M.onScenarioRestarted = function()
  if blocking == 'restartScenario' then
    scenario_scenarios.changeState('running')
    scenario_scenarios.getScenario().showCountdown = false
    scenario_scenarios.getScenario().countDownTime = 0
    guihooks.trigger('ChangeState', 'menu')
    rcom.sendACK(waiting, 'ScenarioRestarted')
    stopBlocking()
  end
end

M.onVehicleConnectionReady = function(vehicleID, port)
  log('I', 'New vehicle connection: ' .. tostring(vehicleID) .. ', ' .. tostring(port))
  if blocking == 'vehicleConnection' then
    local name = ''
    local veh = scenetree.findObjectById(vehicleID)
    if veh ~= nil then
      name = veh:getName()
    end
    if name == '' then
      name = tostring(vehicleID)
    end
    local resp = {type = 'StartVehicleConnection', vid = name, result = port}
    rcom.sendMessage(waiting, resp)
    stopBlocking()
  end
end

M.onVehicleInfoReady = function(vehicleID, info)
  if blocking == 'vehicleInfo' then
    local current = vehicleInfo[vehicleID]
    current['port'] = info.port
    vehicleInfoPending = vehicleInfoPending - 1

    if vehicleInfoPending == 0 then
      local resp = {}
      for k, v in pairs(vehicleInfo) do
        resp[v.name] = v
      end
      resp = {type = 'GetCurrentVehicles', result = resp}
      rcom.sendMessage(waiting, resp)
      vehicleInfo = nil
      stopBlocking()
    end
  end
end

local function setup()
  settings.setValue('uiUnits', 'metric')
  settings.setValue('uiUnitLength', 'metric')
  settings.setValue('uiUnitTemperature', 'c')
  settings.setValue('uiUnitWeight', 'kg')
  settings.setValue('uiUnitTorque', 'metric')
  settings.setValue('uiUnitConsumptionRate', 'metric')
  settings.setValue('uiUnitEnergy', 'metric')
  settings.setValue('uiUnitDate', 'iso')
  settings.setValue('uiUnitPower', 'hp')
  settings.setValue('uiUnitVolume', 'l')
  settings.setValue('uiUnitPressure', 'bar')

  extensions.load('util/partAnnotations')

  if server == nil then
    server = rcom.newSet()
    server:insert(rcom.openServer(port))
    clients = rcom.newSet()
  end
end

M.onInit = function()
  local cmdArgs = Engine.getStartingArgs()
  for i, v in ipairs(cmdArgs) do
    if v == "-rport" then
      port = tonumber(cmdArgs[i + 1])
      setup()
    end
  end
end

M.startConnection = function(p)
  port = p
  setup()
end

M.notifyUI = function()
  local state = {}
  if server ~= nil then
    state.running = true
    state.port = port
  else
    state.running = false
  end
  guihooks.trigger('BeamNGpyExtensionReady', state)
end

-- Handlers

M.handleHello = function(skt, msg)
  local resp = {type = 'Hello', protocolVersion = rcom.protocolVersion}
  rcom.sendMessage(skt, resp)
end

M.handleQuit = function(skt, msg)
  rcom.sendACK(skt, 'Quit')
  quitRequested = true
  blocking = 'quit'
end

M.handleLoadScenario = function(skt, msg)
  local scenarioPath = msg["path"]
  log('I', 'Loading scenario: '..scenarioPath)
  local ret = scenariosLoader.startByPath(scenarioPath)
  if ret then
    log('I', 'Scenario found...')
    block('loadScenario', skt)
  else
    log('I', 'Scenario not found...')
    rcom.sendBNGValueError(skt, 'Scenario not found: "' .. scenarioPath .. '"')
  end
  return true
end

M.handleStartScenario = function(skt, msg)
  scenario_scenarios.changeState("running")
  scenario_scenarios.getScenario().showCountdown = false
  scenario_scenarios.getScenario().countDownTime = 0
  guihooks.trigger("ChangeState", "menu")
  block('startScenario', skt)
  return true
end

M.handleRestartScenario = function(skt, msg)
  scenario_scenarios.restartScenario()
  block('restartScenario', skt)
  return true
end

M.handleStopScenario = function(skt, msg)
  returnToMainMenu()
  block('returnMainMenu', skt)
  return true
end

M.handleGetScenarioName = function(skt, msg)
  local name = scenario_scenarios.getscenarioName()
  local resp = {type = 'ScenarioName', name = name}
  rcom.sendMessage(skt, resp)
end

M.handleHideHUD = function(skt, msg)
  be:executeJS('document.body.style.opacity = "0.0";')
end

M.handleShowHUD = function(skt, msg)
  be:executeJS('document.body.style.opacity = "1.0";')
end

M.handleSetPhysicsDeterministic = function(skt, msg)
  be:setPhysicsSpeedFactor(-1)
  rcom.sendACK(skt, 'SetPhysicsDeterministic')
end

M.handleSetPhysicsNonDeterministic = function(skt, msg)
  be:setPhysicsSpeedFactor(0)
  rcom.sendACK(skt, 'SetPhysicsNonDeterministic')
end

M.handleFPSLimit = function(skt, msg)
  settings.setValue('FPSLimiter', msg['fps'], true)
  settings.setState({FPSLimiterEnabled = true}, true)
  rcom.sendACK(skt, 'SetFPSLimit')
end

M.handleRemoveFPSLimit = function(skt, msg)
  settings.setState({FPSLimiterEnabled = false}, true)
  rcom.sendACK(skt, 'RemovedFPSLimit')
end

M.handlePause = function(skt, msg)
  be:setPhysicsRunning(false)
  rcom.sendACK(skt, 'Paused')
end

M.handleResume = function(skt, msg)
  be:setPhysicsRunning(true)
  rcom.sendACK(skt, 'Resumed')
end

M.handleStep = function(skt, msg)
  local count = msg["count"]
  stepsLeft = count
  block('step', skt)
  be:physicsStep(count)
  return true
end

M.handleTeleport = function(skt, msg)
  local vID = msg['vehicle']
  local veh = scenarioHelper.getVehicleByName(vID)
  if msg['rot'] ~= nil then
    local quat = quat(msg['rot'][1], msg['rot'][2], msg['rot'][3], msg['rot'][4])
    veh:setPositionRotation(msg['pos'][1], msg['pos'][2], msg['pos'][3], quat.x, quat.y, quat.z, quat.w)
  else
    veh:setPosition(Point3F(msg['pos'][1], msg['pos'][2], msg['pos'][3]))
  end
  rcom.sendACK(skt, 'Teleported')
end

M.handleTeleportScenarioObject = function(skt, msg)
  local sobj = scenetree.findObject(msg['id'])
  if msg['rot'] ~= nil then
    local quat = quat(msg['rot'][1], msg['rot'][2], msg['rot'][3], msg['rot'][4])
    sobj:setPosRot(msg['pos'][1], msg['pos'][2], msg['pos'][3], quat.x, quat.y, quat.z, quat.w)
  else
    sobj:setPosition(Point3F(msg['pos'][1], msg['pos'][2], msg['pos'][3]))
  end
  rcom.sendACK(skt, 'ScenarioObjectTeleported')
end

M.handleStartVehicleConnection = function(skt, msg)
  local vid, veh, command

  vid = msg['vid']

  command = 'extensions.load("researchVE")'
  veh = scenarioHelper.getVehicleByName(vid)
  veh:queueLuaCommand(command)

  local exts = msg['exts']
  if exts then
    for idx, ext in pairs(exts) do
      command = 'extensions.load("' .. ext .. '")'
      veh:queueLuaCommand(command)
    end
  end

  block('vehicleConnection', skt)

  command = 'researchVE.startConnection()'
  veh:queueLuaCommand(command)
  return true
end

M.handleOpenShmem = function(skt, msg)
  local name = msg['name']
  local size = msg['size']
  Engine.openShmem(name, size)
  rcom.sendACK(skt, 'OpenedShmem')
end

M.handleCloseShmem = function(skt, msg)
  local name = msg['name']
  Engine.closeShmem(name)
  rcom.sendACK(skt, 'ClosedShmem')
end

M.handleWaitForSpawn = function(skt, msg)
  local name = msg['name']
  spawnPending = name
  block('spawnVehicle', skt)
end

M.onVehicleSpawned = function(vID)
  if blocking == 'spawnVehicle' and spawnPending ~= nil then
    local obj = scenetree.findObject(spawnPending)
    log('I', 'Vehicle spawned: ' .. tostring(vID))
    if obj ~= nil and obj:getID() == vID then
      local resp = {type = 'VehicleSpawned', name = spawnPending}
      spawnPending = nil
      rcom.sendMessage(waiting, resp)
      stopBlocking()
    end
  end
end

M.handleSpawnVehicle = function(skt, msg)
  local name = msg['name']
  local model = msg['model']
  local pos = msg['pos']
  local rot = msg['rot']
  local cling = msg['cling']

  pos = vec3(pos[1], pos[2], pos[3])
  rot = quat(rot)

  local partConfig = msg['partConfig']

  local options = {}
  options.config = partConfig
  options.pos = pos
  options.rot = rot
  options.cling = cling
  options.vehicleName = name
  options.color = msg['color']
  options.color2 = msg['color2']
  options.color3 = msg['color3']
  options.licenseText = msg['licenseText']

  spawnPending = name
  block('spawnVehicle', skt)

  core_vehicles.spawnNewVehicle(model, options)
end

M.handleDespawnVehicle = function(skt, msg)
  local name = msg['vid']
  local veh = scenetree.findObject(name)
  if veh ~= nil then
    veh:delete()
  end
  rcom.sendACK(skt, 'VehicleDespawned')
end

local function setVehicleAnnotationColor(veh, color)
  local meshes = veh:getMeshNames()
  for i = 1, #meshes do
    veh:setMeshAnnotationColor(meshes[i], color)
  end
end

local function lidarsVisualized(state)
  for l, lidar in pairs(lidars) do
    lidar:visualized(state)
  end
end

local function getVehiclePosRot(vid)
  local veh = scenarioHelper.getVehicleByName(vid)
  local rot = vec3(veh:getDirectionVector())
  local up = vec3(veh:getDirectionVectorUp())
  local rot = quatFromDir(rot, up)
  local position = vec3(veh:getPosition())
  return {pos = position, rot = rot}
end

sensors.Camera = function(req, callback)
  lidarsVisualized(false)

  local offset, orientation, up
  local pos, direction, rot, fov, resolution, nearFar, vehicle, vehicleObj, data
  local color, depth, annotation, instance
  local paused = false

  color = req['color']
  depth = req['depth']
  annotation = req['annotation']
  instance = req['instance']

  local shmem = req['shmem']

  if instance ~= nil and be:getPhysicsRunning() then
    be:setPhysicsRunning(false)
    paused = true
  end

  if req['vehicle'] then
    local veh = getVehiclePosRot(req['vehicle'])
    orientation = veh.rot
    offset = veh.pos
  else
    orientation = quatFromEuler(0, 0, 0)
    offset = vec3(0, 0, 0)
  end

  direction = req['direction']
  direction = vec3(direction[1], direction[2], direction[3])

  fov = math.rad(req['fov'])

  resolution = req['resolution']
  nearFar = req['near_far']

  rot = quatFromDir(direction, vec3(0, 0, 1)) * orientation

  pos = req['pos']
  pos = vec3(pos[1], pos[2], pos[3])
  if req['vehicle'] then
    pos = offset + orientation * pos
  else
    pos = offset + pos
  end
  pos = Point3F(pos.x, pos.y, pos.z)

  rot = QuatF(rot.x, rot.y, rot.z, rot.w)

  resolution = Point2F(resolution[1], resolution[2])
  nearFar = Point2F(nearFar[1], nearFar[2])

  local data = nil
  if shmem then
    data = Engine.renderCameraShmem(color, depth, annotation, pos, rot, resolution, fov, nearFar)
  else
    data = Engine.renderCameraBase64Blocking(pos, rot, resolution, fov, nearFar)
  end

  if instance ~= nil then
    AnnotationManager.setInstanceAnnotations(true)

    for k, v in pairs(map.objects) do
      local veh = scenetree.findObject(k)
      if vehicleColors[k] == nil then
        generateVehicleColor(k)
      end

      local color = vehicleColors[k]
      setVehicleAnnotationColor(veh, color)
    end

    frameDelayTimer = 1
    frameDelayFunc = function()
      local otherData = nil
      if shmem then
        otherData = Engine.renderCameraShmem(color, depth, instance, pos, rot, resolution, fov, nearFar)
        otherData['instance'] = otherData['annotation']
        otherData['annotation'] = data['annotation']
      else
        otherData = Engine.renderCameraBase64Blocking(pos, rot, resolution, fov, nearFar)
        otherData['instanceRGB8'] = otherData['annotationRGB8']
        otherData['annotationRGB8'] = data['annotationRGB8']
      end
      AnnotationManager.setInstanceAnnotations(false)

      for k, v in pairs(map.objects) do
        local veh = scenetree.findObject(k)
        local color = vehicleColors[k]
        setVehicleAnnotationColor(veh, ColorI(0, 255, 0, 255))
      end

      if paused then
        be:setPhysicsRunning(true)
      end

      lidarsVisualized(true)
      callback(otherData)
    end
  else
    lidarsVisualized(true)
    callback(data)
  end

end

sensors.Lidar = function(req, callback)
  local name = req['name']
  log('I', 'Getting lidar data! ' .. tostring(name))
  local shmem = req['shmem']
  local lidar = lidars[name]
  if lidar ~= nil then
    if shmem then
      lidar:requestDataShmem(function(realSize)
        log('I', 'Got data shmem!')
        callback({size = realSize})
      end)
    else
      lidar:requestData(function(points)
        log('I', 'Got data points!')
        local res = {}
        for i, p in ipairs(points) do
          table.insert(res, tonumber(p.x))
          table.insert(res, tonumber(p.y))
          table.insert(res, tonumber(p.z))
        end
        log('I', 'Sending points!')
        callback({points = res})
      end)
    end
  else
    callback(nil)
  end
end

sensors.Timer = function(req, callback)
  callback({time = scenario_scenarios.getScenario().timer})
end

local function computeAbsoluteSensorPosRot(vid, sensorPosRelToVeh, sensorRotRelToVeh)
  -- rot
  local veh = getVehiclePosRot(vid)
  local vehicleRot = veh.rot
  local vehiclePos = veh.pos

  local sensorPos = vehiclePos + vehicleRot * sensorPosRelToVeh
  local sensorRot = quatFromDir(sensorRotRelToVeh, vec3(0, 0, 1)) * vehicleRot

  return sensorPos, sensorRot
end

sensors.Ultrasonic = function(req, sendSensorData)
  local sensorPos, sensorRot

  local sensorOffset = req['pos']
  sensorOffset = vec3(sensorOffset[1], sensorOffset[2], sensorOffset[3])

  sensorRot = req['rot']
  sensorRot = vec3(sensorRot[1], sensorRot[2], sensorRot[3])
  if req['vehicle'] then
    local pos, rot = computeAbsoluteSensorPosRot(req['vehicle'], sensorOffset, sensorRot)
    sensorPos = Point3F(pos.x, pos.y, pos.z)
    sensorRot = QuatF(rot.x, rot.y, rot.z, rot.w)
  else
    sensorPos = Point3F(sensorPos.x, sensorPos.y, sensorPos.z)
    sensorRot = quatFromDir(sensorRot, vec3(0, 0, 1))
    sensorRot = QuatF(sensorRot.x, sensorRot.y, sensorRot.z, sensorRot.w)
  end

  local fov = math.rad(req['fov'])

  local resolution = req['resolution']
  resolution = Point2F(resolution[1], resolution[2])

  local near_far = req['near_far']
  near_far = Point2F(near_far[1], near_far[2])

  local dist = Engine.getUltrasonicDistanceMeasurement(sensorPos, sensorRot, resolution, fov, near_far)
  local measurement = {distance = dist}
  sendSensorData(measurement)

  -- commands.setFreeCamera()
  -- setCameraPosRot(sensorPos.x, sensorPos.y, sensorPos.z, sensorRot.x, sensorRot.y, sensorRot.z, sensorRot.w)
end

M.handleStartUSSensorVisualization = function(skt, msg)
  local sensorOffset = msg['pos']
  sensorOffset = vec3(sensorOffset[1], sensorOffset[2], sensorOffset[3])
  local sensorRot = msg['rot']
  sensorRot = vec3(sensorRot[1], sensorRot[2], sensorRot[3])
  local color = msg.color
  color = ColorF(color[1], color[2], color[3], color[4])
  local radius = msg['radius']
  local sphereCallback = function()
    local pos, rot = computeAbsoluteSensorPosRot(msg['vehicle'], sensorOffset, sensorRot)
    local coo = Point3F(pos.x, pos.y, pos.z)
    return {coo=coo, radius=radius, color=color}
  end
  local sphereID = addDynamicDebugSphere(sphereCallback)
  local response = {sphereID = sphereID}
  rcom.sendMessage(skt, response)
end

M.handleStopUSSensorVisualization = function(skt, msg)
  table.remove(debugObjects.dynamicSpheres, msg['dynSphereID'])
end

local function getSensorData(request, callback)
  local response, sensor_type, handler
  sensor_type = request['type']
  handler = sensors[sensor_type]
  if handler ~= nil then
    handler(request, callback)
  else
    callback(nil)
  end
end

local function getNextSensorData(requests, response, callback)
  local key = next(requests)
  if key == nil then
    callback(response)
    return
  end

  local request = requests[key]
  requests[key] = nil

  local cb = function(data)
    response[key] = data
    getNextSensorData(requests, response, callback)
  end

  getSensorData(request, cb)
end

M.handleSensorRequest = function(skt, msg)
  local requests

  local cb = function(response)
    response = {type = 'SensorData', data = response}
    rcom.sendMessage(skt, response)
  end

  requests = msg['sensors']

  getNextSensorData(requests, {}, cb)
  return true
end

M.handleGetDecalRoadVertices = function(skt, msg)
  local response = Sim.getDecalRoadVertices()
  response = {type = "DecalRoadVertices", vertices = response}
  rcom.sendMessage(skt, response)
end

M.handleGetDecalRoadData = function(skt, msg)
  local resp = {type = 'DecalRoadData'}
  local data = {}
  local roads = scenetree.findClassObjects('DecalRoad')
  for idx, roadID in ipairs(roads) do
    local road = scenetree.findObject(roadID)
    local roadData = {
      drivability = road:getField('drivability', ''),
      lanesLeft = road:getField('lanesLeft', ''),
      lanesRight = road:getField('lanesRight', ''),
      oneWay = road:getField('oneWay', '') ~= nil,
      flipDirection = road:getField('flipDirection', '') ~= nil
    }
    data[roadID] = roadData
  end
  resp['data'] = data
  rcom.sendMessage(skt, resp)
end

M.handleGetDecalRoadEdges = function(skt, msg)
  local roadID = msg['road']
  local response = {type = 'DecalRoadEdges'}
  local road = scenetree.findObject(roadID)
  local edges = {}
  for i, e in ipairs(road:getEdgesTable()) do
    local edge = {
      left = {
        e[1].x,
        e[1].y,
        e[1].z
      },
      middle = {
        e[2].x,
        e[2].y,
        e[2].z
      },
      right = {
        e[3].x,
        e[3].y,
        e[3].z
      }
    }
    table.insert(edges, edge)
  end
  response['edges'] = edges
  rcom.sendMessage(skt, response)
end

M.handleEngineFlags = function(skt, msg)
  log('I', 'Setting engine flags.')
  local flags = msg['flags']
  if flags['annotations'] then
    Engine.Annotation.enable(true)
  end

  rcom.sendACK(skt, 'SetEngineFlags')
end

M.handleTimeOfDayChange = function(skt, msg)
  core_environment.setTimeOfDay({time = msg['tod']})
  rcom.sendACK(skt, 'TimeOfDayChanged')
end

local function getVehicleState(vid)
  local vehicle = scenetree.findObject(vid)
  local state = {
    pos = vehicle:getPosition(),
    dir = vehicle:getDirectionVector(),
    up = vehicle:getDirectionVectorUp(),
    vel = vehicle:getVelocity()
  }
  state['pos'] = {
    state['pos'].x,
    state['pos'].y,
    state['pos'].z
  }

  state['dir'] = {
    state['dir'].x,
    state['dir'].y,
    state['dir'].z
  }

  state['up'] = {
    state['up'].x,
    state['up'].y,
    state['up'].z
  }

  state['vel'] = {
    state['vel'].x,
    state['vel'].y,
    state['vel'].z
  }

  return state
end

M.handleUpdateScenario = function(skt, msg)
  local response = {type = 'ScenarioUpdate'}
  local vehicleStates = {}
  for idx, vid in ipairs(msg['vehicles']) do
    vehicleStates[vid] = getVehicleState(vid)
  end
  response['vehicles'] = vehicleStates

  rcom.sendMessage(skt, response)
end

M.handleOpenLidar = function(skt, msg)
  local name = msg['name']
  local shmem = msg['shmem']
  local shmemSize = msg['size']
  local vid = msg['vid']
  local vid = scenetree.findObject(vid):getID()
  local vRes = msg['vRes']
  local vAngle = math.rad(msg['vAngle'])
  local rps = msg['rps']
  local hz = msg['hz']
  local angle = math.rad(msg['angle'])
  local maxDist = msg['maxDist']
  local offset = msg['offset']
  offset = Point3F(offset[1], offset[2], offset[3])
  local direction = msg['direction']
  direction = Point3F(direction[1], direction[2], direction[3])

  local lidar = research.LIDAR(vid, offset, direction, vRes, vAngle, rps, hz, angle, maxDist)
  lidar:open(shmem, shmemSize)
  lidar:enabled(true)
  lidar:visualized(msg['visualized'])
  lidars[name] = lidar

  rcom.sendACK(skt, 'OpenedLidar')
end

M.handleCloseLidar = function(skt, msg)
  local name = msg['name']
  local lidar = lidars[name]
  if lidar ~= nil then
    -- lidar:close()
    lidars[name] = nil
  end
  rcom.sendACK(skt, 'ClosedLidar')
end

M.handleSetWeatherPreset = function(skt, msg)
  local preset = msg['preset']
  local time = msg['time']
  core_weather.switchWeather(preset, time)
  rcom.sendACK(skt, 'WeatherPresetChanged')
end

M.handleGameStateRequest = function(skt, msg)
  local state = core_gamestate.state.state
  resp = {type = 'GameState'}
  if state == 'scenario' then
    resp['state'] = 'scenario'
    resp['scenario_state'] = scenario_scenarios.getScenario().state
  else
    resp['state'] = 'menu'
  end
  rcom.sendMessage(skt, resp)
end

M.handleDisplayGuiMessage = function(skt, msg)
  local message = msg['message']
  guihooks.message(message)
  rcom.sendACK(skt, 'GuiMessageDisplayed')
end

M.handleSwitchVehicle = function(skt, msg)
  local vID = msg['vid']
  local vehicle = scenetree.findObject(vID)
  be:enterVehicle(0, vehicle)
  rcom.sendACK(skt, 'VehicleSwitched')
end

M.handleSetFreeCamera = function(skt, msg)
  local pos = msg['pos']
  local direction = msg['dir']
  local rot = quatFromDir(vec3(direction[1], direction[2], direction[3]))

  commands.setFreeCamera()
  commands.setCameraPosRot(pos[1], pos[2], pos[3], rot.x, rot.y, rot.z, rot.w)
  rcom.sendACK(skt, 'FreeCameraSet')
end

M.handleParticlesEnabled = function(skt, msg)
  local enabled = msg['enabled']
  Engine.Render.ParticleMgr.setEnabled(enabled)
  rcom.sendACK(skt, 'ParticlesSet')
end

M.handleAnnotateParts = function(skt, msg)
  local vehicle = scenetree.findObject(msg['vid'])
  util_partAnnotations.annotateParts(vehicle:getID())
  rcom.sendACK(skt, 'PartsAnnotated')
end

M.handleRevertAnnotations = function(skt, msg)
  local vehicle = scenetree.findObject(msg['vid'])
  util_partAnnotations.revertAnnotations(vehicle:getID())
  rcom.sendACK(skt, 'AnnotationsReverted')
end

M.handleGetPartAnnotations = function(skt, msg)
  local vehicle = scenetree.findObject(msg['vid'])
  local colors = util_partAnnotations.getPartAnnotations(vehicle:getID())
  local converted = {}
  for key, val in pairs(colors) do
    converted[key] = {val.r, val.g, val.b}
  end
  rcom.sendMessage(skt, {type = 'PartAnnotations', colors = converted})
end

M.handleGetPartAnnotation = function(skt, msg)
  local part = msg['part']
  local color = util_partAnnotations.getPartAnnotation(part)
  if color ~= nil then
    color = {color.r, color.g, color.b}
  end
  rcom.sendMessage(skt, {type = 'PartAnnotation', color = color})
end

M.handleGetAnnotations = function(skt, msg)
  local annotations = AnnotationManager.getAnnotations()
  for k, v in pairs(annotations) do
    annotations[k] = {v.r, v.g, v.b}
  end
  local ret = {type = 'Annotations', annotations = annotations}
  rcom.sendMessage(skt, ret)
end

M.handleFindObjectsClass = function(skt, msg)
  local clazz = msg['class']
  local objects = scenetree.findClassObjects(clazz)
  local resp = {type='ClassObjects'}
  local list = {}
  for idx, object in ipairs(objects) do
    object = scenetree.findObject(object)

    local obj = {type=clazz, id=object:getID(), name=object:getName()}

    local scl = object:getScale()
    local pos = object:getPosition()
    local rot = object:getRotation()
    if clazz == 'BeamNGVehicle' then
      local vehicleData = map.objects[obj.id]
      rot = quatFromDir(vehicleData.dirVec, vehicleData.dirVecUp)
    end

    pos = {pos.x, pos.y, pos.z}
    rot ={rot.x, rot.y, rot.z, rot.w}

    scl = {scl.x, scl.y, scl.z}

    obj['position'] = pos
    obj['rotation'] = rot
    obj['scale'] = scl

    obj['options'] = {}
    for fld, nfo in pairs(object:getFieldList()) do
      if fld ~= 'position' and fld ~= 'rotation' and fld ~= 'scale' and fld ~= 'id' and fld ~= 'type' and fld ~= 'name' then
        local val = object:getField(fld, '')
        obj['options'][fld] = val
      end
    end

    table.insert(list, obj)
  end
  resp['objects'] = list
  rcom.sendMessage(skt, resp)
end

M.handleGetDecalRoadVertices = function(skt, msg)
  local response = Sim.getDecalRoadVertices()
  response = {type = "DecalRoadVertices", vertices = response}
  rcom.sendMessage(skt, response)
end

local function placeObject(name, mesh, pos, rot)
  if name == nil then
    name = 'procObj' .. tostring(objectCount)
    objectCount = objectCount + 1
  end

  pos = vec3(pos)
  rot = quat(rot):toTorqueQuat()

  local proc = createObject('ProceduralMesh')
  proc:registerObject(name)
  proc.canSave = false
  scenetree.MissionGroup:add(proc.obj)
  proc:createMesh({{mesh}})
  proc:setPosition(pos:toPoint3F())
  proc:setField('rotation', 0, rot.x .. ' ' .. rot.y .. ' ' .. rot.z .. ' ' .. rot.w)
  proc.scale = Point3F(1, 1, 1)

  be:reloadCollision()

  return proc
end

M.handleCreateCylinder = function(skt, msg)
  local name = msg['name']
  local radius = msg['radius']
  local height = msg['height']
  local material = msg['material']
  local pos = msg['pos']
  local rot = msg['rot']

  local cylinder = procPrimitives.createCylinder(radius, height, material)
  placeObject(name, cylinder, pos, rot)

  rcom.sendACK(skt, 'CreatedCylinder')
end

M.handleCreateBump = function(skt, msg)
  local name = msg['name']
  local length = msg['length']
  local width = msg['width']
  local height = msg['height']
  local upperLength = msg['upperLength']
  local upperWidth = msg['upperWidth']
  local material = msg['material']
  local pos = msg['pos']
  local rot = msg['rot']

  local bump = procPrimitives.createBump(length, width, height, upperLength, upperWidth, material)
  placeObject(name, bump, pos, rot)

  rcom.sendACK(skt, 'CreatedBump')
end

M.handleCreateCone = function(skt, msg)
  local name = msg['name']
  local radius = msg['radius']
  local height = msg['height']
  local material = msg['material']
  local pos = msg['pos']
  local rot = msg['rot']

  local cone = procPrimitives.createCone(radius, height, material)
  placeObject(name, cone, pos, rot)

  rcom.sendACK(skt, 'CreatedCone')
end

M.handleCreateCube = function(skt, msg)
  local name = msg['name']
  local size = vec3(msg['size'])
  local material = msg['material']
  local pos = msg['pos']
  local rot = msg['rot']

  local cube = procPrimitives.createCube(size, material)
  placeObject(name, cube, pos, rot)

  rcom.sendACK(skt, 'CreatedCube')
end

M.handleCreateRing = function(skt, msg)
  local name = msg['name']
  local radius = msg['radius']
  local thickness = msg['thickness']
  local material = msg['material']
  local pos = msg['pos']
  local rot = msg['rot']

  local ring = procPrimitives.createRing(radius, thickness, material)
  placeObject(name, ring, pos, rot)

  rcom.sendACK(skt, 'CreatedRing')
end

M.handleGetBBoxCorners = function(skt, msg)
  local veh = scenetree.findObject(msg['vid'])
  local resp = {type = 'BBoxCorners'}
  local points = {}
  local bbox = veh:getSpawnWorldOOBB()
  for i = 0, 7 do
    local point = bbox:getPoint(i)
    point = {tonumber(point.x), tonumber(point.y), tonumber(point.z)}
    table.insert(points, point)
  end
  resp['points'] = points
  rcom.sendMessage(skt, resp)
end

M.handleSetGravity = function(skt, msg)
  local gravity = msg['gravity']
  core_environment.setGravity(gravity)
  rcom.sendACK(skt, 'GravitySet')
end

M.handleGetAvailableVehicles = function(skt, msg)
  local resp = {type = 'AvailableVehicles', vehicles = {}}

  local models = core_vehicles.getModelList().models
  local configs = core_vehicles.getConfigList().configs

  for model, modelData in pairs(models) do
    local data = {
      author = modelData.Author,
      name = modelData.Name,
      type = modelData.Type,
      key = modelData.key,
    }
    data.configurations = {}
    for key, config in pairs(configs) do
      if config.model_key == model then
        data.configurations[config.key] = {
          author = config.Author,
          model_key = config.model_key,
          key = config.key,
          name = config.Name,
          type = config.Type
        }
      end
    end
    resp.vehicles[model] = data
  end

  rcom.sendMessage(skt, resp)
end

M.handleStartTraffic = function(skt, msg)
  local participants = msg.participants
  local ids = {}
  for idx, participant in ipairs(participants) do
    local veh = scenetree.findObject(participant)

    if veh == nil then
      rcom.sendBNGValueError(skt, 'Vehicle not present for traffic: ' .. tostring(participant))
    end

    table.insert(ids, veh:getID())
  end

  gameplay_traffic.activate(ids)
  rcom.sendACK(skt, 'TrafficStarted')
end

M.handleStopTraffic = function(skt, msg)
  local stop = msg.stop
  gameplay_traffic.deactivate(stop)
  rcom.sendACK(skt, 'TrafficStopped')
end

M.handleChangeSetting = function(skt, msg)
  local key = msg['key']
  local value = msg['value']
  settings.setValue(key, value, true)
  rcom.sendACK(skt, 'SettingsChanged')
end

M.handleApplyGraphicsSetting = function(skt, msg)
  core_settings_graphic.applyGraphicsState()
  rcom.sendACK(skt, 'GraphicsSettingApplied')
end

M.handleSetRelativeCam = function(skt, msg)
  core_camera.setByName(0, 'relative', false, {})

  local vid = be:getPlayerVehicle(0):getID()
  local pos = msg['pos']
  local rot = msg['rot']
  frameDelayTimer = 3
  frameDelayFunc = function()
    pos = vec3(pos[1], pos[2], pos[3])
    core_camera.getCameraDataById(vid)['relative'].pos = pos

    if rot ~= nil then
      rot = quat(rot[1], rot[2], rot[3], rot[4]):toEulerYXZ()
      core_camera.getCameraDataById(vid)['relative'].rot = rot
    end

    rcom.sendACK(skt, 'RelativeCamSet')
  end
  return false
end

local function tableToPoint3F(point, cling, offset)
  local point = Point3F(point[1], point[2], point[3])
  if cling then
    local z = be:getSurfaceHeightBelow(point)
    point = Point3F(point.x, point.y, z+offset)
  end
  return point
end

M.handleAddDebugSpheres = function(skt, msg)
  local sphereIDs = {}
  for idx = 1,#msg.radii do
    local coo = tableToPoint3F(msg.coordinates[idx], msg.cling, msg.offset)
    local color = msg.colors[idx]
    color = ColorF(color[1], color[2], color[3], color[4])
    local sphere = {coo = coo, radius = msg.radii[idx], color = color}
    debugObjectCounter.sphereNum = debugObjectCounter.sphereNum + 1
    debugObjects.spheres[debugObjectCounter.sphereNum] = sphere
    table.insert(sphereIDs, debugObjectCounter.sphereNum)
  end
  local resp = {type = 'DebugSphereAdded', sphereIDs = sphereIDs}
  rcom.sendMessage(skt, resp)
end

M.handleRemoveDebugObjects = function(skt, msg)
  for _, idx in pairs(msg.objIDs) do
    debugObjects[msg.objType][idx] = nil
  end
  rcom.sendACK(skt, 'DebugObjectsRemoved')
end

M.handleAddDebugPolyline = function(skt, msg)
  local polyline = {segments = {}}
  polyline.color = ColorF(msg.color[1], msg.color[2], msg.color[3], msg.color[4])
  local origin = tableToPoint3F(msg.coordinates[1], msg.cling, msg.offset)
  for i = 2, #msg.coordinates do
    local target = tableToPoint3F(msg.coordinates[i], msg.cling, msg.offset)
    local segment = {origin = origin, target = target}
    table.insert(polyline.segments, segment)
    origin = target
  end
  debugObjectCounter.lineNum = debugObjectCounter.lineNum + 1
  table.insert(debugObjects.polylines, debugObjectCounter.lineNum, polyline)
  local resp = {type = 'DebugPolylineAdded', lineID = debugObjectCounter.lineNum}
  rcom.sendMessage(skt, resp)
end

M.handleAddDebugCylinder = function(skt, msg)
  local circleAPos = tableToPoint3F(msg.circlePositions[1], false, 0)
  local circleBPos = tableToPoint3F(msg.circlePositions[2], false, 0)
  local color = ColorF(msg.color[1], msg.color[2], msg.color[3], msg.color[4])
  local cylinder = {circleAPos=circleAPos, circleBPos=circleBPos, radius=msg.radius, color=color}
  debugObjectCounter.cylinderNum = debugObjectCounter.cylinderNum + 1
  table.insert(debugObjects.cylinders, debugObjectCounter.cylinderNum, cylinder)
  local resp = {type='DebugCylinderAdded', cylinderID=debugObjectCounter.cylinderNum}
  rcom.sendMessage(skt, resp)
end

M.handleAddDebugTriangle = function(skt, msg)
  local color = msg.color
  color = ColorI(math.ceil(color[1]*255), math.ceil(color[2]*255), math.ceil(color[3]*255), math.ceil(color[4]*255))
  local pointA = tableToPoint3F(msg.vertices[1], msg.cling, msg.offset)
  local pointB = tableToPoint3F(msg.vertices[2], msg.cling, msg.offset)
  local pointC = tableToPoint3F(msg.vertices[3], msg.cling, msg.offset)
  local triangle = {a=pointA, b=pointB, c=pointC, color=color}
  debugObjectCounter.triangleNum = debugObjectCounter.triangleNum + 1
  table.insert(debugObjects.triangles, debugObjectCounter.triangleNum, triangle)
  local resp = {type ='DebugTriangleAdded', triangleID = debugObjectCounter.triangleNum}
  rcom.sendMessage(skt, resp)
end

M.handleAddDebugRectangle = function(skt, msg)
  local color = msg.color
  color = ColorI(math.ceil(color[1]*255), math.ceil(color[2]*255), math.ceil(color[3]*255), math.ceil(color[4]*255))
  local pointA = tableToPoint3F(msg.vertices[1], msg.cling, msg.offset)
  local pointB = tableToPoint3F(msg.vertices[2], msg.cling, msg.offset)
  local pointC = tableToPoint3F(msg.vertices[3], msg.cling, msg.offset)
  local pointD = tableToPoint3F(msg.vertices[4], msg.cling, msg.offset)
  local rectangle = {a=pointA, b=pointB, c=pointC, d=pointD, color=color}
  debugObjectCounter.rectangleNum = debugObjectCounter.rectangleNum + 1
  table.insert(debugObjects.rectangles, debugObjectCounter.rectangleNum, rectangle)
  local resp = {type ='DebugRectangleAdded', rectangleID = debugObjectCounter.rectangleNum}
  rcom.sendMessage(skt, resp)
end

M.handleAddDebugText = function(skt, msg)
  local color = ColorF(msg.color[1], msg.color[2], msg.color[3], msg.color[4])
  local origin = tableToPoint3F(msg.origin, msg.cling, msg.offset)
  local content = String(msg.content)
  local text = {origin = origin, content = content, color = color}
  debugObjectCounter.textNum = debugObjectCounter.textNum + 1
  table.insert(debugObjects.text, debugObjectCounter.textNum, text)
  local resp = {type ='DebugTextAdded', textID = debugObjectCounter.textNum}
  rcom.sendMessage(skt, resp)
end

M.handleAddDebugSquarePrism = function(skt, msg)
  local color = ColorF(msg.color[1], msg.color[2], msg.color[3], msg.color[4])
  local az, bz = msg.endPoints[1][3], msg.endPoints[2][3]
  local sideA = tableToPoint3F(msg.endPoints[1], false, 0)
  local sideB = tableToPoint3F(msg.endPoints[2], false, 0)
  local sideADims = Point2F(msg.dims[1][1], msg.dims[1][2])
  local sideBDims = Point2F(msg.dims[2][1], msg.dims[2][2])
  local prism = {sideA=sideA, sideB=sideB, sideADims=sideADims, sideBDims=sideBDims, color = color}
  debugObjectCounter.prismNum = debugObjectCounter.prismNum + 1
  table.insert(debugObjects.squarePrisms, debugObjectCounter.prismNum, prism)
  local resp = {type ='DebugSquarePrismAdded', prismID = debugObjectCounter.prismNum}
  rcom.sendMessage(skt, resp)
end

M.onDrawDebug = function(dtReal, lastFocus)
  for _, sphere in pairs(debugObjects.spheres) do
    debugDrawer:drawSphere(sphere.coo, sphere.radius, sphere.color)
  end
  for _, dSphere in pairs(debugObjects.dynamicSpheres) do
    local spec = dSphere.getSpec()
    debugDrawer:drawSphere(spec.coo, spec.radius, spec.color)
  end
  for _, polyline in pairs(debugObjects.polylines) do
    for _, segment in pairs(polyline.segments) do
      debugDrawer:drawLine(segment.origin, segment.target, polyline.color)
    end
  end
  for _, cylinder in pairs(debugObjects.cylinders) do
    debugDrawer:drawCylinder(cylinder.circleAPos, cylinder.circleBPos, cylinder.radius, cylinder.color)
  end
  for _, triangle in pairs(debugObjects.triangles) do
    debugDrawer:drawTriSolid(triangle.a, triangle.b, triangle.c, triangle.color)
  end
  for _, rectangle in pairs(debugObjects.rectangles) do
    debugDrawer:drawQuadSolid(rectangle.a, rectangle.b, rectangle.c, rectangle.d, rectangle.color)
  end
  for _, line in pairs(debugObjects.text) do
    debugDrawer:drawText(line.origin, line.content, line.color)
  end
  for _, prism in pairs(debugObjects.squarePrisms) do
    debugDrawer:drawSquarePrism(prism.sideA, prism.sideB, prism.sideADims, prism.sideBDims, prism.color)
  end
end

M.handleQueueLuaCommandGE = function(skt, msg)
  local func, loading_err = load(msg.chunk)
  if func then
    local status, err = pcall(func)
    if not status then
      log('E', 'execution error: "' .. err .. '"')
    end
  else
    log('E', 'compilation error in: "' .. msg.chunk .. '"')
  end
  rcom.sendACK(skt, 'ExecutedLuaChunkGE')
end

M.handleGetLevels = function(skt, msg)
  local list = core_levels.getList()
  local resp = {type = 'GetLevels', result = list}
  rcom.sendMessage(skt, resp)
end

M.handleGetScenarios = function(skt, msg)
  local list = scenario_scenariosLoader.getList(nil, true)
  local resp = {type = 'GetScenarios', result = list}
  rcom.sendMessage(skt, resp)
end

M.handleGetCurrentScenario = function(skt, msg)
  if scenario_scenarios == nil then
    rcom.sendBNGValueError(skt, 'No scenario loaded.')
    return false
  end

  local scenario = nil
  local ref = scenario_scenarios.getScenario()

  -- Horribly inefficient but the scenario object returned by the extension contains a lot of fields
  -- that are not serializable and not suitable to be sent over the socket so we find the respective
  -- scenario entry in the list of all scenarios instead
  -- TODO: Filter out unserializable fields from scenario object and send those instead
  local scenarios = scenario_scenariosLoader.getList(nil, true)
  for i = 1, #scenarios do
    if scenarios[i].sourceFile == ref.sourceFile then
      scenario = scenarios[i]
      break
    end
  end

  local resp = {type = 'GetCurrentScenario', result = scenario}
  rcom.sendMessage(skt, resp)
end

M.handleCreateScenario = function(skt, msg)
  local name = msg['name']
  local level = msg['level']
  local prefab = msg['prefab']
  local info = msg['info']
  local outFile = nil

  if name == nil then
    rcom.sendBNGValueError(skt, 'Scenario needs a name.')
    return false
  end

  if level == nil then
    rcom.sendBNGValueError(skt, 'Scenario needs an associated level.')
    return false
  end

  if info == nil then
    rcom.sendBNGValueError(skt, 'Scenario needs an info file definition.')
    return false
  end

  local path = '/levels/' .. level .. '/scenarios/'

  if prefab ~= nil then
    local prefabPath = path .. name .. '.prefab'
    outFile = io.open(prefabPath, 'w')
    outFile:write(prefab)
    outFile:close()
    outFile = nil
  end

  local infoPath = path .. name .. '.json'
  outFile = io.open(infoPath, 'w')
  outFile:write(jsonEncode({info}))
  outFile:close()
  outFile = nil

  local resp = {type = 'CreateScenario', result = infoPath}
  rcom.sendMessage(skt, resp)
end

M.handleDeleteScenario = function(skt, msg)
  local infoPath = msg['path']
  local scenarioDir, infoFile, _ = path.splitWithoutExt(infoPath)
  local prefabPath = scenarioDir .. infoFile .. '.prefab'

  FS:removeFile(infoPath)
  FS:removeFile(prefabPath)

  rcom.sendACK(skt, 'DeleteScenario')
end

M.handleGetCurrentVehicles = function(skt, msg)
  vehicleInfo = {}

  for id, v in pairs(map.objects) do
    local veh = scenetree.findObjectById(id)
    local data = core_vehicle_manager.getVehicleData(id)

    local info = {}
    info['id'] = id
    info['model'] = veh:getJBeamFilename()
    info['name'] = veh:getName()
    if info['name'] == nil then
      info['name'] = tostring(id)
    end

    local currentId = be:getPlayerVehicle(0):getID()
    be:enterVehicle(0, veh)
    info['config'] = core_vehicle_partmgmt.getConfig()
    be:enterVehicle(0, scenetree.findObjectById(currentId))

    info['options'] = jsonReadFile('/vehicles/' .. info['model'] .. '.json')

    vehicleInfo[id] = info

    vehicleInfoPending = vehicleInfoPending + 1
    veh:queueLuaCommand('extensions.load("researchVE")')
    veh:queueLuaCommand('researchVE.requestVehicleInfo()')
  end

  block('vehicleInfo', skt)
end

local function getSceneTreeNode(obj)
  local node = {}
  node.class = obj:getClassName()
  node.name = obj:getName()
  node.id = obj:getID()
  if obj.getObject ~= nil and obj.getCount ~= nil then
    node.children = {}
    local count = obj:getCount()
    for i=0, count - 1 do
      local child = getSceneTreeNode(Sim.upcast(obj:getObject(i)))
      table.insert(node.children, child)
    end
  end
  return node
end

M.handleGetSceneTree = function(skt, msg)
  local rootGrp = Sim.upcast(Sim.findObject('MissionGroup'))
  local tree = getSceneTreeNode(rootGrp)
  local resp = {type = 'GetSceneTree', result = tree}
  rcom.sendMessage(skt, resp)
end

local typeConverters = {}
typeConverters['MatrixPosition'] = function(t)
  return string.split(t)
end
typeConverters['MatrixRotation'] = function(t)
  return string.split(t)
end
typeConverters['Point3F'] = function(t)
  return string.split(t)
end

local function serializeGenericObject(obj)
  local ignoreNames = {
    id = true,
    name = true,
    internalName = true,
    isSelectionEnabled = true,
    isRenderEnabled = true,
    hidden = true,
    canSaveDynamicFields = true,
    canSave = true,
    parentGroup = true,
    persistentId = true,
    rotationMatrix = true,
    class = true,
    superClass = true,
    edge = true,
    plane = true,
    point = true,
  }

  local okayTypes = {
    int = true,
    string = true,
    filename = true,
    float = true,
    MatrixPosition = true,
    MatrixRotation = true,
    annotation = true,
    bool = true,
    Point3F = true,
    ColorF = true,
    TSMeshType = true,
  }

  local position = nil
  if obj.getPosition ~= nil then
    position = obj:getPosition()
    position = {position.x, position.y, position.z}
  else
    position = {0, 0, 0}
  end

  local rotation = nil
  if obj.getRotation ~= nil then
    rotation = obj:getRotation()
    rotation = {rotation.x, rotation.y, rotation.z, rotation.w}
  else
    rotation = {0, 0, 0, 0}
  end

  local scale = nil
  if obj.getScale ~= nil then
    scale = obj:getScale()
    scale = {scale.x, scale.y, scale.z}
  else
    scale = {0, 0, 0}
  end

  local ret = {
    id = obj:getID(),
    name = obj:getName(),
    class = obj:getClassName(),
    position = position,
    rotation = rotation,
    scale = scale
  }

  local fields = obj:getFieldList()
  for field, props in pairs(fields) do
    if ignoreNames[field] == nil then
      local type = props['type']
      if okayTypes[type] then
        local converter = typeConverters[type]
        if converter ~= nil then
          ret[field] = converter(obj:getField(field, ''))
        else
          ret[field] = obj:getField(field, '')
        end
      end
    end
  end

  return ret
end

local objectSerializers = {}
objectSerializers['DecalRoad'] = function(obj)
  local ret = serializeGenericObject(obj)

  local position = obj:getPosition()
  position = {position.x, position.y, position.z}
  local rotation = obj:getRotation()
  rotation = {rotation.x, rotation.y, rotation.z, rotation.w}
  local scale = obj:getScale()
  scale = {scale.x, scale.y, scale.z}

  local annotation = obj:getField('annotation', '')
  local detail = obj:getField('Detail', '')
  local material = obj:getField('Material', '')
  local breakAngle = obj:getField('breakAngle', '')
  local drivability = obj:getField('drivability', '')
  local flipDirection = obj:getField('flipDirection', '')
  local improvedSpline = obj:getField('improvedSpline', '')
  local lanesLeft = obj:getField('lanesLeft', '')
  local lanesRight = obj:getField('lanesRight', '')
  local oneWay = obj:getField('oneWay', '')
  local overObjects = obj:getField('overObjects', '')

  local lines = {}
  local edges = obj:getEdgesTable()
  for i = 1, #edges do
    local edge = edges[i]
    table.insert(lines, {
      left = {
        edge[1].x,
        edge[1].y,
        edge[1].z
      },
      middle = {
        edge[2].x,
        edge[2].y,
        edge[2].z
      },
      right = {
        edge[3].x,
        edge[3].y,
        edge[3].z
      }
    })
  end

  ret.lines = lines

  return ret
end

M.handleGetObject = function(skt, msg)
  local id = msg['id']
  local obj = Sim.findObjectById(id)
  if obj ~= nil then
    obj = Sim.upcast(obj)
    local class = obj:getClassName()
    local serializer = objectSerializers[class]
    if serializer ~= nil then
      obj = serializer(obj)
    else
      obj = serializeGenericObject(obj)
    end
    local resp = {type = 'GetObject', result = obj}
    rcom.sendMessage(skt, resp)
  else
    rcom.sendBNGValueError(skt, 'Unknown object ID: ' .. tostring(id))
  end
end

M.handleGetPartConfig = function(skt, msg)
  local vid = msg['vid']
  local veh = scenetree.findObject(vid)
  local cur = be:getPlayerVehicle(0):getID()
  be:enterVehicle(0, veh)
  local cfg = core_vehicle_partmgmt.getConfig()
  local resp = {type = 'PartConfig', config = cfg}
  veh = scenetree.findObjectById(cur)
  be:enterVehicle(0, veh)
  rcom.sendMessage(skt, resp)
end

M.handleGetPartOptions = function(skt, msg)
  local vid = msg['vid']
  local veh = scenetree.findObject(vid)
  local cur = be:getPlayerVehicle(0):getID()
  be:enterVehicle(0, veh)
  local data = core_vehicle_manager.getPlayerVehicleData()
  local slotMap = jbeamIO.getAvailableSlotMap(data.ioCtx)
  local resp = {type = 'PartOptions', options = slotMap}
  veh = scenetree.findObjectById(cur)
  be:enterVehicle(0, veh)
  rcom.sendMessage(skt, resp)
end

M.handleSetPartConfig = function(skt, msg)
  local vid = msg['vid']
  local veh = scenetree.findObject(vid)
  local cur = be:getPlayerVehicle(0):getID()
  be:enterVehicle(0, veh)
  local cfg = msg['config']
  core_vehicle_partmgmt.setConfig(cfg)
  veh = scenetree.findObjectById(cur)
  be:enterVehicle(0, veh)
end

return M
