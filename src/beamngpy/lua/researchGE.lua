-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}
local logTag = 'ResearchGE'
local version = 'v1.17'

local socket = require('libs/luasocket/socket.socket')
local rcom = require('utils/researchCommunication')

local scenariosLoader = require('scenario/scenariosLoader')
local scenarioHelper = require('scenario/scenariohelper')

local procPrimitives = require('util/trackBuilder/proceduralPrimitives')

local host = '127.0.0.1'
local port = 64256

local skt = nil
local clients = {}

local gameState = 'menu'

local conSleep = 1
local stepsLeft = 0
local stepACK = false

local loadNotified = false

local loadRequested = false
local startRequested = false
local restartRequested = false
local waitingForMainMenu = false

local sensors = {}

local lidars = {}

local spawnPending = nil

local objectCount = 1

local frameDelayTimer = -1
local frameDelayFunc = nil

local debugLines = {}

local _log = log
local function log(level, message)
  _log(level, logTag, message)
end

local function checkMessage()
  local message, err = rcom.readMessage(clients)

  if err ~= nil then
    skt = nil
    clients = {}
    conSleep = 5
    return false
  end

  if message ~= nil then
    local msgType = message['type']
    if msgType ~= nil then
      msgType = 'handle' .. msgType
      local handler = M[msgType]
      if handler ~= nil then
        return handler(message)
      else
        extensions.hook('onSocketMessage', skt, message)
        return true
      end
    else
      return true
    end
  else
    return false
  end
end

local function connect()
  log('I', 'Trying to connect to: ' .. host .. ':' .. tostring(port))
  skt = socket.connect(host, port)
  if skt ~= nil then
    log('I', 'Connected!')
    table.insert(clients, skt)

    local hello = {type = 'Hello', version = version}
    rcom.sendMessage(skt, hello)
  else
    log('I', 'Could not connect...')
  end
end

M.onPreRender = function(dt)
  if skt == nil then
    if conSleep <= 0 then
      conSleep = 5
      connect()
    else
      conSleep = conSleep - dt
    end

    return
  end

  if frameDelayTimer > 0 then
    frameDelayTimer = frameDelayTimer - 1
    if frameDelayTimer == 0 then
      frameDelayFunc()
      frameDelayFunc = nil
      frameDelayTimer = 0
    end
  end

  if stepsLeft > 0 then
    stepsLeft = stepsLeft - 1
    if stepsLeft == 0 and stepACK then
      rcom.sendACK(skt, 'Stepped')
    end
  end

  if waitingForMainMenu then
    if next(core_gamestate.state) == nil then
      rcom.sendACK(skt, 'ScenarioStopped')
      waitingForMainMenu = false
    end
  end

  if stepsLeft == 0 then
    while checkMessage() do end
  end
end

M.onScenarioUIReady = function(state)
  if state == 'start' and loadRequested and loadNotified == false then
    rcom.sendACK(skt, 'MapLoaded')
    loadNotified = true
    loadRequested = false
  end
end

M.onCountdownEnded = function()
  if startRequested then
    rcom.sendACK(skt, 'ScenarioStarted')
    loadNotified = false
    startRequested = false
  end
end

M.onScenarioRestarted = function()
  if restartRequested then
    M.handleStartScenario(nil)
    rcom.sendACK(skt, 'ScenarioRestarted')
    restartRequested = false
  end
end

M.onInit = function()
  local cmdArgs = Engine.getStartingArgs()
  for i, v in ipairs(cmdArgs) do
    if v == "-rport" then
      port = tonumber(cmdArgs[i + 1])
    end

    if v == "-rhost" then
      host = cmdArgs[i + 1]
    end
  end

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
end

-- Handlers

M.handleLoadScenario = function(msg)
  local scenarioPath = msg["path"]
  local ret = scenariosLoader.startByPath(scenarioPath)
  log('I', 'Loading scenario: '..scenarioPath)
  if ret then
    log('I', 'Scenario found...')
    loadRequested = true
  else
    log('I', 'Scenario not found...')
    rcom.sendBNGValueError(skt, 'Scenario not found: "' .. scenarioPath .. '"')
  end
  return false
end

M.handleStartScenario = function(msg)
  scenario_scenarios.changeState("running")
  scenario_scenarios.getScenario().showCountdown = false
  scenario_scenarios.getScenario().countDownTime = 0
  guihooks.trigger("ChangeState", "menu")
  startRequested = true
  return true
end

M.handleRestartScenario = function(msg)
  scenario_scenarios.restartScenario()
  restartRequested = true
  return false
end

M.handleStopScenario = function(msg)
  returnToMainMenu()
  waitingForMainMenu = true
  return false
end

M.handleGetScenarioName = function(msg)
  local name = scenario_scenarios.getscenarioName()
  local resp = {type = 'ScenarioName', name = name}
  rcom.sendMessage(skt, resp)
end

M.handleHideHUD = function(msg)
  be:executeJS('document.body.style.opacity = "0.0";')
  return true
end

M.handleShowHUD = function(msg)
  be:executeJS('document.body.style.opacity = "1.0";')
  return true
end

M.handleSetPhysicsDeterministic = function(msg)
  be:setPhysicsSpeedFactor(-1)
  rcom.sendACK(skt, 'SetPhysicsDeterministic')
  return true
end

M.handleSetPhysicsNonDeterministic = function(msg)
  be:setPhysicsSpeedFactor(0)
  rcom.sendACK(skt, 'SetPhysicsNonDeterministic')
  return true
end

M.handleFPSLimit = function(msg)
  settings.setValue('FPSLimiter', msg['fps'], true)
  settings.setState({FPSLimiterEnabled = true}, true)
  rcom.sendACK(skt, 'SetFPSLimit')
  return true
end

M.handleRemoveFPSLimit = function(msg)
  settings.setState({FPSLimiterEnabled = false}, true)
  rcom.sendACK(skt, 'RemovedFPSLimit')
  return true
end

M.handlePause = function(msg)
  be:setPhysicsRunning(false)
  rcom.sendACK(skt, 'Paused')
  return true
end

M.handleResume = function(msg)
  be:setPhysicsRunning(true)
  rcom.sendACK(skt, 'Resumed')
  return true
end

M.handleStep = function(msg)
  local count = msg["count"]
  be:physicsStep(count)
  stepsLeft = count
  stepACK = msg["ack"]
  return true
end

M.handleTeleport = function(msg)
  local vID = msg['vehicle']
  local veh = scenarioHelper.getVehicleByName(vID)
  if msg['rot'] ~= nil then
    local quat = quat(msg['rot'][1], msg['rot'][2], msg['rot'][3], msg['rot'][4])
    veh:setPositionRotation(msg['pos'][1], msg['pos'][2], msg['pos'][3], quat.x, quat.y, quat.z, quat.w)
  else
    veh:setPosition(Point3F(msg['pos'][1], msg['pos'][2], msg['pos'][3]))
  end
  rcom.sendACK(skt, 'Teleported')
  return true
end

M.handleTeleportScenarioObject = function(msg)
  local sobj = scenetree.findObject(msg['id'])
  if msg['rot'] ~= nil then
    local quat = quat(msg['rot'][1], msg['rot'][2], msg['rot'][3], msg['rot'][4])
    sobj:setPosRot(msg['pos'][1], msg['pos'][2], msg['pos'][3], quat.x, quat.y, quat.z, quat.w)
  else
    sobj:setPosition(Point3F(msg['pos'][1], msg['pos'][2], msg['pos'][3]))
  end
  rcom.sendACK(skt, 'ScenarioObjectTeleported')
  return true
end

M.handleVehicleConnection = function(msg)
  local vID, vHost, vPort, veh, command

  vID = msg['vid']
  vHost = msg['host']
  vPort = msg['port']

  command = 'extensions.load("researchVE")'
  veh = scenarioHelper.getVehicleByName(vID)
  veh:queueLuaCommand(command)

  local exts = msg['exts']
  if exts then
    for idx, ext in pairs(exts) do
      command = 'extensions.load("' .. ext .. '")'
      veh:queueLuaCommand(command)
    end
  end

  command = 'researchVE.startConnecting("' .. vHost .. '", '
  command = command .. tostring(vPort) .. ')'
  veh:queueLuaCommand(command)
  return true
end

M.handleOpenShmem = function(msg)
  local name = msg['name']
  local size = msg['size']

  Engine.openShmem(name, size)

  rcom.sendACK(skt, 'OpenedShmem')
  return true
end

M.handleCloseShmem = function(msg)
  local name = msg['name']

  Engine.closeShmem(name)

  rcom.sendACK(skt, 'ClosedShmem')
  return true
end

M.handleWaitForSpawn = function(msg)
  local name = msg['name']
  spawnPending = name
  return true
end

M.onVehicleSpawned = function(vID)
  if spawnPending ~= nil then
    local obj = scenetree.findObject(spawnPending)
    log('I', 'Vehicle spawned: ' .. tostring(vID))
    if obj ~= nil and obj:getID() == vID then
      local resp = {type = 'VehicleSpawned', name = spawnPending}
      spawnPending = nil
      rcom.sendMessage(skt, resp)
    end
  end
end

M.handleSpawnVehicle = function(msg)
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

  core_vehicles.spawnNewVehicle(model, options)
end

M.handleDespawnVehicle = function(msg)
  local name = msg['vid']
  local veh = scenetree.findObject(name)
  if veh ~= nil then
    veh:delete()
  end
  rcom.sendACK(skt, 'VehicleDespawned')
end

sensors.Camera = function(req, callback)
  local offset, orientation, up
  local pos, direction, rot, fov, resolution, nearFar, vehicle, vehicleObj, data
  local color, depth, annotation

  color = req['color']
  depth = req['depth']
  annotation = req['annotation']

  if req['vehicle'] then
    vehicle = scenarioHelper.getVehicleByName(req['vehicle'])
    orientation = vec3(vehicle:getDirectionVector())

    up = vec3(vehicle:getDirectionVectorUp())
    orientation = quatFromDir(orientation, up)
    offset = vec3(vehicle:getPosition())
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

  local data = Engine.renderCameraShmem(color, depth, annotation, pos, rot, resolution, fov, nearFar)

  callback(data)
end

sensors.Lidar = function(req, callback)
  local name = req['name']
  local lidar = lidars[name]
  if lidar ~= nil then
    lidar:requestDataShmem(function(realSize)
      callback({size = realSize})
    end)
  else
    callback(nil)
  end
end

sensors.Timer = function(req, callback)
  callback({time = scenario_scenarios.getScenario().timer})
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

M.handleSensorRequest = function(msg)
  local requests

  local cb = function(response)
    response = {type = 'SensorData', data = response}
    rcom.sendMessage(skt, response)
  end

  requests = msg['sensors']

  getNextSensorData(requests, {}, cb)
  return true
end

M.handleGetDecalRoadVertices = function(msg)
  local response = Sim.getDecalRoadVertices()
  response = {type = "DecalRoadVertices", vertices = response}
  rcom.sendMessage(skt, response)
  return true
end

M.handleGetDecalRoadData = function(msg)
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
  return true
end

M.handleGetDecalRoadEdges = function(msg)
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
  return true
end

M.handleEngineFlags = function(msg)
  log('I', 'Setting engine flags!')
  local flags = msg['flags']
  if flags['annotations'] then
    Engine.Annotation.enable(true)
  end

  rcom.sendACK(skt, 'SetEngineFlags')
  return true
end

M.handleTimeOfDayChange = function(msg)
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

M.handleUpdateScenario = function(msg)
  local response = {type = 'ScenarioUpdate'}
  local vehicleStates = {}
  for idx, vid in ipairs(msg['vehicles']) do
    vehicleStates[vid] = getVehicleState(vid)
    print('Got vehicle state: ' .. vid)
  end
  response['vehicles'] = vehicleStates

  rcom.sendMessage(skt, response)
  return true
end

M.handleOpenLidar = function(msg)
  log('I', 'Opening lidar!')
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
  if msg['visualized'] then
    log('I', 'Visualizing lidar!')
    lidar:visualized(true)
  else
    log('I', 'Not visualizing lidar!')
    lidar:visualized(false)
  end
  lidars[name] = lidar

  rcom.sendACK(skt, 'OpenedLidar')
  return true
end

M.handleCloseLidar = function(msg)
  local name = msg['name']
  local lidar = lidars[name]
  if lidar ~= nil then
    -- lidar:close()
    lidars[name] = nil
  end
  rcom.sendACK(skt, 'ClosedLidar')
  return true
end

M.handleSetWeatherPreset = function(msg)
  local preset = msg['preset']
  local time = msg['time']
  core_weather.switchWeather(preset, time)
  rcom.sendACK(skt, 'WeatherPresetChanged')
end

M.handleGameStateRequest = function(msg)
  local state = core_gamestate.state.state
  resp = {type = 'GameState'}
  if state == 'scenario' then
    resp['state'] = 'scenario'
    resp['scenario_state'] = scenario_scenarios.getScenario().state
  else
    resp['state'] = 'menu'
  end
  rcom.sendMessage(skt, resp)
  return true
end

M.handleDisplayGuiMessage = function(msg)
  local message = msg['message']
  guihooks.message(message)
  rcom.sendACK(skt, 'GuiMessageDisplayed')
end

M.handleSwitchVehicle = function(msg)
  local vID = msg['vid']
  local vehicle = scenetree.findObject(vID)
  be:enterVehicle(0, vehicle)
  rcom.sendACK(skt, 'VehicleSwitched')
end

M.handleSetFreeCamera = function(msg)
  local pos = msg['pos']
  local direction = msg['dir']
  local rot = quatFromDir(vec3(direction[1], direction[2], direction[3]))

  commands.setFreeCamera()
  commands.setCameraPosRot(pos[1], pos[2], pos[3], rot.x, rot.y, rot.z, rot.w)
  rcom.sendACK(skt, 'FreeCameraSet')
  return true
end

M.handleParticlesEnabled = function(msg)
  local enabled = msg['enabled']
  Engine.Render.ParticleMgr.setEnabled(enabled)
  rcom.sendACK(skt, 'ParticlesSet')
end

M.handleAnnotateParts = function(msg)
  local vehicle = scenetree.findObject(msg['vid'])
  util_partAnnotations.annotateParts(vehicle:getID())
  rcom.sendACK(skt, 'PartsAnnotated')
end

M.handleRevertAnnotations = function(msg)
  print('Handling annotation reversion')
  local vehicle = scenetree.findObject(msg['vid'])
  util_partAnnotations.revertAnnotations(vehicle:getID())
  rcom.sendACK(skt, 'AnnotationsReverted')
end

M.handleGetPartAnnotations = function(msg)
  local vehicle = scenetree.findObject(msg['vid'])
  local colors = util_partAnnotations.getPartAnnotations(vehicle:getID())
  local converted = {}
  for key, val in pairs(colors) do
    converted[key] = {val.r, val.g, val.b}
  end
  rcom.sendMessage(skt, {type = 'PartAnnotations', colors = converted})
end

M.handleGetPartAnnotation = function(msg)
  local part = msg['part']
  local color = util_partAnnotations.getPartAnnotation(part)
  if color ~= nil then
    color = {color.r, color.g, color.b}
  end
  rcom.sendMessage(skt, {type = 'PartAnnotation', color = color})
end

M.handleFindObjectsClass = function(msg)
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

M.handleGetDecalRoadVertices = function(msg)
  local response = Sim.getDecalRoadVertices()
  response = {type = "DecalRoadVertices", vertices = response}
  rcom.sendMessage(skt, response)
  return true
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

M.handleCreateCylinder = function(msg)
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

M.handleCreateBump = function(msg)
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

M.handleCreateCone = function(msg)
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

M.handleCreateCube = function(msg)
  local name = msg['name']
  local size = vec3(msg['size'])
  local material = msg['material']
  local pos = msg['pos']
  local rot = msg['rot']

  local cube = procPrimitives.createCube(size, material)
  placeObject(name, cube, pos, rot)

  rcom.sendACK(skt, 'CreatedCube')
end

M.handleCreateRing = function(msg)
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

M.handleGetBBoxCorners = function(msg)
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

M.handleSetGravity = function(msg)
  local gravity = msg['gravity']
  core_environment.setGravity(gravity)
  rcom.sendACK(skt, 'GravitySet')
end

M.handleGetAvailableVehicles = function(msg)
  local resp = {type = 'AvailableVehicles', vehicles = {}}

  local models = core_vehicles.getModelList().models
  local configs = core_vehicles.getConfigList().configs

  for model, modelData in pairs(models) do
    local data = deepcopy(modelData)
    data.configurations = {}
    for key, config in pairs(configs) do
      if config.model_key == model then
        data.configurations[config.key] = config
      end
    end
    resp.vehicles[model] = data
  end

  rcom.sendMessage(skt, resp)
end

M.handleStartTraffic = function(msg)
  local participants = msg.participants
  local ids = {}
  for idx, participant in ipairs(participants) do
    local veh = scenetree.findObject(participant)

    if veh == nil then
      rcom.sendBNGValueError(skt, 'Vehicle not present for traffic: ' .. tostring(participant))
      return false
    end

    table.insert(ids, veh:getID())
  end

  gameplay_traffic.activate(ids)
  rcom.sendACK(skt, 'TrafficStarted')
end

M.handleStopTraffic = function(msg)
  local stop = msg.stop
  gameplay_traffic.deactivate(stop)
  rcom.sendACK(skt, 'TrafficStopped')
end

M.handleChangeSetting = function(msg)
  local key = msg['key']
  local value = msg['value']
  settings.setValue(key, value, true)
  rcom.sendACK(skt, 'SettingsChanged')
end

M.handleApplyGraphicsSetting = function(msg)
  core_settings_graphic.applyGraphicsState()
  rcom.sendACK(skt, 'GraphicsSettingApplied')
end

M.handleSetRelativeCam = function(msg)
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
end

local debugObjects = { spheres = {}, 
                       polylines = {}, 
                       cylinders = {}, 
                       triangles = {}, 
                       rectangles ={},
                       text = {},
                       squarePrisms = {}
                      }
local debugObjectCounter = {sphereNum = 0, 
                            lineNum = 0, 
                            cylinderNum = 0, 
                            triangleNum = 0,
                            rectangleNum = 0,
                            textNum = 0,
                            prismNum = 0
                          }

local function tableToPoint3F(point, cling, offset)
  local point = Point3F(point[1], point[2], point[3])
  if cling then 
    local z = be:getSurfaceHeightBelow(point)
    point = Point3F(point.x, point.y, z+offset)
  end
  return point
end

M.handleAddDebugSpheres = function(msg)
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

M.handleRemoveDebugObjects = function(msg)
  for _, idx in pairs(msg.objIDs) do
    debugObjects[msg.objType][idx] = nil
  end
  rcom.sendACK(skt, 'DebugObjectsRemoved')
end

M.handleAddDebugPolyline = function(msg)
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

M.handleAddDebugCylinder = function(msg)
  local circleAPos = tableToPoint3F(msg.circlePositions[1], false, 0)
  local circleBPos = tableToPoint3F(msg.circlePositions[2], false, 0)
  local color = ColorF(msg.color[1], msg.color[2], msg.color[3], msg.color[4])
  local cylinder = {circleAPos=circleAPos, circleBPos=circleBPos, radius=msg.radius, color=color}
  debugObjectCounter.cylinderNum = debugObjectCounter.cylinderNum + 1
  table.insert(debugObjects.cylinders, debugObjectCounter.cylinderNum, cylinder)
  local resp = {type='DebugCylinderAdded', cylinderID=debugObjectCounter.cylinderNum}
  rcom.sendMessage(skt, resp)
end

M.handleAddDebugTriangle = function(msg)
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

M.handleAddDebugRectangle = function(msg)
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

M.handleAddDebugText = function(msg)
  local color = ColorF(msg.color[1], msg.color[2], msg.color[3], msg.color[4])
  local origin = tableToPoint3F(msg.origin, msg.cling, msg.offset)
  local content = String(msg.content)
  local text = {origin = origin, content = content, color = color}
  debugObjectCounter.textNum = debugObjectCounter.textNum + 1
  table.insert(debugObjects.text, debugObjectCounter.textNum, text)
  local resp = {type ='DebugTextAdded', textID = debugObjectCounter.textNum}
  rcom.sendMessage(skt, resp)
end

M.handleAddDebugSquarePrism = function(msg)
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

M.handleQueueLuaCommandGE = function(msg)
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

return M
