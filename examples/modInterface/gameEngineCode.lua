local M = {}

local _log = log
local function log(level, msg)
  _log(level, 'gameEngineCode', msg)
end

local function handleFoo(request)
    log('I', 'Hello ' .. request['someName'] .. '!\n')
    request:sendACK('FooAck')
    return true
end

local function handleGetListOfVehicleModels(request)
    local models = {}
    for k, veh in pairs(getAllVehicles()) do 
        local vehType =  veh:getPath()
        vehType = string.gsub(vehType, 'vehicles/', '')
        vehType = string.gsub(vehType, '/', '')
        table.insert(models, vehType)
    end
    local response = {type='ListOfVehicleModels', data=models}
    request:sendResponse(response)
    return true
end

local function onSocketMessage(request)
    local msgType = 'handle' .. request['type']
    local handler = M[msgType]
    if handler ~= nil then
        handler(request)
    else
        log('E', 'handler does not exist: ' .. msgType)
    end
end

M.onSocketMessage = onSocketMessage
M.handleFoo = handleFoo
M.handleGetListOfVehicleModels = handleGetListOfVehicleModels

return M