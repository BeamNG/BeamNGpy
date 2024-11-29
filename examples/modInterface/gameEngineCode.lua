local M = {}

local logTag = 'gameEngineCode'

local function handleFoo(request)
    log('I', logTag, 'Hello ' .. request['someName'] .. '!\n')
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
        log('E', logTag, 'handler does not exist: ' .. msgType)
    end
end

local function onInit()
    log('I', logTag, 'Extension loaded.')
    setExtensionUnloadMode(M, 'manual') -- this is needed for the extension to survive through level loads
end

M.onInit = onInit
M.onSocketMessage = onSocketMessage
M.handleFoo = handleFoo
M.handleGetListOfVehicleModels = handleGetListOfVehicleModels

return M