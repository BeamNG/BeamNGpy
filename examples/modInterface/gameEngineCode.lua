local M = {}

local rcom = require('utils/researchCommunication')

local _log = log
local function log(level, msg)
  _log(level, 'gameEngineCode', msg)
end

local function handleFoo(skt, msg)
    log("I", "Hello " .. msg['someName'] .. '!\n')
end

local function handleListOfVehicleModels(skt, msg)
    local models = {}
    for k, veh in pairs(getAllVehicles()) do 
        local vehType =  veh:getPath()
        vehType = string.gsub(vehType, 'vehicles/', '')
        vehType = string.gsub(vehType, '/', '')
        table.insert(models, vehType)
    end
    local response = {vehType="ListOfVehicleModels", data=models}
    rcom.sendMessage(skt, response)
    return true
end

local function onSocketMessage(skt, msg)
    local msgType = 'handle' .. msg['type']
    local handler = M[msgType]
    if handler ~= nil then
        handler(skt, msg)
    else
        log("E", "handler does not exist: " .. msgType)
    end
end

M.onSocketMessage = onSocketMessage
M.handleFoo = handleFoo
M.handleListOfVehicleModels = handleListOfVehicleModels

return M