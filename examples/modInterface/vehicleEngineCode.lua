local M = {}

local rcom = require('utils/researchCommunication')

local _log = log
local function log(level, msg)
  _log(level, 'vehicleEngineCode', msg)
end

local function handleBar(skt, msg)
    -- this code only runs in all vehicle VMs
    -- you'll have to introduce a check here if its only supposed to run in a particular vehicle VM
    log('D', "I was here last")
    print(msg.text)
end

local function onSocketMessage(skt, msg)
    log('D', "I was here first")
    log("D", "onSocketMessage")
    local msgType = 'handle' .. msg['type']
    local handler = M[msgType]
    if handler ~= nil then
        handler(skt, msg)
    else
        log("E", "handler does not exist: " .. msgType)
    end
end

M.onSocketMessage = onSocketMessage
M.handleBar = handleBar

return M