local M = {}

local rcom = require('utils/researchCommunication')

local _log = log
local function log(level, msg)
  _log(level, 'vehicleEngineCode', msg)
end

local function handleBar(skt, msg)
    log("I", msg.text)
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
M.handleBar = handleBar

return M