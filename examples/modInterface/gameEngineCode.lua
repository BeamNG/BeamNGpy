local M = {}

local rcom = require('utils/researchCommunication')

local _log = log
local function log(level, msg)
  _log(level, 'gameEngineCode', msg)
end

local function handleFoo(skt, msg)
    log("I", "Hello " .. msg['someName'] .. '!\n')
    rcom.sendACK(skt, 'Goodbye!')
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

return M