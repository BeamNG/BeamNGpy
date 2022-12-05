local M = {}

local _log = log
local function log(level, msg)
  _log(level, 'vehicleEngineCode', msg)
end

local function handleBar(request)
    log('I', request.text)
    request:sendACK('BarAck')
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
M.handleBar = handleBar

return M