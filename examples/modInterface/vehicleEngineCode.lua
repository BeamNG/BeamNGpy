local M = {}

local logTag = 'vehicleEngineCode'

local function handleBar(request)
    log('I', logTag, request.text)
    request:sendACK('BarAck')
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
end

M.onInit = onInit
M.onSocketMessage = onSocketMessage
M.handleBar = handleBar

return M