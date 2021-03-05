-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}

local mp = require('libs/lua-MessagePack/MessagePack')
local socket = require('libs/luasocket/socket.socket')

M.protocolVersion = 'v1.19'

-- Simple set implementation from the LuaSocket samples
M.newSet = function()
  local reverse = {}
  local set = {}
  return setmetatable(set, {__index = {
    insert = function(set, value)
      if not reverse[value] then
        table.insert(set, value)
        reverse[value] = #set
      end
    end,
    remove = function(set, value)
      local index = reverse[value]
      if index then
        reverse[value] = nil
        local top = table.remove(set)
        if top ~= value then
          reverse[top] = index
          set[index] = top
        end
      end
    end
  }})
end

M.checkForClients = function(servers)
  local ret = {}
  local readable, _, error = socket.select(servers, nil, 0)
  for _, input in ipairs(readable) do
    local client = input:accept()
    client:settimeout(0.1, 't')
    table.insert(ret, client)
  end
  return ret
end

M.receive = function(skt)
  local length, err = skt:receive(16)

  if err then
    log('E', 'ResearchCom', 'Error reading from socket: '..tostring(err))
    return nil, err
  end

  local data, err = skt:receive(tonumber(length))

  if err then
    log('E', 'ResearchCom', 'Error reading from socket: '..tostring(err))
    return nil, err
  end

  return data, nil
end

M.checkMessages = function(E, clients)
  local message, err
  local readable, writable, error = socket.select(clients, clients, 0)
  local ret = true

  for i = 1, #readable do
    local skt = readable[i]

    if writable[skt] == nil then
      goto continue
    end
    
    skt:settimeout(0.01, 't')
    
    message, err = M.receive(skt)
    
    if err ~= nil then
      clients:remove(skt)
      log('E', 'ResearchCom', 'Error reading from socket: ' .. tostring(skt) .. ' - ' .. tostring(err))
      goto continue
    end

    if message ~= nil then
      message = mp.unpack(message)
      local msgType = message['type']
      if msgType ~= nil then
        msgType = 'handle' .. msgType
        local handler = E[msgType]
        if handler ~= nil then
          if handler(skt, message) then
            ret = false
          end
        else
          extensions.hook('onSocketMessage', skt, message)
        end
      else
        log('E', 'ResearchCom', 'Got message without message type: ' .. tostring(message))
        goto continue
      end
    end
    
    ::continue::
  end

  if #readable > 0 then
    return ret
  else
    return false
  end
end

M.sendMessage = function(skt, message)
  local length
  if skt == nil then
    return
  end

  message = mp.pack(message)
  length = #message
  length = string.format('%016d', length)
  skt:send(length)
  skt:send(message)
end

M.sendACK = function(skt, type)
  local message = {type = type}
  M.sendMessage(skt, message)
end

M.sendBNGError = function(skt, message)
  local message = {bngError = message}
  M.sendMessage(skt, message)
end

M.sendBNGValueError = function(skt, message)
  local message = {bngValueError = message}
  M.sendMessage(skt, message)
end

M.openServer = function(port)
  local server = assert(socket.bind('*', port))
  server:settimeout(0.01)
  return server
end

return M
