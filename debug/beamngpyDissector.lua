-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt
--
-- HOW TO USE THIS FILE
-- This is a Wireshark dissector which you can use as a plugin by
-- copying this file into `%appdata%/Wireshark/plugins` and also copy
-- `MessagePack.lua` included IN THIS FOLDER to the Wireshark plugin directory.

-- With the usage of this plugin, you can debug the BeamNGpy communication
-- on the network level.
local mp = require('MessagePack')

local NAME = 'beamngpy'
local PORT = 25252
local LEGACY_PORT = 64256

-- we create our new protocol
local proto = Proto(NAME, "BeamNGpy TCP")

BEAMNG_VEHICLES = {}

local fields = {
  length = ProtoField.uint32(NAME .. '.length', 'Payload Length', base.DEC),
  id = ProtoField.uint32(NAME .. '.id', 'ID', base.DEC),
  type = ProtoField.string(NAME .. '.type', 'Type'),
  payload = ProtoField.bytes(NAME .. '.payload', 'Payload')
}
proto.fields = fields

-- Function to create a dynamic ProtoField based on field name and type
local function pf_dynamic_field(name, title, field_type)
  name = name:gsub("%./", ".")
  name = name:gsub("/", "_")
  name = name:gsub("*", "ALL")
  name = name:gsub("%.%.", ".0.")
  if field_type == "number" then
    return ProtoField.int32(name, title, base.DEC)
  end
  if field_type == "boolean" then
    return ProtoField.bool(name, title)
  end

  return ProtoField.string(name, title)
end

-- Helper function to add dynamic fields for the decoded MessagePack data
local function add_msgpack_fields(tree, data, prefix)
  for key, value in pairs(data) do
    local field_name = prefix .. "." .. tostring(key)
    if field_name == 'beamngpy.payload.type' then -- skip this, it's extracted into a custom field
      goto continue
    end

    -- Determine the field type based on the value type
    local field_type, display_value = nil, value
    if type(value) == "number" then
      field_type = "number"
    elseif type(value) == "boolean" then
      field_type = "boolean"
      display_value = value and "true" or "false"
    elseif type(value) == "table" then
      -- Recursively add fields for nested tables
      local subtree = tree:add(proto, nil, field_name, field_name)
      add_msgpack_fields(subtree, value, field_name)
      goto continue
    elseif type(value) == "string" then
      field_type = "string"
    else
      field_type = "bytes"
    end

    -- Add the dynamic field to the tree
    tree:add(pf_dynamic_field(field_name, key, field_type), key .. ':', display_value)
    ::continue::
  end
end

function proto.dissector(buffer, pinfo, tree)
  if buffer:len() < 4 then
    pinfo.desegment_len = DESEGMENT_ONE_MORE_SEGMENT
    return
  end

  pinfo.cols.protocol:set(NAME)
  local length = buffer(0, 4):uint()

  -- Check if the complete message is available
  if buffer:len() < 4 + length then
    pinfo.desegment_len = (4 + length) - buffer:len()
    return
  end

  -- We start by adding our protocol to the dissection display tree.
  local subtree = tree:add(proto, buffer())
  subtree:add(fields.length, buffer(0, 4))

  local payload = buffer(4, length)
  subtree:add(fields.payload, payload)

  local direction = nil
  local src = 'GE'
  if pinfo.src_port == PORT or pinfo.src_port == LEGACY_PORT then
    direction = '←'
  elseif pinfo.dst_port == PORT or pinfo.dst_port == LEGACY_PORT then
    direction = '→'
  end
  for vid, port in pairs(BEAMNG_VEHICLES) do
    if pinfo.src_port == port then
      src = vid
      direction = '←'
    elseif pinfo.dst_port == port then
      src = vid
      direction = '→'
    end
  end

  -- dissect the message fields
  local decoded_data = mp.unpack(payload:raw())

  local data_type = decoded_data.type
  if data_type then
    subtree:add(fields.type, data_type)
  elseif decoded_data.bngError or decoded_data.bngValueError then
    data_type = 'ERROR'
  else
    data_type = 'UNKNOWN'
  end

  if decoded_data.type == 'StartVehicleConnection' then
    if decoded_data.result then
      local port = math.floor(decoded_data.result)
      local vid = decoded_data.vid
      print(vid .. ' <-> ' .. tostring(port))
      if BEAMNG_VEHICLES[vid] ~= port then
        BEAMNG_VEHICLES[vid] = port
        DissectorTable.get("tcp.port"):add(port, proto)
      end
    end
  end

  -- If decoding was successful, add dynamic fields for each key-value pair
  if type(decoded_data) == "table" then
    add_msgpack_fields(subtree, decoded_data, NAME .. '.payload')
  else
    subtree:add_expert_info(PI_MALFORMED, PI_ERROR, 'Failed to decode MessagePack data')
  end

  local info = '[' .. data_type .. '] (' .. src .. ') | Length: ' .. length
  if direction then
    info = direction .. ' ' .. info
  end
  pinfo.cols.info:set(info)
  return message_length
end

-- we register our protocol on TCP port 25252
local tcp_table = DissectorTable.get("tcp.port")
tcp_table:add(PORT, proto)
tcp_table:add(LEGACY_PORT, proto)