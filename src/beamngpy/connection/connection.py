import logging
import socket
from time import sleep

import msgpack

from ..beamngcommon import LOGGER_ID, PROTOCOL_VERSION, BNGError, BNGValueError
from . import socket_manager as SocketManager


class Connection:
    """
    The class for handling socket communication between beamNGPy and the simulator, including establishing connections to both the simulator and to its
    vehicles individually, and for sending and recieving data across these sockets.
    """

    @staticmethod
    def _textify_string(data):
        """
        Attempts to convert binary data to utf-8. If we can do this, we do it. If not, we leave as binary data.
        Args:
            data (data): The candidate data.
        Returns:
            The conversion, if it was possible to convert. Otherwise the untouched binary data.
        """
        try:
            return data.decode('utf-8')
        except:
            return data

    @staticmethod
    def _string_cleanup(data):
        """
        Recursively iterates through data, and attempts to convert all binary data to utf-8.
        If we can do this with any elements of the data, we do it. If not, we leave them as binary data.
        Args:
            data (data): The data.
        Returns:
            The (possibly) converted data.
        """
        type_d = type(data)
        if type_d is list:
            for i, val in enumerate(data):
                type_v = type(val)
                if type_v is bytes:
                    data[i] = Connection._textify_string(val)
                elif type_v is list or type_v is dict:
                    Connection._string_cleanup(val)
        elif type_d is dict:
            for key, val in data.items():
                type_v = type(val)
                if type_v is bytes:
                    data[key] = Connection._textify_string(val)
                elif type_v is list or type_v is dict:
                    Connection._string_cleanup(val)
        elif type_d is bytes:
            data = Connection._textify_string(data)
        return data

    def __init__(self, bng, host, port=None):
        """
        Instantiates an instance of the Connection class, creating an unconnected socket ready to be connected when required.

        Args:
            bng (class): The BeamNGpy instance.
            host (str): The host to connect to.
            port (int): The port to connect to.
        """
        self.bng = bng
        self.host = host
        self.port = port
        # The socket related to this connection instance. This is set upon connecting.
        self.skt = None
        self.logger = logging.getLogger(f'{LOGGER_ID}.BeamNGpy')
        self.comm_logger = logging.getLogger(f'{LOGGER_ID}.communication')
        self.req_id = 0
        self.received_messages = {}

    def connect_to_vehicle(self, vehicle, tries=25):
        """
        Sets the socket of this Connection instance, and attempts to connect it to the given vehicle.
        Upon failure, connections are re-attempted a limited amount of times.

        Args:
            vehicle (:class:`.Vehicle`): The vehicle instance to be connected.
            tries (int): The number of connection attempts.
        """
        # If we do not have a port (ie because it is the first time we wish to send to the given vehicle), then fetch a new port from the simulator.
        if self.port is None:
            connection_msg = {'type': 'StartVehicleConnection'}
            connection_msg['vid'] = vehicle.vid
            if vehicle.extensions is not None:
                connection_msg['exts'] = vehicle.extensions
            resp = self.bng.connection.send(connection_msg).recv('StartVehicleConnection')
            vid = resp['vid']
            assert vid == vehicle.vid
            self.port = int(resp['result'])
            self.logger.debug(f'Created new vehicle connection on port {self.port}')
            self.logger.info(f'Vehicle {vid} connected to simulation.')

        # Now attempt to connect to the given vehicle.
        flags = vehicle.get_engine_flags()
        self.bng.set_engine_flags(flags)
        while tries > 0:
            try:
                self.logger.info(f'Attempting to connect to vehicle {vehicle.vid}')
                self.skt = SocketManager.add_connection(self.host, self.port)
                break
            except (ConnectionRefusedError, ConnectionAbortedError) as err:
                msg = f'Error connecting to BeamNG.tech vehicle {vehicle.vid}. {tries} tries left.'
                self.logger.error(msg)
                self.logger.exception(err)
                sleep(5)
                tries -= 1

        # Send a first message across the socket to ensure we have matching protocol values.
        self.hello()
        self.logger.info(f'Successfully connected to vehicle {vehicle.vid}.')

        # Connect the vehicle sensors.
        for _, sensor in vehicle.sensors.items():
            sensor.connect(self.bng, vehicle)

    def connect_to_beamng(self, tries=25, log_tries=True):
        """
        Sets the socket of this connection instance and attempts to connect to the simulator over the host and port configuration set in this class.
        Upon failure, connections are re-attempted a limited amount of times.

        Args:
            tries (int): The number of connection attempts.
            log_tries (bool): True if the connection logs should be propagated to the caller. Defaults to True.

        Returns:
            True if the connection was successful, False otherwise.
        """
        # Attempt to connect to the simulator through this socket.
        if log_tries:
            self.logger.info('Connecting to BeamNG.tech at: 'f'({self.host}, {self.port})')
        connected = False
        while tries > 0:
            try:
                self.skt = SocketManager.add_connection(self.host, self.port)
                connected = True
                break
            except (ConnectionRefusedError, ConnectionAbortedError) as err:
                if log_tries:
                    self.logger.error(f'Error connecting to BeamNG.tech. {tries} tries left.')
                    self.logger.exception(err)
                tries -= 1
                if tries > 0:
                    sleep(5)

        if connected:
            self.hello()
            if log_tries:
                self.logger.info('BeamNGpy successfully connected to BeamNG.')
        return connected

    def disconnect(self):
        """
        Closes socket communication for this Connection instance.
        """
        if self.skt is not None:
            SocketManager.remove_connection(self.skt)
        self.port = None
        self.host = None
        self.skt = None

    def reconnect(self):
        """
        Attempts to re-connect using this Connection instance, with the cached port and host.
        This will be called if a connection has been lost, in order to re-establish the connection.
        """
        SocketManager.remove_connection(self.skt)
        sleep_time = 0
        while True:
            try:
                self.skt = SocketManager.add_connection(self.host, self.port)
                break
            except (ConnectionRefusedError, ConnectionAbortedError, OSError) as err:
                self.logger.error(f'Error reconnecting to BeamNG.tech.')
                self.logger.exception(err)
                sleep(sleep_time)
                sleep_time = 0.5

    def _assign_request_id(self):
        req_id = self.req_id
        self.req_id += 1
        return req_id

    def _pack_data(self, data):
        req_id = self._assign_request_id()
        data['_id'] = req_id
        self.comm_logger.debug(f'Sending {data}.')
        data = msgpack.packb(data, use_bin_type=True)
        return req_id, data

    def send(self, data):
        """
        Encodes the given data using Messagepack and sends the resulting bytes over the socket of this Connection instance.
        NOTE: messages are prefixed by the message length value.

        Args:
            data (dict): The data to encode and send
        """
        req_id, data = self._pack_data(data)
        try:
            # First, attempt to send over the current socket stored in this Connection instance.
            self.skt.send(data)
        except socket.error:
            self.reconnect()  # If the send has failed, we attempt to re-connect then we send again.
            self.skt.send(data)
        return Response(self, req_id)

    async def send_async(self, data):
        """
        Encodes the given data using Messagepack and sends the resulting bytes over the socket of this Connection instance.
        NOTE: messages are prefixed by the message length value.

        Args:
            data (dict): The data to encode and send
        """
        req_id, data = self._pack_data(data)
        try:
            # First, attempt to send over the current socket stored in this Connection instance.
            await self.skt.send_async(data)
        except socket.error:
            self.reconnect()  # If the send has failed, we attempt to re-connect then we send again.
            await self.skt.send_async(data)
        return Response(self, req_id)

    async def send_recv_async(self, data, type=None, ack=None):
        response = await self.send_async(data)
        result = await response.recv_async(type)
        if ack:
            await response.ack_async(ack)
        return result

    def _unpack_data(self, data):
        data = msgpack.unpackb(data, raw=False, strict_map_key=False)
        self.comm_logger.debug(f'Received {data}.')

        # Converts all non-binary strings in the data into utf-8 format.
        data = self._string_cleanup(data)
        if 'bngError' in data:
            return BNGError(data['bngError'])
        if 'bngValueError' in data:
            return BNGValueError(data['bngValueError'])

        return data

    def recv(self, req_id):
        if req_id in self.received_messages:
            return self.received_messages.pop(req_id, None)
        while True:
            message = self.skt.recv()
            message = self._unpack_data(message)
            if not '_id' in message:
                raise BNGError(
                    'Invalid message received! The version of BeamNG.tech running is incompatible with this version of BeamNGpy.')
            _id = int(message['_id'])
            del message['_id']

            if _id == req_id:
                return message
            self.received_messages[_id] = message

    async def recv_async(self, req_id):
        if req_id in self.received_messages:
            return self.received_messages.pop(req_id)
        while True:
            message = await self.skt.recv_async()
            message = self._unpack_data(message)
            if not '_id' in message:
                raise BNGError(
                    'Invalid message received! The version of BeamNG.tech running is incompatible with this version of BeamNGpy.')
            _id = int(message['_id'])
            del message['_id']

            if _id == req_id:
                return message
            self.received_messages[_id] = message

    def message(self, req, **kwargs):
        """
        Generic message function which is parameterized with the type of message to send and all parameters that are to be embedded in the request.
        Responses are expected to have the same type as the request. If this is not the case, an error is raised.

        Args:
            req (str): The request type.

        Returns:
            The response received from the simulator as a dictionary.
        """
        self.logger.debug(f'Sending message of type {req} to BeamNG.tech\'s game engine in blocking mode.')
        kwargs['type'] = req
        resp = self.send(kwargs).recv(req)
        self.logger.debug(f'Got response for message of type {req}.')

        if 'result' in resp:
            return resp['result']
        return None

    def hello(self):
        """
        First function called after connections. Exchanges the protocol version with the connected simulator and raises an error upon mismatch.
        """
        data = dict(type='Hello')
        data['protocolVersion'] = PROTOCOL_VERSION
        resp = self.send(data).recv('Hello')
        if resp['protocolVersion'] != PROTOCOL_VERSION:
            msg = 'Mismatching BeamNGpy protocol versions. Please ensure both BeamNG.tech and BeamNGpy are using the desired versions.\n' \
                  f'BeamNGpy\'s is: {PROTOCOL_VERSION}' \
                  f'BeamNG.tech\'s is: { resp["protocolVersion"] }'
            raise BNGError(msg)
        self.logger.info('Successfully connected to BeamNG.tech.')


class Response:
    def __init__(self, connection: Connection, req_id: int):
        self.connection = connection
        self.req_id = req_id

    def ack(self, ack_type: str):
        message = self.recv()
        if message['type'] != ack_type:
            raise BNGError(f'Wrong ACK: {ack_type} != {message["type"]}')

    def recv(self, type: str = None) -> dict:
        message = self.connection.recv(self.req_id)
        if isinstance(message, Exception):
            raise message
        if type and message['type'] != type:
            raise BNGValueError(f'Got Message type "{message["type"]}" but expected "{type}".')
        return message

    async def recv_async(self, type: str = None) -> dict:
        message = await self.connection.recv_async(self.req_id)
        if isinstance(message, Exception):
            raise message
        if type and message['type'] != type:
            raise BNGValueError(f'Got Message type "{message["type"]}" but expected "{type}".')
        return message

    async def ack_async(self, ack_type: str):
        message = await self.recv_async()
        if message['type'] != ack_type:
            raise BNGError(f'Wrong ACK: {ack_type} != {message["type"]}')
