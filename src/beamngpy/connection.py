import logging
import msgpack
import socket
from struct import pack, unpack
from time import sleep
from .beamngcommon import LOGGER_ID, PROTOCOL_VERSION, BNGError, BNGValueError

BUF_SIZE = 131072

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
        self.skt = None                                                         # The socket related to this connection instance. This is set upon connecting.
        self.logger = logging.getLogger(f'{LOGGER_ID}.BeamNGpy')
        self.logger.setLevel(logging.DEBUG)
        self.comm_logger = logging.getLogger(f'{LOGGER_ID}.communication')

    def connect_to_vehicle(self, vehicle, tries=25):
        """
        Sets the socket of this Connection instance, and attempts to connect it to the given vehicle.
        Upon failure, connections are re-attempted a limited amount of times.

        Args:
            vehicle (:class:`.Vehicle`): The vehicle instance to be connected.
            tries (int): The number of connection attempts.
        """
        self._initialize_socket()

        # If we do not have a port (ie because it is the first time we wish to send to the given vehicle), then fetch a new port from the simulator.
        if self.port is None:
            connection_msg = {'type': 'StartVehicleConnection'}
            connection_msg['vid'] = vehicle.vid
            if vehicle.extensions is not None:
                connection_msg['exts'] = vehicle.extensions
            self.bng.connection.send(connection_msg)
            resp = self.bng.connection.recv()
            assert resp['type'] == 'StartVehicleConnection'
            vid = resp['vid']
            assert vid == vehicle.vid
            self.port = int(resp['result'])
            self.logger.debug(f"Created new vehicle connection on port {self.port}")
            self.logger.info(f'Vehicle {vid} connected to simulation.')

        # Now attempt to connect to the given vehicle.
        flags = vehicle.get_engine_flags()
        self.bng.set_engine_flags(flags)
        while tries > 0:
            try:
                self.logger.info(f'Attempting to connect to vehicle {vehicle.vid}')
                self.skt.connect((self.host, self.port))
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
            sensor.connect(self.bng, self)

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
        self._initialize_socket()

        # Attempt to connect to the simulator through this socket.
        if log_tries:
            self.logger.info('Connecting to BeamNG.tech at: 'f'({self.host}, {self.port})')
        connected = False
        while tries > 0:
            try:
                self.skt.connect((self.host, self.port))
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
            self.skt.close()
        self.port = None
        self.host = None
        self.skt = None

    def reconnect(self):
        """
        Attempts to re-connect using this Connection instance, with the cached port and host.
        This will be called if a connection has been lost, in order to re-establish the connection.
        """
        self._initialize_socket()
        sleepTime = 0
        while True:
            try:
                self.skt.connect((self.host, self.port))
                break
            except (ConnectionRefusedError, ConnectionAbortedError) as err:
                sleep(sleepTime)
                sleepTime = 0.5

    def send(self, data):
        """
        Encodes the given data using Messagepack and sends the resulting bytes over the socket of this Connection instance.
        NOTE: messages are prefixed by the message length value.

        Args:
            data (dict): The data to encode and send
        """
        self.comm_logger.debug(f'Sending {data}.')
        data = msgpack.packb(data, use_bin_type=True)
        length = pack('!I', len(data))                                  # Prefix the message length to the front of the message data.
        data = length + data
        try:
            return self.skt.sendall(data)                               # First, attempt to send over the current socket stored in this Connection instance.
        except socket.error:
            self.reconnect()                                            # If the send has failed, we attempt to re-connect then we send again.
            return self.skt.sendall(data)

    def recv(self):
        """
        Reads a Messagepack-encoded message from the socket of this Connection instance, decodes it, then returns it.
        NOTE: messages are prefixed by the message length value.

        Returns:
            The recieved message, which has been decoded.
        """
        # First, attempt to receive and decode the message length.
        packed_length = self._recv_exactly(4)
        length = unpack('!I', packed_length)[0]

        # Now knowing the message length, attempt to recieve and decode the message body, populating a buffer as we go.
        recv_buffer = self._recv_exactly(length)
        data = msgpack.unpackb(recv_buffer, raw=False)
        self.comm_logger.debug(f'Received {data}.')
        if 'bngError' in data:
            raise BNGError(data['bngError'])
        if 'bngValueError' in data:
            raise BNGValueError(data['bngValueError'])

        # Converts all non-binary strings in the data into utf-8 format.
        return self._string_cleanup(data)

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
        self.send(kwargs)
        resp = self.recv()
        if resp['type'] != req:
            msg = 'Got Message type "{}" but expected "{}".'
            msg = msg.format(resp['type'], req)
            raise BNGValueError(msg)
        self.logger.debug(f"Got response for message of type {req}.")

        if 'result' in resp:
            return resp['result']
        return None

    def hello(self):
        """
        First function called after connections. Exchanges the protocol version with the connected simulator and raises an error upon mismatch.
        """
        data = dict(type='Hello')
        data['protocolVersion'] = PROTOCOL_VERSION
        self.send(data)
        resp = self.recv()
        assert resp['type'] == 'Hello'
        if resp['protocolVersion'] != PROTOCOL_VERSION:
            msg = 'Mismatching BeamNGpy protocol versions. Please ensure both BeamNG.tech and BeamNGpy are using the desired versions.\n' \
                  f'BeamNGpy\'s is: {PROTOCOL_VERSION}' \
                  f'BeamNG.tech\'s is: { resp["protocolVersion"] }'
            raise BNGError(msg)
        self.logger.info('Successfully connected to BeamNG.tech.')

    def _initialize_socket(self):
        """
        Set up the socket with the appropriate parameters for TCP_NODELAY.
        """
        self.skt = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.skt.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self.skt.settimeout(None)

    def _recv_exactly(self, length):
        """
        Receives exactly ``length`` bytes from the socket. If a socket error happens, the function
        tries to re-establish the connection.

        Returns:
            An array of length ``length`` with the received data.
        """
        recv_buffer = []
        while length > 0:
            try:
                received = self.skt.recv(min(BUF_SIZE, length))
            except socket.error:
                self.reconnect()
                received = self.skt.recv(min(BUF_SIZE, length))
            recv_buffer.extend(received)
            length -= len(received)
        assert length == 0

        return bytes(recv_buffer)