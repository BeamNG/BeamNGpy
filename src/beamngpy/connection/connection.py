from __future__ import annotations

import logging
import socket
from time import sleep
from typing import TYPE_CHECKING, Any, Dict, Tuple, cast

import msgpack

from beamngpy.logging import LOGGER_ID, BNGError, BNGValueError
from beamngpy.types import StrDict

from .prefixed_length_socket import PrefixedLengthSocket

if TYPE_CHECKING:
    from beamngpy.vehicle import Vehicle


class Connection:
    """
    The class for handling socket communication between BeamNGpy and the simulator, including establishing connections to both the simulator and to its
    vehicles individually, and for sending and recieving data across these sockets.

    Instantiates an instance of the Connection class, creating an unconnected socket ready to be connected when required.

    Args:
        host: The host to connect to.
        port: The port to connect to.
    """

    PROTOCOL_VERSION = "v1.23"

    @staticmethod
    def _textify_string(data: bytes) -> str | bytes:
        """
        Attempts to convert binary data to utf-8. If we can do this, we do it. If not, we leave as binary data.
        Args:
            data: The candidate data.
        Returns:
            The conversion, if it was possible to convert. Otherwise the untouched binary data.
        """
        try:
            return data.decode("utf-8")
        except:
            return data

    @staticmethod
    def _string_cleanup_rec(data: dict | list | bytes) -> dict | list | bytes | str:
        """
        Recursively iterates through data, and attempts to convert all binary data to utf-8.
        If we can do this with any elements of the data, we do it. If not, we leave them as binary data.
        Args:
            data: The data.
        Returns:
            The (possibly) converted data.
        """
        if isinstance(data, list):
            for i, val in enumerate(data):
                if isinstance(val, bytes):
                    data[i] = Connection._textify_string(val)
                elif isinstance(val, (list, dict)):
                    Connection._string_cleanup_rec(val)
        elif isinstance(data, dict):
            for key, val in data.items():
                if isinstance(val, bytes):
                    data[key] = Connection._textify_string(val)
                elif isinstance(val, (list, dict)):
                    Connection._string_cleanup_rec(val)
        elif isinstance(data, bytes):
            return Connection._textify_string(data)
        return data

    @staticmethod
    def _string_cleanup(data: StrDict) -> StrDict:
        for key, val in data.items():
            if isinstance(val, bytes):
                data[key] = Connection._textify_string(val)
            elif isinstance(val, (list, dict)):
                Connection._string_cleanup_rec(val)
        return data

    def __init__(self, host: str, port: int | None = None):
        self.host = host
        self.port = port
        # The socket related to this connection instance. This is set upon connecting.
        self.skt: PrefixedLengthSocket | None = None
        self.logger = logging.getLogger(f"{LOGGER_ID}.BeamNGpy")
        self.comm_logger = logging.getLogger(f"{LOGGER_ID}.communication")
        self.req_id = 0
        self.received_messages: Dict[int, StrDict | BNGError | BNGValueError] = {}

    def connect_to_vehicle(self, vehicle: Vehicle, tries: int = 25) -> None:
        """
        Sets the socket of this Connection instance, and attempts to connect it to the given vehicle.
        Upon failure, connections are re-attempted a limited amount of times.

        Args:
            vehicle: The vehicle instance to be connected.
            tries: The number of connection attempts.
        """
        if not self.port:
            raise BNGError("The simulator port is not set!")
        while tries > 0:
            try:
                self.logger.info(f"Attempting to connect to vehicle {vehicle.vid}")
                self.skt = PrefixedLengthSocket(self.host, self.port)
                break
            except (ConnectionRefusedError, ConnectionAbortedError, OSError) as err:
                msg = f"Error connecting to BeamNG.tech vehicle {vehicle.vid}. {tries} tries left."
                self.logger.error(msg)
                self.logger.exception(err)
                sleep(5)
                tries -= 1

        # Send a first message across the socket to ensure we have matching protocol values.
        self.hello()
        self.logger.info(f"Successfully connected to vehicle {vehicle.vid}.")

    def connect_to_beamng(self, tries: int = 25, log_tries: bool = True) -> bool:
        """
        Sets the socket of this connection instance and attempts to connect to the simulator over the host and port configuration set in this class.
        Upon failure, connections are re-attempted a limited amount of times.

        Args:
            tries: The number of connection attempts.
            log_tries: True if the connection logs should be propagated to the caller. Defaults to True.

        Returns:
            True if the connection was successful, False otherwise.
        """
        if not self.port:
            raise BNGError("The simulator port is not set!")

        # Attempt to connect to the simulator through this socket.
        if log_tries:
            self.logger.info(
                "Connecting to BeamNG.tech at: " f"({self.host}, {self.port})"
            )
        connected = False
        while tries > 0:
            try:
                self.skt = PrefixedLengthSocket(self.host, self.port)
                connected = True
                break
            except (ConnectionRefusedError, ConnectionAbortedError) as err:
                if log_tries:
                    self.logger.error(
                        f"Error connecting to BeamNG.tech. {tries} tries left."
                    )
                    self.logger.exception(err)
                tries -= 1
                if tries > 0:
                    sleep(5)

        if connected:
            self.hello()
            if log_tries:
                self.logger.info("BeamNGpy successfully connected to BeamNG.")
        return connected

    def disconnect(self) -> None:
        """
        Closes socket communication for this Connection instance.
        """
        if self.skt is not None:
            self.skt.close()
        self.skt = None

    def _assign_request_id(self) -> int:
        req_id = self.req_id
        self.req_id += 1
        return req_id

    def _pack_data(self, data: StrDict) -> Tuple[int, bytes]:
        req_id = self._assign_request_id()
        data["_id"] = req_id
        self.comm_logger.debug("Sending %s.", data)
        packed = cast(
            bytes, msgpack.packb(data, use_bin_type=True)
        )  # the cast is for type checker
        return req_id, packed

    def _unpack_data(self, data: bytes) -> StrDict:
        unpacked: StrDict = msgpack.unpackb(data, raw=False, strict_map_key=False)
        self.comm_logger.debug("Received %s.", unpacked)

        # Converts all non-binary strings in the data into utf-8 format.
        return self._string_cleanup(unpacked)

    def send(self, data: StrDict) -> Response:
        """
        Encodes the given data using Messagepack and sends the resulting bytes over the socket of this Connection instance.
        NOTE: messages are prefixed by the message length value.

        Args:
            data: The data to encode and send
        """
        if not self.skt:
            raise BNGError("Cannot send, not connected to the simulator.")
        req_id, packed_data = self._pack_data(data)
        try:
            # First, attempt to send over the current socket stored in this Connection instance.
            self.skt.send(packed_data)
        except socket.error:
            self.skt.reconnect()  # If the send has failed, we attempt to re-connect then we send again.
            self.skt.send(packed_data)
        return Response(self, req_id)

    def recv(self, req_id: int) -> StrDict | BNGError | BNGValueError:
        if req_id in self.received_messages:
            return self.received_messages.pop(req_id)
        if not self.skt:
            raise BNGError("Cannot receive, not connected to the simulator.")
        while True:
            message = self.skt.recv()
            message = self._unpack_data(message)
            if not "_id" in message:
                raise BNGError(
                    "Invalid message received! The version of BeamNG.tech running is incompatible with this version of BeamNGpy."
                )
            _id = int(message["_id"])
            del message["_id"]

            if "bngError" in message:
                message = BNGError(message["bngError"])
            elif "bngValueError" in message:
                message = BNGValueError(message["bngValueError"])

            if _id == req_id:
                return message
            self.received_messages[_id] = message

    def message(self, req: str, **kwargs: Any) -> Any:
        """
        Generic message function which is parameterized with the type of message to send and all parameters that are to be embedded in the request.
        Responses are expected to have the same type as the request. If this is not the case, an error is raised.

        Args:
            req (str): The request type.

        Returns:
            The response received from the simulator as a dictionary.
        """
        self.logger.debug(
            f"Sending message of type {req} to BeamNG.tech's engine in blocking mode."
        )
        kwargs["type"] = req
        resp = self.send(kwargs).recv(type=req)
        self.logger.debug(f"Got response for message of type {req}.")

        if "result" in resp:
            return resp["result"]
        return None

    def hello(self) -> None:
        """
        First function called after connections. Exchanges the protocol version with the connected simulator and raises an error upon mismatch.
        """
        data = dict(type="Hello")
        data["protocolVersion"] = self.PROTOCOL_VERSION
        resp = self.send(data).recv("Hello")
        if resp["protocolVersion"] != self.PROTOCOL_VERSION:
            msg = (
                "Mismatching BeamNGpy protocol versions. Please ensure both BeamNG.tech and BeamNGpy are using the desired versions.\n"
                f"BeamNGpy's is: {self.PROTOCOL_VERSION}\n"
                f'BeamNG.tech\'s is: { resp["protocolVersion"] }'
            )
            raise BNGError(msg)
        self.logger.info("Successfully connected to BeamNG.tech.")


class Response:
    def __init__(self, connection: Connection, req_id: int):
        self.connection = connection
        self.req_id = req_id

    def recv(self, type: str | None = None) -> StrDict:
        message = self.connection.recv(self.req_id)
        if isinstance(message, Exception):
            raise message
        if type and message["type"] != type:
            raise BNGValueError(
                f'Got Message type "{message["type"]}" but expected "{type}".'
            )
        return message

    def ack(self, ack_type: str) -> None:
        message = self.recv()
        if message["type"] != ack_type:
            raise BNGError(f'Wrong ACK: {ack_type} != {message["type"]}')
