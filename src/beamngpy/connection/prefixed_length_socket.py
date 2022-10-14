from __future__ import annotations

import socket
import time
from struct import pack, unpack

BUF_SIZE = 131072


class PrefixedLengthSocket:
    HEADER_BYTES = 4

    @staticmethod
    def _initialize_socket():
        """
        Create a socket with the appropriate parameters for TCP_NODELAY.
        """
        skt = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        skt.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        skt.settimeout(None)
        return skt

    def _recv_exactly(self, length: int):
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

    def __init__(self, host: str, port: int):
        self.host = host
        self.port = port
        self.skt = self._initialize_socket()
        self.skt.connect((host, port))

    def __hash__(self) -> int:
        return id(self)

    def send(self, data):
        length = pack('!I', len(data))  # Prefix the message length to the front of the message data.
        data = length + data
        self.skt.sendall(data)

    def recv(self):
        packed_length = self._recv_exactly(self.HEADER_BYTES)
        length = unpack('!I', packed_length)[0]

        message = self._recv_exactly(length)
        return message

    def close(self):
        self.skt.close()

    def reconnect(self):
        """
        Attempts to re-connect using this instance, with the cached port and host.
        This will be called if a connection has been lost, in order to re-establish the connection.
        """
        self.skt = self._initialize_socket()
        sleep_time = 0
        while True:
            try:
                self.skt.connect((self.host, self.port))
                break
            except (ConnectionRefusedError, ConnectionAbortedError) as err:
                time.sleep(sleep_time)
                sleep_time = 0.5
