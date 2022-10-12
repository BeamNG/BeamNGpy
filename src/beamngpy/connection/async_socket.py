import asyncio
import logging
from asyncio.streams import StreamReader, StreamWriter
from struct import pack, unpack

from ..beamng import LOGGER_ID

BUF_SIZE = 131072


class AsyncSocket:
    HEADER_BYTES = 4

    @classmethod
    async def create(cls, host: str, port: int):
        self = AsyncSocket()
        self.host = host
        self.port = port
        self.loop = asyncio.get_running_loop()
        self.reader, self.writer = await asyncio.open_connection(host, port)
        return self

    def sockname(self):
        return self.writer._transport.get_extra_info('sockname')

    def __init__(self):
        self.host: str = None
        self.port: int = None
        self.reader: StreamReader = None
        self.writer: StreamWriter = None
        self.task: asyncio.Task = None
        self.loop: asyncio.AbstractEventLoop = None
        self.messages_to_send = asyncio.Queue()
        self.received_messages = asyncio.Queue()
        self.comm_logger = logging.getLogger(f'{LOGGER_ID}.communication')

    def __hash__(self) -> int:
        return id(self)

    async def _send(self, data):
        length = pack('!I', len(data))  # Prefix the message length to the front of the message data.
        data = length + data
        self.writer.write(data)
        await self.writer.drain()

    async def _recv(self):
        packed_length = await self.reader.readexactly(self.HEADER_BYTES)
        length = unpack('!I', packed_length)[0]

        message = await self.reader.readexactly(length)
        return message

    async def send_async(self, data: bytes):
        return await asyncio.wrap_future(asyncio.run_coroutine_threadsafe(self._send(data), self.loop))

    def send(self, data: bytes):
        asyncio.run_coroutine_threadsafe(self._send(data), self.loop)

    async def recv_async(self) -> bytes:
        return await asyncio.wrap_future(
            asyncio.run_coroutine_threadsafe(self._recv(), self.loop), loop=asyncio.get_running_loop())

    def recv(self):
        return asyncio.run_coroutine_threadsafe(self._recv(), self.loop).result()

    async def close(self):
        self.writer.close()
        await self.writer.wait_closed()
