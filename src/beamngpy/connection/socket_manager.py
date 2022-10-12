import asyncio
import threading

from .async_socket import AsyncSocket

_sockets = dict()
_loop: asyncio.AbstractEventLoop = None
_running: threading.Thread = None


def add_connection(host: str, port: int) -> AsyncSocket:
    if not _running:
        start_manager()

    try:
        skt = asyncio.run_coroutine_threadsafe(AsyncSocket.create(host, port), _loop).result()
        _sockets[skt] = skt
    except:
        if not _sockets:
            stop_manager()
        raise
    return skt


def remove_connection(skt: AsyncSocket):
    del _sockets[skt]
    asyncio.run_coroutine_threadsafe(skt.close(), _loop).result()
    if not _sockets:
        stop_manager()


def start_background_loop(loop: asyncio.AbstractEventLoop) -> None:
    asyncio.set_event_loop(loop)
    loop.run_forever()


def start_manager():
    global _running, _loop
    assert not _running

    _loop = asyncio.new_event_loop()
    t = threading.Thread(target=start_background_loop, args=(_loop, ))
    t.setDaemon(True)
    t.start()
    _running = t


def stop_manager():
    global _running, _loop
    assert _running
    _loop.call_soon_threadsafe(_loop.stop)
    _running = None
