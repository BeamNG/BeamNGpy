from __future__ import annotations

from beamngpy.logging import bngpy_logger

from multiprocessing import resource_tracker as rt
from multiprocessing import shared_memory as shm
import sys
import threading

if sys.version_info >= (3, 13):
    SharedMemory = shm.SharedMemory
else:

    class SharedMemory(shm.SharedMemory):
        __lock = threading.Lock()

        def __init__(
            self,
            name: str | None = None,
            create: bool = False,
            size: int = 0,
            *,
            track: bool = True,
        ) -> None:
            self._track = track
            if track:
                return super().__init__(name=name, create=create, size=size)
            with self.__lock:
                tmp = rt.register
                rt.register = self.__register
                try:
                    super().__init__(name=name, create=create, size=size)
                finally:
                    rt.register = tmp

        @staticmethod
        def __register(*args, **kwargs) -> None:
            return

        def unlink(self) -> None:
            if shm._USE_POSIX and self._name:
                shm._posixshmem.shm_unlink(self._name)
                if self._track:
                    shm.unregister(self._name, "shared_memory")


class BNGSharedMemory(SharedMemory):
    def __init__(self, size: int):
        super().__init__(None, create=True, size=size, track=False)
        self._closed = False

    def read(self, size: int) -> memoryview:
        return self.buf[:size]

    def try_close(self):
        if not self._closed:
            try:
                self.close()
                self._closed = True
            except Exception as e:
                bngpy_logger.warning(f"Cannot close shared memory. Original error: {e}")

    def __del__(self):
        self.try_close()
