from __future__ import annotations

from multiprocessing.shared_memory import SharedMemory
from beamngpy.logging import bngpy_logger


class BNGSharedMemory(SharedMemory):
    def __init__(self, size: int):
        super().__init__(None, create=True, size=size)
        self.closed = False

    def read(self, size: int) -> memoryview:
        return self.buf[:size]

    def close_and_unlink(self):
        if not self.closed:
            try:
                self.close()
                self.unlink()
                self.closed = True
            except Exception as e:
                bngpy_logger.error(f"Cannot close shared memory. Original error: {e}")

    def __del__(self):
        self.close_and_unlink()
