from __future__ import annotations

from multiprocessing.shared_memory import SharedMemory


class BNGSharedMemory(SharedMemory):
    def __init__(self, size: int):
        super().__init__(None, create=True, size=size)
        self.closed = False

    def read(self, size: int) -> memoryview:
        return self.buf[:size]

    def close_and_unlink(self):
        if not self.closed:
            self.close()
            self.unlink()
            self.closed = True

    def __del__(self):
        self.close_and_unlink()