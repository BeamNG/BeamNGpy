from __future__ import annotations

from multiprocessing.shared_memory import SharedMemory

def read(shmem: SharedMemory, size: int) -> memoryview:
    return shmem.buf[:size]

def allocate(size: int) -> SharedMemory:
    return SharedMemory(None, create=True, size=size)