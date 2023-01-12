from __future__ import annotations

import mmap
import platform


def read(shmem: mmap.mmap, buffer_size: int) -> bytes:
    shmem.seek(0)
    return shmem.read(buffer_size)

def allocate(size: int, handle: str) -> mmap.mmap:
    os = platform.system()
    if os == 'Linux':
        raise NotImplementedError('Shared memory not available on Linux clients!')
    return mmap.mmap(0, size, handle)