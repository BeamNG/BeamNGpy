"""Helper function to generate a streaming response from print statements."""

import asyncio
import threading

async def stream_from_print(func, error_string):
    q: asyncio.Queue[str | None] = asyncio.Queue()
    loop = asyncio.get_running_loop()

    class Writer:
        def write(self, data: str):
            loop.call_soon_threadsafe(q.put_nowait, data)
        def flush(self):
            pass

    def run_in_thread():
        try:
            func(Writer())
        except Exception as e:
            # Send the traceback to the client
            err_msg = error_string(e)
            loop.call_soon_threadsafe(q.put_nowait, err_msg)
        finally:
            loop.call_soon_threadsafe(q.put_nowait, None)  # end of stream

    threading.Thread(target=run_in_thread, daemon=True).start()

    while True:
        chunk = await q.get()
        if chunk is None:
            break
        yield chunk
