import os
import signal
import subprocess


def kill_process_tree(process: subprocess.Popen, group: bool = False):
    """Kill a process tree."""
    if process.stdin:
        process.stdin.close()
    if os.name == "nt":
        with open(os.devnull, "w") as devnull:
            subprocess.call(
                ["taskkill", "/F", "/T", "/PID", str(process.pid)],
                stdout=devnull,
                stderr=devnull,
            )
            process.wait()
    else:
        try:
            if group:
                os.killpg(os.getpgid(process.pid), signal.SIGTERM)
            else:
                os.kill(process.pid, signal.SIGTERM)
            process.wait()
        except Exception:
            pass
