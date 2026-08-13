import os
import signal
import subprocess
import time

BEAMNG_PROCESS_NAMES = (
    "BeamNG.drive.x64.exe",
    "BeamNG.drive.exe",
    "BeamNG.tech.x64.exe",
    "BeamNG.tech.exe",
    "BeamNG.x64.exe",
)


def kill_stale_beamng_processes() -> None:
    """Force-kill leftover BeamNG processes (Windows). No-op on other platforms."""
    if os.name != "nt":
        return
    with open(os.devnull, "w") as devnull:
        for name in BEAMNG_PROCESS_NAMES:
            subprocess.call(
                ["taskkill", "/F", "/IM", name],
                stdout=devnull,
                stderr=devnull,
            )
    time.sleep(1.0)


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
