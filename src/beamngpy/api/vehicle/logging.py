from __future__ import annotations

from pathlib import Path
from typing import Sequence

from beamngpy.types import StrDict
from beamngpy.utils.vsl_config import resolve_vsl_start

from .base import VehicleApi


class LoggingApi(VehicleApi):
    """
    Controls the **Vehicle Signal Logger** (``tech/vehicleSignalLogger``) in the simulator,
    same as the World Editor *Vehicle Signal Logger* window.

    The **output** CSV (timestamp + columns) is not the same as an editor **config** CSV.
    """

    def start(
        self,
        output_file: str | None = None,
        *,
        signals: Sequence[str] | None = None,
        config_path: str | Path | None = None,
        steps: int | None = None,
        static_data: StrDict | None = None,
    ) -> None:
        """
        Start logging to ``output_file`` (path as seen by the vehicle Lua VM in the
        simulator, typically under the user folder, e.g. ``log/vsl_test.csv``).

        ``config_path`` is a path on the **Python** host to a VSL **configuration** CSV
        (same format as the World Editor export). When set, its signal rows, output path,
        and frequency steps are used as defaults.

        Explicit kwargs always have priority over the config when not ``None``: ``output_file``,
        ``signals``, ``steps``, ``static_data``.

        Args:
            output_file: The path to the output CSV file. Defaults to "vsl_signals_log.csv".
            signals: The names of the signals to log (list of strings). Defaults to vehicle positions only.
            config_path: The path to the VSL configuration CSV file. If provided, uses signals, frequency steps,
                         and output path from the config file unless overridden by explicit kwargs.
            steps: The steps N to log at (logs every N-th physics step, int >= 1). Defaults to 1.
            static_data: The static data to log.
        """
        resolved = resolve_vsl_start(
            config_path=config_path,
            output_file=output_file,
            signals=signals,
            steps=steps,
            static_data=static_data,
        )
        data: StrDict = dict(
            type="StartVSLLogging",
            filepath=resolved.filepath,
            signalNames=resolved.signals,
            frequencySteps=resolved.steps,
            staticData=resolved.static_data,
        )
        self._send(data).ack("StartedVSLLogging")
        self._logger.info(
            "Started Vehicle Signal Logger on %s (%d channels, every %d physics steps).",
            resolved.filepath,
            len(resolved.signals),
            data["frequencySteps"],
        )

    def stop(self) -> None:
        """Stop logging and unload Vehicle Signal Logger if it was loaded."""
        self._send(dict(type="StopVSLLogging")).ack("StoppedVSLLogging")
        self._logger.info("Stopped Vehicle Signal Logger.")
