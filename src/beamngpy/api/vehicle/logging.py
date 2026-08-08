from __future__ import annotations

from pathlib import Path
from typing import Sequence

from beamngpy.types import StrDict
from beamngpy.utils.vsl_config import resolve_vsl_start

from .base import VehicleApi


class LoggingApi(VehicleApi):
    """
    Controls the **Vehicle Signal Logger** (``tech/vslSignalLogger``) in the simulator
    (**BeamNG.tech** / **BeamNG.drive**), same as the World Editor *Vehicle Signal Logger* window.

    Use :meth:`start` with ``signal_names=`` and/or ``config_path=``.
    Use :meth:`stop` to end a session.

    The **output** CSV (timestamp + columns) is not the same as an editor **config** CSV.
    """

    def start(
        self,
        output_file: str | None = None,
        *,
        signal_names: Sequence[str] | None = None,
        config_path: str | Path | None = None,
        frequency_steps: int | None = None,
        static_data: StrDict | None = None,
    ) -> None:
        """
        Start logging to ``output_file`` (path as seen by the vehicle Lua VM in the
        simulator, typically under the user folder, e.g. ``log/vsl_test.csv``).

        ``config_path`` is a path on the **Python** host to a VSL **configuration** CSV
        (same format as the World Editor export). When set, its signal rows and
        ``settings`` are used as defaults. Settings used here:

        * ``frequencySteps`` — log every N physics steps (int ≥ 1)
        * ``filename`` — output path

        Explicit kwargs always win over the config when not ``None``: ``output_file``,
        ``signal_names``, ``frequency_steps``, ``static_data``.
        If ``signal_names`` is omitted, signal rows come from ``config_path``.

        Sampling is steps only (canonical representation shared with the controller):

        * ``frequency_steps`` — log every N physics steps (int ≥ 1). Omitted → use
          ``settings/frequencySteps`` from the config if present, else the simulator
          default (typically 1).

        Convert period/Hz yourself if needed, e.g.
        ``frequency_steps = max(1, round(period_s * 2000))``.

        Without a config CSV, ``output_file`` and ``signal_names`` are required.
        """
        resolved = resolve_vsl_start(
            config_path=config_path,
            output_file=output_file,
            signal_names=signal_names,
            frequency_steps=frequency_steps,
            static_data=static_data,
        )

        data: StrDict = dict(
            type="StartVSLLogging",
            filepath=resolved.filepath,
        )
        if resolved.frequency_steps is not None:
            data["frequencySteps"] = int(resolved.frequency_steps)
        if resolved.static_data is not None:
            data["staticData"] = resolved.static_data
        if resolved.signal_names is not None:
            data["signalNames"] = list(resolved.signal_names)
            n = len(resolved.signal_names)
        else:
            data["signals"] = [dict(s) for s in (resolved.signals or ())]
            n = len(data["signals"])

        self._send(data).ack("StartedVSLLogging")
        if "frequencySteps" in data:
            self._logger.info(
                "Started Vehicle Signal Logger on %s (%d channels, every %d physics steps).",
                resolved.filepath,
                n,
                data["frequencySteps"],
            )
        else:
            self._logger.info(
                "Started Vehicle Signal Logger on %s (%d channels).",
                resolved.filepath,
                n,
            )

    def stop(self) -> None:
        """Stop logging and unload ``vslSignalLogger`` if it was loaded."""
        self._send(dict(type="StopVSLLogging")).ack("StoppedVSLLogging")
        self._logger.info("Stopped Vehicle Signal Logger.")
