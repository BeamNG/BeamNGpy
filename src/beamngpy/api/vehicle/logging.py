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
        sampling_period_s: float | None = None,
        sampling_rate_hz: float | None = None,
        static_data: StrDict | None = None,
    ) -> None:
        """
        Start logging to ``output_file`` (path as seen by the vehicle Lua VM in the
        simulator, typically under the user folder, e.g. ``log/vsl_test.csv``).

        ``config_path`` is a path on the **Python** host to a VSL **configuration** CSV
        (same format as the World Editor export). When set, its signal rows and
        ``settings`` are used as defaults. Current editor settings keys:

        * ``samplingPeriodS`` — seconds (preferred)
        * ``samplingRateHz`` — Hz, converted to period when period is absent
        * ``samplingUnit`` — ``s`` or ``Hz`` (UI preference)
        * ``filename`` — output path

        Explicit kwargs always win over the config when not ``None``: ``output_file``,
        ``signal_names``, sampling rate, ``static_data``.
        If ``signal_names`` is omitted, signal rows come from ``config_path``.

        Sampling — same unit choices as the World Editor dropdown (s | Hz). Pass
        **at most one** of:

        * ``sampling_period_s`` — seconds between CSV rows (unit **s**; sent as
          ``samplingPeriodS``). Default is ``0.025`` s (40 Hz) if omitted.
          Clamped to ``[0.0005, 1e4]`` s (max **2000** Hz).
        * ``sampling_rate_hz`` — samples per second (unit **Hz**); converted to period
          via ``1 / rate`` then sent as ``samplingPeriodS`` (e.g. ``40`` → ``0.025`` s).
          Clamped to at most **2000** Hz (``0.0005`` s).

        Without a config CSV, ``output_file`` and ``signal_names`` are required.
        """
        resolved = resolve_vsl_start(
            config_path=config_path,
            output_file=output_file,
            signal_names=signal_names,
            sampling_period_s=sampling_period_s,
            sampling_rate_hz=sampling_rate_hz,
            static_data=static_data,
        )

        data: StrDict = dict(
            type="StartVSLLogging",
            filepath=resolved.filepath,
        )
        if resolved.sampling_period_s is not None:
            data["samplingPeriodS"] = float(resolved.sampling_period_s)
        if resolved.static_data is not None:
            data["staticData"] = resolved.static_data
        if resolved.signal_names is not None:
            data["signalNames"] = list(resolved.signal_names)
            n = len(resolved.signal_names)
        else:
            data["signals"] = [dict(s) for s in (resolved.signals or ())]
            n = len(data["signals"])

        self._send(data).ack("StartedVSLLogging")
        if "samplingPeriodS" in data:
            self._logger.info(
                "Started Vehicle Signal Logger on %s (%d channels, samplingPeriodS=%.5f s).",
                resolved.filepath,
                n,
                data["samplingPeriodS"],
            )
        else:
            self._logger.info(
                "Started Vehicle Signal Logger on %s (%d channels, default sampling period).",
                resolved.filepath,
                n,
            )

    def stop(self) -> None:
        """Stop logging and unload ``vslSignalLogger`` if it was loaded."""
        self._send(dict(type="StopVSLLogging")).ack("StoppedVSLLogging")
        self._logger.info("Stopped Vehicle Signal Logger.")
