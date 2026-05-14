from __future__ import annotations

from pathlib import Path
from typing import Mapping, Sequence

from beamngpy.logging import BNGError
from beamngpy.types import StrDict
from beamngpy.utils.vsl_config import parse_vsl_config_csv

from .base import VehicleApi


class LoggingApi(VehicleApi):
    """
    Controls the **Vehicle Signal Logger** (``tech/vslSignalLogger``) in the simulator
    (**BeamNG.tech** / **BeamNG.drive**), same as the World Editor *Vehicle Signal Logger* window.

    Use :meth:`start` / :meth:`start_logging` (aliases) with ``signals=``, ``signal_names=``,
    or ``input_signal_file=``. Use :meth:`stop` / :meth:`stop_logging` to end a session.
    Use :meth:`start_from_vsl_config_csv` when the CSV ``settings`` block should supply
    the default output path and stride.

    The **output** CSV (timestamp + columns) is not the same as an editor **config** CSV.
    """

    def set_options_from_json(self, filename: str) -> None:
        raise BNGError(
            "Vehicle Signal Logger does not use the legacy JSON settings API. "
            "Use start(..., signals=..., signal_names=..., or input_signal_file=...) or start_from_vsl_config_csv(...)."
        )

    def write_options_to_json(self, filename: str = "template.json") -> None:
        raise BNGError("Vehicle Signal Logger does not export JSON settings over this API.")

    def start(
        self,
        filepath: str,
        signals: Sequence[Mapping[str, object]] | None = None,
        *,
        signal_names: Sequence[str] | None = None,
        input_signal_file: str | Path | None = None,
        frequency_steps: int = 1,
        static_data: StrDict | None = None,
    ) -> None:
        """
        Start logging to ``filepath`` (path as seen by the vehicle Lua VM in the
        simulator, typically under the user folder, e.g. ``log/vsl_test.csv``).

        Pass **exactly one** of ``signals``, ``signal_names``, or ``input_signal_file``.

        ``input_signal_file`` is a path on the **Python** host to a VSL **configuration**
        CSV (same format as the World Editor export); signal rows are parsed and sent
        as ``signals``. Optional ``frequencySteps`` in that CSV is **not** applied
        automatically—pass ``frequency_steps=`` to match the editor, or read the file
        yourself. Use :meth:`start_from_vsl_config_csv` to pick up default output path
        and stride from the CSV ``settings`` rows.
        """
        if frequency_steps < 1:
            raise BNGError("frequency_steps must be >= 1")

        modes = [signals is not None, signal_names is not None, input_signal_file is not None]
        if sum(bool(m) for m in modes) != 1:
            raise BNGError(
                "Pass exactly one of: signals=<list of dicts>, signal_names=<list of strings>, "
                "or input_signal_file=<path to VSL config CSV>."
            )

        data: StrDict = dict(
            type="StartVSLLogging",
            filepath=filepath,
            frequencySteps=frequency_steps,
        )
        if static_data is not None:
            data["staticData"] = static_data

        if input_signal_file is not None:
            sig_list, _meta = parse_vsl_config_csv(input_signal_file)
            data["signals"] = sig_list
            n = len(sig_list)
        elif signal_names is not None:
            names = [str(s) for s in signal_names]
            if not names:
                raise BNGError("signal_names must be non-empty")
            data["signalNames"] = names
            n = len(names)
        else:
            sig_list = [dict(s) for s in (signals or ())]
            if not sig_list:
                raise BNGError("signals must be a non-empty sequence")
            data["signals"] = sig_list
            n = len(sig_list)

        self._send(data).ack("StartedVSLLogging")
        self._logger.info(
            "Started Vehicle Signal Logger on %s (%d channels, every %d physics steps).",
            filepath,
            n,
            frequency_steps,
        )

    start_logging = start

    def start_from_vsl_config_csv(
        self,
        config_path: str | Path,
        *,
        output_filepath: str | None = None,
        frequency_steps: int | None = None,
        static_data: StrDict | None = None,
    ) -> None:
        """Load editor-exported config CSV on this machine, then :meth:`start` with ``signals=`` (single parse)."""
        signals, meta = parse_vsl_config_csv(config_path)
        out = output_filepath or meta.get("output_filepath")
        if not out or not isinstance(out, str):
            raise BNGError(
                "No output CSV path: pass output_filepath= or add settings/filename in the config CSV."
            )
        freq = (
            frequency_steps
            if frequency_steps is not None
            else int(meta.get("frequency_steps") or 1)
        )
        self.start(
            out,
            signals,
            frequency_steps=max(1, freq),
            static_data=static_data,
        )

    def stop(self) -> None:
        """Stop logging and unload ``vslSignalLogger`` if it was loaded."""
        self._send(dict(type="StopVSLLogging")).ack("StoppedVSLLogging")
        self._logger.info("Stopped Vehicle Signal Logger.")

    stop_logging = stop
