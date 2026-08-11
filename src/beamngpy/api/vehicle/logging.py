from __future__ import annotations

import csv
from pathlib import Path

from .base import VehicleApi
from ...logging import BNGError
from ...types import StrDict


class LoggingApi(VehicleApi):
    """
    Controls the **Vehicle Signal Logger** (``tech/vehicleSignalLogger``) in the simulator,
    same as the World Editor *Vehicle Signal Logger* window.

    The **output** CSV (timestamp + columns) is not the same as an editor **config** CSV.

    **Logging data from sensors is currently not supported through BeamNGpy. Use logging from the
    Vehicle Signal Logger window in the World Editor instead.**
    """

    def start(
        self,
        output_file: str | None = None,
        *,
        signals: list[str] | None = None,
        config_path: str | Path | None = None,
        steps: int | None = None,
    ) -> None:
        """
        Start logging to ``output_file`` (path as seen by the vehicle Lua VM in the
        simulator, typically under the user folder, e.g. ``log/vsl_test.csv``).

        ``config_path`` is a path on the **Python** host to a VSL **configuration** CSV
        (same format as the World Editor export). When set, its signal rows, output path,
        and frequency steps are used as defaults.

        Explicit kwargs always have priority over the config when not ``None``: ``output_file``,
        ``signals``, ``steps``.

        Args:
            output_file: The path to the output CSV file. Defaults to "vsl_signals_log.csv".
            signals: The names of the signals to log (list of strings). Defaults to vehicle positions only.
            config_path: The path to the VSL configuration CSV file. If provided, uses signals, frequency steps,
                         and output path from the config file unless overridden by explicit kwargs.
            steps: The steps N to log at (logs every N-th physics step, int >= 1). Defaults to 1.
        """
        data = _resolve_vsl_start(
            config_path=config_path,
            output_file=output_file,
            signals=signals,
            steps=steps,
        )
        self._send(dict(type="StartVSLLogging", data=data)).ack("StartedVSLLogging")
        self._logger.info(
            "Started Vehicle Signal Logger on %s (%d channels, every %d physics steps).",
            data["filepath"],
            len(data["signals"]),
            data["steps"],
        )

    def stop(self) -> None:
        """Stop logging and unload Vehicle Signal Logger if it was loaded."""
        self._send(dict(type="StopVSLLogging")).ack("StoppedVSLLogging")
        self._logger.info("Stopped Vehicle Signal Logger.")


def _parse_vsl_config_csv(path: str | Path) -> StrDict:
    """
    Read a VSL config CSV and return signals and settings, if available.

    Config layout matches ``lua/ge/extensions/editor/vehicleSignalEditor.lua``:

    * ``signal`` rows: ``groupName``, ``name``, ``description``, ``dataType``
    * ``settings`` rows: key in column ``name``, value in ``description``:

      - ``filename``: output log path
      - ``steps``: log every N physics steps (int >= 1)

    * ``vehicle`` rows: ignored here (model/config hints for the editor only)

    Returns:
        data: StrDict containing keys "signals", "filepath", and "steps" (if available).
    """
    p = Path(path)
    if not p.is_file():
        raise BNGError(f"VSL config CSV not found: {p}")

    signals: list[StrDict] = []
    data: StrDict = {}

    try:
        with p.open(newline="", encoding="utf-8-sig") as f:
            reader = csv.reader(f)
            for row in reader:
                if not row or not row[0]:
                    continue
                kind = row[0].strip()
                if kind == "signal":
                    if len(row) < 3:
                        continue
                    g, n = row[1].strip(), row[2].strip()
                    if not g or not n:
                        continue
                    signals.append(
                        {
                            "name": n,
                            "groupName": g,
                            "description": row[3].strip() if len(row) > 3 else "",
                            "type": row[4].strip() if len(row) > 4 else "",
                        }
                    )
                elif kind == "settings" and len(row) > 3:
                    key = row[2].strip() if len(row) > 2 else ""
                    val = row[3].strip() if len(row) > 3 else ""
                    if key == "filename" and isinstance(val, str) and val:
                        data["filepath"] = val
                    if key == "steps":
                        try:
                            data["steps"] = max(1, int(val))
                        except ValueError:
                            pass
    except OSError as e:
        raise BNGError(f"Failed to read VSL config CSV {p}: {e}") from e

    if not signals:
        raise BNGError(
            f"No signal rows found in {p}. Expected rows with first column 'signal'."
        )
    data["signals"] = signals

    return data


def _resolve_vsl_start(
    *,
    config_path: str | Path | None = None,
    output_file: str | None = None,
    signals: list[str] | None = None,
    steps: int | None = None,
) -> StrDict:
    """Resolve VSL starting arguments using defaults, config or explicit kwargs (in this order)."""
    # Default values
    data: StrDict = dict(
        filepath="vsl_signals_log.csv",
        signals=["vehiclePositionX", "vehiclePositionY", "vehiclePositionZ"],
        steps=1,
    )

    # Override with values from config file
    if config_path is not None:
        config_data = _parse_vsl_config_csv(config_path)
        config_data["signals"] = [signal["name"] for signal in config_data["signals"]]
        data.update(config_data)

    # Override with explicit kwargs
    if steps is not None:
        if not isinstance(steps, int) or steps < 1:
            raise BNGError("steps must be an integer >= 1")
        data["steps"] = steps
    if output_file is not None:
        if not isinstance(output_file, str) or output_file == "":
            raise BNGError("output_file must be a non-empty string")
        data["filepath"] = output_file
    if signals is not None:
        if not isinstance(signals, list) or len(signals) == 0:
            raise BNGError("signals must be a non-empty list")
        if any(not isinstance(name, str) or name == "" for name in signals):
            raise BNGError("signals must be a list of non-empty strings")
        data["signals"] = signals

    return data
