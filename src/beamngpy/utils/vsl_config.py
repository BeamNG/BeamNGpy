"""Parse Vehicle Signal Logger Configuration CSV files produced by the World Editor."""

from __future__ import annotations

import csv
from pathlib import Path
from typing import List, NamedTuple, Sequence, Tuple

from beamngpy.logging import BNGError
from beamngpy.types import StrDict


class VslStartResolved(NamedTuple):
    """Resolved options for :meth:`~beamngpy.api.vehicle.logging.LoggingApi.start`."""

    filepath: str
    signals: List[str]
    steps: int 
    static_data: StrDict


def parse_vsl_config_csv(path: str | Path) -> Tuple[List[StrDict], StrDict]:
    """
    Read a VSL config CSV and return signals plus optional metadata.

    Config layout matches ``lua/ge/extensions/editor/vehicleSignalEditor.lua``:

    * ``signal`` rows: ``groupName``, ``name``, ``description``, ``dataType``
    * ``settings`` rows: key in column ``name``, value in ``description``:

      - ``filename``: output log path
      - ``frequencySteps``: log every N physics steps (int >= 1)

    * ``vehicle`` rows: ignored here (model/config hints for the editor only)

    Returns:
        ``(signals, meta)`` where ``meta`` may contain ``output_filepath`` and
        ``steps``.
    """
    p = Path(path)
    if not p.is_file():
        raise BNGError(f"VSL config CSV not found: {p}")

    signals: List[StrDict] = []
    meta: StrDict = {}

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
                    if not key or not val:
                        continue
                    if key == "filename" and isinstance(val, str) and val:
                        meta["output_filepath"] = val
                    elif key == "frequencySteps":
                        try:
                            meta["steps"] = max(1, int(val))
                        except ValueError:
                            pass
    except OSError as e:
        raise BNGError(f"Failed to read VSL config CSV {p}: {e}") from e

    if not signals:
        raise BNGError(
            f"No signal rows found in {p}. Expected rows with first column 'signal'."
        )

    return signals, meta


def resolve_vsl_start(
    *,
    config_path: str | Path | None = None,
    output_file: str | None = None,
    signals: Sequence[str] | None = None,
    steps: int | None = None,
    static_data: StrDict | None = None,
) -> VslStartResolved:
    """Resolve VSL starting arguments using defaults, config or explicit kwargs (in this order)."""
    # Default values
    data = VslStartResolved(
        filepath="vsl_signals_log.csv",
        signals=["vehiclePositionX", "vehiclePositionY", "vehiclePositionZ"],
        steps=1,
        static_data={},
    )

    # Override with values from config file
    if config_path is not None:
        cfg_signals, meta = parse_vsl_config_csv(config_path)
        data.filepath = meta.get("output_filepath", data.filepath)
        data.steps = meta.get("steps", data.steps)
        data.signals = [signal["name"] for signal in cfg_signals]

    # Override with explicit kwargs
    if steps is not None:
        if not isinstance(steps, int) or steps < 1:
            raise BNGError("steps must be an integer >= 1")
        data.steps = steps
    if output_file is not None:
        if not isinstance(output_file, str) or output_file == "":
            raise BNGError("output_file must be a non-empty string")
        data.filepath = output_file
    if signals is not None:
        if not isinstance(signals, Sequence) or len(signals) == 0:
            raise BNGError("signals must be a non-empty list")
        for name in signals:
            if not isinstance(name, str) or name == "":
                raise BNGError("signals must be a list of non-empty strings")
        data.signals = list(signals)
    if static_data is not None:
        if not isinstance(static_data, dict):
            raise BNGError("static_data must be a dictionary")
        data.static_data = static_data

    return data
