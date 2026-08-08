"""
Parse Vehicle Signal Logger **configuration** CSV files produced by the World Editor
``vslSignalEditor`` (Save configuration) in **BeamNG.tech** / **BeamNG.drive**.

This is not the same as the **output** CSV written by ``vslSignalLogger`` at runtime
(timestamp + signal columns).

Sampling is always ``frequencySteps`` (int ≥ 1): log every N physics steps.
s/Hz conversion is not performed here — pass steps, or convert yourself.
"""

from __future__ import annotations

import csv
from pathlib import Path
from typing import List, NamedTuple, Sequence, Tuple

from beamngpy.logging import BNGError
from beamngpy.types import StrDict


class VslStartResolved(NamedTuple):
    """Resolved options for :meth:`~beamngpy.api.vehicle.logging.LoggingApi.start`."""

    filepath: str
    signals: List[StrDict] | None
    signal_names: List[str] | None
    frequency_steps: int | None
    static_data: StrDict | None


def parse_vsl_config_csv(path: str | Path) -> Tuple[List[StrDict], StrDict]:
    """
    Read a VSL config CSV and return signals plus optional metadata.

    Config layout matches ``lua/ge/extensions/editor/vslSignalEditor.lua``:

    * ``signal`` rows: ``groupName``, ``name``, ``description``, ``dataType``
    * ``settings`` rows: key in column ``name``, value in ``description``:

      - ``filename`` → output log path
      - ``frequencySteps`` → log every N physics steps (int ≥ 1)

    * ``vehicle`` rows: ignored here (model/config hints for the editor only)

    Returns:
        ``(signals, meta)`` where ``meta`` may contain ``output_filepath`` and
        ``frequency_steps``.
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
                            "type": row[4].strip() if len(row) > 4 else "Float",
                        }
                    )
                elif kind == "settings" and len(row) > 3:
                    key = row[2].strip() if len(row) > 2 else ""
                    val = row[3].strip() if len(row) > 3 else ""
                    if not key or not val:
                        continue
                    if key == "filename":
                        meta["output_filepath"] = val
                    elif key == "frequencySteps":
                        try:
                            meta["frequency_steps"] = max(1, int(float(val)))
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
    signal_names: Sequence[str] | None = None,
    frequency_steps: int | None = None,
    static_data: StrDict | None = None,
) -> VslStartResolved:
    """
    Merge a VSL config CSV with explicit start overrides.

    Explicit kwargs that are not ``None`` always win. Sampling is steps only:
    ``frequency_steps`` or ``settings/frequencySteps`` from the config CSV.
    """
    if frequency_steps is not None:
        steps = int(frequency_steps)
        if steps < 1:
            raise BNGError("frequency_steps must be >= 1")
    else:
        steps = None

    meta: StrDict = {}
    cfg_signals: List[StrDict] | None = None
    if config_path is not None:
        cfg_signals, meta = parse_vsl_config_csv(config_path)

    out: str | None = output_file if output_file is not None else None
    if out is None:
        raw = meta.get("output_filepath")
        out = raw if isinstance(raw, str) and raw else None
    if not out:
        raise BNGError(
            "No output CSV path: pass output_file= or add settings/filename in the config CSV "
            "(config_path=)."
        )

    resolved_signals: List[StrDict] | None = None
    resolved_names: List[str] | None = None
    if signal_names is not None:
        names = [str(s) for s in signal_names]
        if not names:
            raise BNGError("signal_names must be non-empty")
        resolved_names = names
    elif cfg_signals is not None:
        resolved_signals = cfg_signals
    else:
        raise BNGError(
            "No signals: pass signal_names= or config_path= to a VSL config CSV."
        )

    if steps is None and "frequency_steps" in meta:
        steps = int(meta["frequency_steps"])  # type: ignore[arg-type]

    return VslStartResolved(
        filepath=out,
        signals=resolved_signals,
        signal_names=resolved_names,
        frequency_steps=steps,
        static_data=static_data,
    )
