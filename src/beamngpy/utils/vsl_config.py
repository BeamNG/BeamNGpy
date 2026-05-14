"""
Parse Vehicle Signal Logger **configuration** CSV files produced by the World Editor
``vslSignalEditor`` (Save configuration) in **BeamNG.tech** / **BeamNG.drive**.

This is not the same as the **output** CSV written by ``vslSignalLogger`` at runtime
(timestamp + signal columns).
"""

from __future__ import annotations

import csv
from pathlib import Path
from typing import List, Tuple

from beamngpy.logging import BNGError
from beamngpy.types import StrDict


def parse_vsl_config_csv(path: str | Path) -> Tuple[List[StrDict], StrDict]:
    """
    Read a VSL config CSV and return signals plus optional metadata.

    Config layout matches ``lua/ge/extensions/editor/vslSignalEditor.lua``:

    * ``signal`` rows: ``groupName``, ``name``, ``description``, ``dataType``
    * ``settings`` rows: key in column ``name`` (e.g. ``filename``, ``frequencySteps``)
    * ``vehicle`` rows: ignored here (model/config hints for the editor only)

    Args:
        path: Local path to the config ``.csv`` (on the machine running BeamNGpy).

    Returns:
        ``(signals, meta)`` where ``meta`` may contain ``output_filepath`` (str) and
        ``frequency_steps`` (int) from ``settings`` rows.

    Raises:
        BNGError: If the file cannot be read or contains no signal rows.
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
                    sig: StrDict = {
                        "name": n,
                        "groupName": g,
                        "description": row[3].strip() if len(row) > 3 else "",
                        "type": row[4].strip() if len(row) > 4 else "Float",
                    }
                    signals.append(sig)
                elif kind == "settings" and len(row) > 3:
                    key = row[2].strip() if len(row) > 2 else ""
                    val = row[3].strip() if len(row) > 3 else ""
                    if key == "filename" and val:
                        meta["output_filepath"] = val
                    elif key == "frequencySteps" and val:
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
