"""
Parse Vehicle Signal Logger **configuration** CSV files produced by the World Editor
``vslSignalEditor`` (Save configuration) in **BeamNG.tech** / **BeamNG.drive**.

This is not the same as the **output** CSV written by ``vslSignalLogger`` at runtime
(timestamp + signal columns).
"""

from __future__ import annotations

import csv
from pathlib import Path
from typing import List, NamedTuple, Sequence, Tuple

from beamngpy.logging import BNGError
from beamngpy.types import StrDict

# Current editor settings keys (vslSignalEditor save format only).
# settings row: type=settings, name=<key>, description=<value>

# Max sample rate 2000 Hz → min period 0.0005 s (physics rate ceiling).
SAMPLE_RATE_MAX_HZ = 2000.0
SAMPLE_PERIOD_MIN_S = 1.0 / SAMPLE_RATE_MAX_HZ  # 0.0005
SAMPLE_PERIOD_MAX_S = 1e4


class VslStartResolved(NamedTuple):
    """Resolved options for :meth:`~beamngpy.api.vehicle.logging.LoggingApi.start`."""

    filepath: str
    signals: List[StrDict] | None
    signal_names: List[str] | None
    sampling_period_s: float | None
    static_data: StrDict | None


def clamp_sampling_period_s(period_s: float) -> float:
    """Clamp period to [0.0005, 1e4] s (max 2000 Hz)."""
    return min(SAMPLE_PERIOD_MAX_S, max(SAMPLE_PERIOD_MIN_S, float(period_s)))


def sampling_rate_hz_to_period_s(rate_hz: float) -> float:
    """Convert sample rate (Hz) to period (s). Caps at 2000 Hz (0.0005 s)."""
    rate = float(rate_hz)
    if rate <= 0:
        raise BNGError("sampling_rate_hz must be > 0")
    rate = min(SAMPLE_RATE_MAX_HZ, rate)
    return clamp_sampling_period_s(1.0 / rate)


def _normalize_sampling_unit(raw: str) -> str | None:
    u = raw.strip().lower()
    if u == "s":
        return "s"
    if u == "hz":
        return "Hz"
    return None


def _meta_from_sampling_settings(
    *,
    period: float | None,
    rate_hz: float | None,
    unit: str | None,
) -> StrDict:
    """
    Build meta sampling fields.

    Priority for the native period used when starting the logger:
    1. ``samplingPeriodS`` (seconds)
    2. ``samplingRateHz`` → ``1 / rate``
    """
    meta: StrDict = {}
    if unit is not None:
        meta["sampling_unit"] = unit

    if period is not None:
        period_s = clamp_sampling_period_s(period)
        meta["sampling_period_s"] = period_s
        meta["sampling_rate_hz"] = 1.0 / period_s
        return meta

    if rate_hz is not None:
        period_s = sampling_rate_hz_to_period_s(rate_hz)
        meta["sampling_period_s"] = period_s
        meta["sampling_rate_hz"] = 1.0 / period_s
        if unit is None:
            meta["sampling_unit"] = "Hz"
        return meta

    return meta


def parse_vsl_config_csv(path: str | Path) -> Tuple[List[StrDict], StrDict]:
    """
    Read a VSL config CSV and return signals plus optional metadata.

    Config layout matches ``lua/ge/extensions/editor/vslSignalEditor.lua``:

    * ``signal`` rows: ``groupName``, ``name``, ``description``, ``dataType``
    * ``settings`` rows: key in column ``name``, value in ``description``
      (same keys as World Editor save):

      - ``filename`` → output log path
      - ``samplingPeriodS`` → seconds between samples (preferred)
      - ``samplingRateHz`` → samples per second (converted when period absent)
      - ``samplingUnit`` → ``s`` or ``Hz`` (UI preference; optional)

    * ``vehicle`` rows: ignored here (model/config hints for the editor only)

    Args:
        path: Local path to the config ``.csv`` (on the machine running BeamNGpy).

    Returns:
        ``(signals, meta)`` where ``meta`` may contain:

        * ``output_filepath`` (str)
        * ``sampling_period_s`` (float) — native period for the logger
        * ``sampling_rate_hz`` (float) — derived or from CSV when known
        * ``sampling_unit`` (``\"s\"`` | ``\"Hz\"``)

    Raises:
        BNGError: If the file cannot be read or contains no signal rows.
    """
    p = Path(path)
    if not p.is_file():
        raise BNGError(f"VSL config CSV not found: {p}")

    signals: List[StrDict] = []
    meta: StrDict = {}
    period_from_csv: float | None = None
    rate_from_csv: float | None = None
    unit_from_csv: str | None = None

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
                    if not key or not val:
                        continue
                    if key == "filename":
                        meta["output_filepath"] = val
                    elif key == "samplingPeriodS":
                        try:
                            period_from_csv = float(val)
                        except ValueError:
                            pass
                    elif key == "samplingRateHz":
                        try:
                            rate_from_csv = float(val)
                        except ValueError:
                            pass
                    elif key == "samplingUnit":
                        unit_from_csv = _normalize_sampling_unit(val)
    except OSError as e:
        raise BNGError(f"Failed to read VSL config CSV {p}: {e}") from e

    if not signals:
        raise BNGError(
            f"No signal rows found in {p}. Expected rows with first column 'signal'."
        )

    meta.update(
        _meta_from_sampling_settings(
            period=period_from_csv,
            rate_hz=rate_from_csv,
            unit=unit_from_csv,
        )
    )

    return signals, meta


def resolve_vsl_start(
    *,
    config_path: str | Path | None = None,
    output_file: str | None = None,
    signal_names: Sequence[str] | None = None,
    sampling_period_s: float | None = None,
    sampling_rate_hz: float | None = None,
    static_data: StrDict | None = None,
) -> VslStartResolved:
    """
    Merge a VSL config CSV with explicit start overrides.

    Values from ``config_path`` are defaults. Any explicit argument that is not
    ``None`` wins (output path, signal names, sampling rate, static data).

    Full signal rows come only from ``config_path``. Use ``signal_names`` to log by
    name without a config file, or to override the config's signal list.

    Pass at most one of ``sampling_period_s`` / ``sampling_rate_hz`` as overrides.
    ``sampling_rate_hz`` is converted to a period (``1 / rate``) before send — same as
    the World Editor unit dropdown (s | Hz).

    When overrides are omitted, config resolution prefers:

    1. ``settings/samplingPeriodS``
    2. ``settings/samplingRateHz`` (converted)
    """
    if sampling_period_s is not None and sampling_rate_hz is not None:
        raise BNGError("Pass only one of sampling_period_s= or sampling_rate_hz=.")

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

    period = sampling_period_s
    if sampling_rate_hz is not None:
        period = sampling_rate_hz_to_period_s(sampling_rate_hz)
    if period is None and "sampling_period_s" in meta:
        period = float(meta["sampling_period_s"])  # type: ignore[arg-type]
    if period is not None:
        period = clamp_sampling_period_s(period)

    return VslStartResolved(
        filepath=out,
        signals=resolved_signals,
        signal_names=resolved_names,
        sampling_period_s=period,
        static_data=static_data,
    )
