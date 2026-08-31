from __future__ import annotations

from collections.abc import Callable, Iterator
from contextlib import contextmanager
from time import sleep
from typing import Any

import pytest

from beamngpy import BeamNGpy, Scenario, Vehicle


@contextmanager
def loaded_scenario(
    beamng: BeamNGpy,
    *,
    level: str,
    name: str,
    description: str,
    vehicles: list[tuple[Vehicle, dict[str, Any]]],
    label: str,
    hz: int = 60,
) -> Iterator[tuple[BeamNGpy, Vehicle]]:
    """Standard sensor-test scenario load: deterministic Hz, hide HUD, yield ego vehicle."""
    with beamng as bng:
        if not vehicles:
            raise ValueError("loaded_scenario requires at least one vehicle")

        scenario = Scenario(level, name, description=description)
        for vehicle, add_kwargs in vehicles:
            scenario.add_vehicle(vehicle, **add_kwargs)

        scenario.make(bng)
        bng.settings.set_deterministic(hz)
        bng.scenario.load(scenario)
        bng.ui.hide_hud()
        bng.scenario.start()

        print(f"{label} start.")
        yield bng, vehicles[0][0]

        sleep(3)
        print(f"{label} complete.")
        bng.ui.show_hud()


def poll_auto_or_adhoc(
    sensor,
    is_auto: bool,
    *,
    attempt: int | None = None,
    adhoc_wait: float = 3,
):
    """Run automatic poll() or the ad-hoc request/collect flow."""
    suffix = f" {attempt}" if attempt is not None else ""
    if is_auto:
        print(f"\nAutomatic polling attempt{suffix}")
        return sensor.poll()

    print(f"\nAd-hoc poll request attempt{suffix}")
    request_id = sensor.send_ad_hoc_poll_request()
    print("Ad-hoc poll requests sent. Unique request Id number: ", request_id)
    sleep(adhoc_wait)
    print(
        "Is ad-hoc request complete? ",
        sensor.is_ad_hoc_poll_request_ready(request_id),
    )
    return sensor.collect_ad_hoc_poll_request(request_id)


def make_module_scene_fixture(
    scenario_cm: Callable[[BeamNGpy], Iterator[tuple[BeamNGpy, Vehicle]]],
):
    @pytest.fixture(scope="module")
    def scene(beamng: BeamNGpy) -> Iterator[tuple[BeamNGpy, Vehicle]]:
        with scenario_cm(beamng) as loaded:
            yield loaded

    return scene


@contextmanager
def using_sensor(sensor, removed_msg: str | None = None):
    try:
        yield sensor
    finally:
        sensor.remove()
        if removed_msg:
            print(removed_msg)


def latest_sensor_reading(readings):
    if isinstance(readings, list):
        return readings[-1]
    return readings[list(readings.keys())[-1]]


def cycle_visibility(sensor, toggles: int = 3):
    for i in range(toggles * 2):
        sleep(1)
        sensor.set_is_visualised(not (i % 2 == 0))
