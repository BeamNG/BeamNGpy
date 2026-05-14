"""
Smoke-test **Vehicle Signal Logger** (VSL) over BeamNGpy.

"""

import argparse
from pathlib import Path
from time import sleep

from beamngpy import BeamNGpy, Scenario, Vehicle, set_up_simple_logging

_EXAMPLES_DIR = Path(__file__).resolve().parent
BUNDLED_SIGNALS_CSV = _EXAMPLES_DIR / "data" / "mySignalsList.csv"
DEFAULT_SIGNAL_NAMES = (
    "vehicleVelocityX",
    "vehicleVelocityY",
    "vehicleVelocityZ",
    "throttle",
    "brake",
    "steering",
)


def main() -> None:
    set_up_simple_logging()

    parser = argparse.ArgumentParser(description="Smoke-test Vehicle Signal Logger via BeamNGpy.")
    parser.add_argument(
        "--signal-input",
        dest="signal_input",
        choices=("list", "file"),
        default="list",
        metavar="{list,file}",
        help="A=list: log by signal names (default). B=file: log from a VSL configuration CSV.",
    )
    parser.add_argument(
        "--signal-names",
        dest="signal_names",
        nargs="+",
        metavar="NAME",
        default=None,
        help="With --signal-input list: names to log (default: velocity X/Y/Z + throttle/brake/steering). Ignored when --signal-input file.",
    )
    parser.add_argument(
        "--input-signal-file",
        dest="input_signal_file",
        metavar="PATH",
        default=None,
        help="With --signal-input file: VSL CSV path (default: examples/data/mySignalsList.csv next to this script).",
    )
    args = parser.parse_args()

    beamng = BeamNGpy("localhost", 25252)
    beamng.open()

    scenario = Scenario(
        "west_coast_usa",
        "vsl beamngpy test",
        description="Vehicle Signal Logger smoke test (AI traffic needs a map with roads).",
    )
    vehicle = Vehicle("ego_vehicle", model="etk800", license="RED", color="Red")
    scenario.add_vehicle(
        vehicle,
        pos=(-717.121, 101, 250.675),
        rot_quat=(0, 0, 0.3826834, 0.9238795),
        safe_spawn=True,
    )

    scenario.make(beamng)

    beamng.settings.set_deterministic(60)
    beamng.scenario.load(scenario)
    beamng.ui.hide_hud()
    beamng.scenario.start()

    # Vehicle must exist in GE before StartVehicleConnection; control() needs the socket too.
    beamng.vehicles.await_spawn(vehicle.vid)
    vehicle.connect(beamng)

    # Output path is interpreted by the simulator (user folder is a good place).
    out_csv = "log/vsl_beamngpy_test_west_coast_usa.csv"

    if args.signal_input == "list":
        names = args.signal_names if args.signal_names is not None else DEFAULT_SIGNAL_NAMES
        vehicle.logging.start_logging(
            out_csv,
            signal_names=names,
            frequency_steps=50,
        )
    else:
        csv_path = (
            Path(args.input_signal_file).expanduser()
            if args.input_signal_file is not None
            else BUNDLED_SIGNALS_CSV
        )
        if not csv_path.is_file():
            raise FileNotFoundError(
                f"Signal list CSV not found: {csv_path}. "
                "Add examples/data/mySignalsList.csv or pass --input-signal-file PATH with --signal-input file."
            )
        vehicle.logging.start_logging(
            out_csv,
            input_signal_file=csv_path,
            frequency_steps=50,
        )

    vehicle.ai.set_mode("traffic")
    sleep(4.0)

    vehicle.logging.stop_logging()
    vehicle.ai.set_mode("disabled")

    beamng.ui.show_hud()
    print(f"Check for CSV under the simulator user folder, e.g.: .../{out_csv}")
    input("Press Enter to exit...")
    beamng.disconnect()


if __name__ == "__main__":
    main()
