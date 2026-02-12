"""Optimization module for tuning vehicle parameters to match targets."""

from beamngpy import BeamNGpy, Scenario, Vehicle
from .generation import generate_car
from .models import TemplateVehicle
from .models import OptimizationVariables, OutputValues, TargetValues, VehicleInputs, OptimizationEnabled
from .newton import solve
import numpy as np
import os
from time import time
import shutil
from time import sleep


def generate_optimal_car(id: str, vehicle: TemplateVehicle, user_folder: str, install_path: str, file=None):
    """Optimize and generate template car using the given vehicle."""
    if vehicle.optimization.required():
        print(f"Starting optimization using BeamNG. Please wait...", file=file, flush=True)
        var, outputs = optimize(id, vehicle, user_folder, install_path, file=file)
        print_result(var, outputs, vehicle.optimization.targets, vehicle.optimization.enabled, file=file)
    else:
        var = vehicle.optimization.variables
    inputs = vehicle.to_inputs(id, var)
    generate_car(inputs, user_folder)


def optimize(
    id: str, vehicle: TemplateVehicle, user_folder: str, install_path: str, file=None
) -> tuple[OptimizationVariables, OutputValues]:
    """Optimizes the variables of the given vehicle to match the defined targets."""
    with BeamNG(id, vehicle, user_folder, install_path) as beamng:
        # Measure distance from lowest tire node to ground from a high position
        z_ref = 10.0
        beamng.load_scenario(pos=(0, 0, z_ref), pause=True)
        node_info = beamng.v.get_node_info()
        z_lowest = min(n['pos'][2] for n in node_info)
        z_ref_ground = z_ref - z_lowest
        beamng.beamng.resume()

        # Stick the lowest tire node to the ground as initial position
        beamng.load_scenario(pos=(0, 0, z_ref_ground), pause=True)
        v = beamng.v
        bng = beamng.beamng

        def f(x):
            var = vehicle.optimization.variables.from_numpy(x, vehicle.optimization.enabled)
            inputs = vehicle.to_inputs(id, var)
            outputs = measure_output_values(v, inputs, bng, user_folder)
            dev = deviations(vehicle.optimization.targets, outputs, vehicle.optimization.enabled)
            print(f"evaluate: x: {x}, f(x): {dev}", file=file, flush=True)
            return dev, (var, outputs)

        max_iter = 100
        x_initial = vehicle.optimization.variables.to_numpy(vehicle.optimization.enabled)
        x_min = vehicle.optimization.variables.min_values(vehicle.optimization.enabled)
        x_max = vehicle.optimization.variables.max_values(vehicle.optimization.enabled)
        dx = vehicle.optimization.variables.min_step(vehicle.optimization.enabled)
        f_tol = vehicle.optimization.targets.tolerance(vehicle.optimization.enabled)
        _, _, solved, reason, extra = solve(f, x_initial, dx, x_min, x_max, f_tol, max_iter, file=file)
        if not solved:
            raise ValueError(f"Optimization failed: {reason}")
        var_sol, outputs_sol = extra
        print(f"Optimization successful. {reason}", file=file, flush=True)

        return var_sol, outputs_sol


class BeamNG:
    """Context manager for optimization with BeamNG."""

    def __init__(self, id: str, vehicle: TemplateVehicle, user_folder: str, install_path: str):
        self.user_folder = user_folder
        self.beamng = BeamNGpy("localhost", 25252, home=install_path, user=user_folder)
        self.vehicle_id = id
        self.vehicle = vehicle
        self.v: Vehicle | None = None

    def load_scenario(self, pos=(0, 0, 0), pause=False):
        scenario = Scenario("smallgrid", "template_car_parametrization")
        v = Vehicle("template_car", model=self.vehicle_id)
        scenario.add_vehicle(v, pos=pos)
        scenario.make(self.beamng)
        if pause:
            self.beamng.pause()
        self.beamng.scenario.load(scenario)
        self.beamng.scenario.start()
        self.v = v

    def __enter__(self):
        self.beamng.open([], "-gfx", "vk")
        self.beamng.settings.change("fpsLimitEnabled", False)
        self.beamng.settings.change("fpsLimitBackgroundEnabled", False)
        self.beamng.set_deterministic(speed_factor=1)
        cleanup(self.user_folder, self.vehicle_id)
        inputs = self.vehicle.to_inputs(self.vehicle_id, self.vehicle.optimization.variables)
        generate_car(inputs, self.user_folder, use_mods_folder=False)
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        cleanup(self.user_folder, self.vehicle_id)
        self.beamng.close()


def deviations(targets: TargetValues, outputs: OutputValues, enabled: OptimizationEnabled) -> np.ndarray:
    """Compute deviations from targets."""
    v = []
    for name, is_enabled in enabled.model_dump().items():
        if is_enabled:
            value = getattr(outputs, name)
            target = getattr(targets, name)
            v.append(value - target)
    return np.array(v, dtype=np.float64)


def measure_output_values(v: Vehicle, inputs: VehicleInputs, beamng: BeamNGpy, user_folder: str) -> OutputValues:
    """Generate vehicle, reload it and measure mass after settling."""
    generate_car(inputs, user_folder, use_mods_folder=False)
    beamng.pause()
    beamng.scenario.restart()
    reload_vehicle(v)
    data = measure_samples(v, beamng)
    last_sample = data[-1]
    p = get_mass_properties(last_sample)
    _, cg_y, cg_z = p["center_of_gravity"]
    inertia_yaw = p["inertia"]["z"]
    inertia_pitch = p["inertia"]["x"]
    inertia_roll = p["inertia"]["y"]
    return OutputValues(mass=p["mass"], cg_y=cg_y, cg_z=cg_z, inertia_yaw=inertia_yaw, inertia_pitch=inertia_pitch, inertia_roll=inertia_roll)


def get_mass_properties(sample: dict):
    """Get mass, inertia about center of gravity and center of gravity with respect to front axle centerpoint."""
    p = sample["mass_properties"]
    node_info = sample["nodes"]
    cog = np.array(p["center_of_gravity"])
    fw1l_pos = np.array(node_info[0]["pos"])
    fw1r_pos = np.array(node_info[1]["pos"])
    front_axle_center = (fw1l_pos + fw1r_pos) * 0.5
    cog_front_axle = cog - front_axle_center
    return {
        "mass": p["mass"],
        "center_of_gravity": tuple(cog_front_axle.tolist()),
        "inertia": p["inertia"],
    }


def print_result(
    var: OptimizationVariables, outputs: OutputValues, targets: TargetValues, enabled: OptimizationEnabled, file=None
) -> None:
    s = ""
    for name, is_enabled in enabled.model_dump().items():
        output = getattr(outputs, name)
        target = getattr(targets, name)
        factor = getattr(var, f"{name}_factor")
        deviation = output - target
        pretty_name = OutputValues.model_fields[name].title
        if is_enabled:
            s += f"- {pretty_name}: {output} (target: {target}, deviation: {deviation}, factor: {factor})\n"
        else:
            s += f"- {pretty_name}: {output} (no target, factor: {factor})\n"
    print(s, end="", file=file, flush=True)


def reload_vehicle(v: Vehicle, debug=False):
    """Reloads the given vehicle's files from disk and re-spawns it (equivalent to Ctrl+R in BeamNG).

    This will temporarily switch the focus to the given vehicle as a side effect.
    TODO: Move this to BeamNGpy.
    """
    beamng = v._ge_api._beamng
    vehicles_api = beamng.vehicles
    t = time()
    player_vid = vehicles_api.get_player_vehicle_id()['vid']
    if debug:
        print(f"get player vehicle id took {time() - t:.3f} seconds")
    needs_switch = player_vid != v.vid
    if needs_switch:
        t = time()
        vehicles_api.switch(v)
        if debug:
            print(f"switch took {time() - t:.3f} seconds")
    cmd = "return core_vehicle_manager.reloadVehicle(0)"
    beamng.queue_lua_command(cmd)
    t = time()
    vehicles_api.await_reconnect(v.vid)
    if debug:
        print(f"await spawn took {time() - t:.3f} seconds")
    t = time()
    v.connect(beamng, tries=1)  # needs re-connect after reload
    if debug:
        print(f"connect took {time() - t:.3f} seconds")
    if needs_switch:
        t = time()
        vehicles_api.switch(player_vid)
        if debug:
            print(f"switch back took {time() - t:.3f} seconds")


def measure_samples(v: Vehicle, beamng: BeamNGpy, steps=100, final_time=10.0):
    quantities = [
        {"name": "mass_properties", "data": {"without_wheels": False}},
        {"name": "node_info", "key": "nodes", "data": {"nodes": ["fw1l", "fw1r"]}},
    ]
    start_recording(v, quantities=quantities, steps=steps)
    beamng.resume()
    data = []
    num_empty = 0
    while not data or data[-1]['time'] < final_time:
        sleep(50e-3)
        d = fetch_recorded_data(v)
        data.extend(d)
        if len(d) == 0:
            num_empty += 1
        if num_empty > 20:
            msg = [
                "The simulation was hanging for too long.",
                "Possibly an instability has been detected.",
                "Your chosen parameter values might be too extreme.",
                "Try using different values.",
            ]
            raise RuntimeError(" ".join(msg))
    stop_recording(v)
    data = [d for d in data if d['time'] <= final_time]
    return data


def start_recording(v: Vehicle, quantities=None, steps=1) -> None:
    # TODO: Move this to BeamNGpy.
    self = v._root
    if quantities is None:
        quantities = []
    data = dict(type="StartRecording", quantities=quantities, steps=steps)
    self._send(data).ack("StartRecording")


def fetch_recorded_data(v: Vehicle) -> list[dict]:
    # TODO: Move this to BeamNGpy.
    self = v._root
    data = dict(type="FetchRecordedData")
    response = self._send(data).recv("FetchRecordedData")
    data = response["data"]
    return data


def stop_recording(v: Vehicle) -> list[dict]:
    # TODO: Move this to BeamNGpy.
    self = v._root
    data = dict(type="StopRecording")
    response = self._send(data).recv("StopRecording")
    data = response["data"]
    return data


def cleanup(user_folder, vehicle_id):
    """Remove vehicle files from disk."""
    dirs = [
        os.path.join(user_folder, "current", "mods", "unpacked", vehicle_id),
        os.path.join(user_folder, "current", "vehicles", vehicle_id),
    ]
    for dir in dirs:
        if os.path.exists(dir):
            shutil.rmtree(dir)
