"""Functions for generation of the template car mod files."""

import os
import shutil

from jinja2 import Template
from .models import VehicleInputs
from .utils import validate_id, read_file, jbeam_template_dir


def generate_car(inputs: VehicleInputs, user_folder: str, use_mods_folder=True):
    """Generate template car using the given vehicle and optimization variables."""

    id = inputs.id
    validate_id(id)
    name = inputs.name
    if name == "":
        name = id
    parameters = inputs.parameters.model_dump()
    mass_factor = inputs.variables.mass_factor
    cg_y_factor = inputs.variables.cg_y_factor
    cg_z_factor = inputs.variables.cg_z_factor
    inertia_yaw_factor = inputs.variables.inertia_yaw_factor
    inertia_pitch_factor = inputs.variables.inertia_pitch_factor
    inertia_roll_factor = inputs.variables.inertia_roll_factor
    weight_body_roof = 10*mass_factor
    weight_body_main = 20*mass_factor
    structure = inputs.structure
    body_shape = structure.body_shape.value
    suspension_front = structure.suspension_front.value
    suspension_rear = structure.suspension_rear.value
    color = get_default_color(body_shape)

    # === TEMPLATES ===
    template_dir = jbeam_template_dir()
    template_car_jbeam = read_file(f'{template_dir}/template_car.jbeam')

    template_car_body_sedan_jbeam = read_file(f'{template_dir}/body/sedan/template_car_body_sedan.jbeam')
    
    template_car_body_wagon_jbeam = read_file(f'{template_dir}/body/wagon/template_car_body_wagon.jbeam')

    template_car_body_coupe_jbeam = read_file(f'{template_dir}/body/coupe/template_car_body_coupe.jbeam')

    template_car_body_pickup_jbeam = read_file(f'{template_dir}/body/pickup/template_car_body_pickup.jbeam')

    template_car_doublewishbone_R_jbeam = read_file(f'{template_dir}/suspension/rear/template_car_doublewishbone_R.jbeam')

    template_car_hubs_F_jbeam = read_file(f'{template_dir}/common/template_car_hubs_F.jbeam')

    template_car_hubs_R_jbeam = read_file(f'{template_dir}/common/template_car_hubs_R.jbeam')

    template_car_liveaxle_4link_R_jbeam = read_file(f'{template_dir}/suspension/rear/template_car_liveaxle_4link_R.jbeam')

    template_car_liveaxle_4link_dually_R_jbeam = read_file(f'{template_dir}/suspension/rear/template_car_liveaxle_4link_dually_R.jbeam')

    template_car_multilink_R_jbeam = read_file(f'{template_dir}/suspension/rear/template_car_multilink_R.jbeam')

    template_car_semitrailing_R_jbeam = read_file(f'{template_dir}/suspension/rear/template_car_semitrailing_R.jbeam')

    template_car_strut_R_jbeam = read_file(f'{template_dir}/suspension/rear/template_car_strut_R.jbeam')

    template_car_torsionbeam_R_jbeam = read_file(f'{template_dir}/suspension/rear/template_car_torsionbeam_R.jbeam')

    info_json = read_file(f'{template_dir}/info.json')
    info_default_json = read_file(f'{template_dir}/info_default.json')
    default_pc = read_file(f'{template_dir}/default.pc')

    # === FILES TO GENERATE ===
    files = {
        "template_car.jbeam": template_car_jbeam,
        "template_car_hubs_F.jbeam": template_car_hubs_F_jbeam,
        "template_car_hubs_R.jbeam": template_car_hubs_R_jbeam,
        "template_car_brakes.jbeam": read_file(f'{template_dir}/common/template_car_brakes.jbeam'),
        "template_car_engine.jbeam": read_file(f'{template_dir}/common/template_car_engine.jbeam'),
        "info.json": info_json,
        "info_default.json": info_default_json,
        "default.pc": default_pc,
        "template_car_front.dae": read_file(f'{template_dir}/common/template_car_front.dae'),
        "main.materials.json": read_file(f'{template_dir}/main.materials.json'),
    }
    files_copy = {
        "default.jpg": f"{template_dir}/body/{body_shape}/default.jpg",
    }

    # Add body shape file based on configuration
    if body_shape == "sedan":
        node_positions = node_positions_sedan(inputs.parameters)
        files["template_car_body_sedan.jbeam"] = template_car_body_sedan_jbeam
    elif body_shape == "wagon":
        node_positions = node_positions_wagon(inputs.parameters)
        files["template_car_body_wagon.jbeam"] = template_car_body_wagon_jbeam
    elif body_shape == "coupe":
        node_positions = node_positions_coupe(inputs.parameters)
        files["template_car_body_coupe.jbeam"] = template_car_body_coupe_jbeam
    elif body_shape == "pickup":
        node_positions = node_positions_pickup(inputs.parameters)
        files["template_car_body_pickup.jbeam"] = template_car_body_pickup_jbeam
    node_positions = transform_node_dictionary(node_positions, inputs.parameters, weight_body_main, weight_body_roof, cg_y_factor, cg_z_factor, inertia_yaw_factor, inertia_pitch_factor, inertia_roll_factor)

    # Add DAE files based on body shape
    if body_shape == "sedan":
        files["template_car_sedan_middle.dae"] = read_file(f'{template_dir}/body/sedan/template_car_sedan_middle.dae')
        files["template_car_sedan_rear.dae"] = read_file(f'{template_dir}/body/sedan/template_car_sedan_rear.dae')
    elif body_shape == "wagon":
        files["template_car_wagon_middle.dae"] = read_file(f'{template_dir}/body/wagon/template_car_wagon_middle.dae')
        files["template_car_wagon_rear.dae"] = read_file(f'{template_dir}/body/wagon/template_car_wagon_rear.dae')
    elif body_shape == "coupe":
        files["template_car_coupe_middle.dae"] = read_file(f'{template_dir}/body/coupe/template_car_coupe_middle.dae')
        files["template_car_coupe_rear.dae"] = read_file(f'{template_dir}/body/coupe/template_car_coupe_rear.dae')
    elif body_shape == "pickup":
        files["template_car_pickup_middle.dae"] = read_file(f'{template_dir}/body/pickup/template_car_pickup_middle.dae')
        files["template_car_pickup_rear.dae"] = read_file(f'{template_dir}/body/pickup/template_car_pickup_rear.dae')

    # Add rear suspension file based on configuration
    if suspension_rear == "doublewishbone":
        files["template_car_doublewishbone_R.jbeam"] = template_car_doublewishbone_R_jbeam
    elif suspension_rear == "liveaxle_4link":
        files["template_car_liveaxle_4link_R.jbeam"] = template_car_liveaxle_4link_R_jbeam
    elif suspension_rear == "liveaxle_4link_dually":
        files["template_car_liveaxle_4link_dually_R.jbeam"] = template_car_liveaxle_4link_dually_R_jbeam
    elif suspension_rear == "multilink":
        files["template_car_multilink_R.jbeam"] = template_car_multilink_R_jbeam
    elif suspension_rear == "semitrailing":
        files["template_car_semitrailing_R.jbeam"] = template_car_semitrailing_R_jbeam
    elif suspension_rear == "strut":
        files["template_car_strut_R.jbeam"] = template_car_strut_R_jbeam
    elif suspension_rear == "torsionbeam":
        files["template_car_torsionbeam_R.jbeam"] = template_car_torsionbeam_R_jbeam

    # Add front suspension file based on configuration
    if suspension_front == "doublewishbone":
        files["template_car_doublewishbone_F.jbeam"] = read_file(f'{template_dir}/suspension/front/template_car_doublewishbone_F.jbeam')
    elif suspension_front == "liveaxle_4link":
        files["template_car_liveaxle_4link_F.jbeam"] = read_file(f'{template_dir}/suspension/front/template_car_liveaxle_4link_F.jbeam')
    elif suspension_front == "strut":
        files["template_car_strut_F.jbeam"] = read_file(f'{template_dir}/suspension/front/template_car_strut_F.jbeam')

    # === UNPACKED FOLDER CREATION ===
    if not os.path.exists(user_folder):
        raise FileNotFoundError(f"User folder not found: {user_folder}. Update the `user_folder` variable in the `settings.json` file, or run the game once to use the default folder location")

    if use_mods_folder:
        # Create vehicle-specific folder inside unpacked folder
        mod_path = os.path.join(user_folder, "current", "mods", "unpacked", id)
        # Delete existing vehicle folder if it exists to clear old files that can conflict
        if os.path.exists(mod_path):
            shutil.rmtree(mod_path)
        vehicle_files_dir = os.path.join(mod_path, "vehicles", id)
    else:
        vehicle_files_dir = os.path.join(user_folder, "current", "vehicles", id)
        # don't remove existing files in this case, otherwise BeamNG triggers an unintended reload of the vehicle

    os.makedirs(vehicle_files_dir, exist_ok=True)
    for filename, template_str in files.items():
        template = Template(template_str)
        content = template.render(**parameters, **node_positions, name=name, body_shape=body_shape, suspension_front=suspension_front, suspension_rear=suspension_rear, color=color)
        file_path = os.path.join(vehicle_files_dir, filename)
        if use_mods_folder:
            with open(file_path, 'w', encoding='utf-8') as f:
                f.write(content)
        else:
            # avoid copying of files that did not change, because this might trigger an unintended reload of the vehicle
            old_content = read_file(file_path) if os.path.exists(file_path) else None
            if content != old_content:
                with open(file_path, 'w', encoding='utf-8') as f:
                    f.write(content)
    for filename, source_path in files_copy.items():
        if use_mods_folder:
            shutil.copy(source_path, os.path.join(vehicle_files_dir, filename))
        else:
            # avoid re-copying static files, because this might trigger an unintended reload of the vehicle
            file = os.path.join(vehicle_files_dir, filename)
            if not os.path.exists(file):
                shutil.copy(source_path, file)


def node_positions_sedan(parameters):
    return {
        'f1': [
            0.000,
            -parameters.front_overhang,
            0.000,
        ],
        'f1l': [
            parameters.body_width * 0.3,
            -parameters.front_overhang + 0.03,
            0.000,
        ],
        'f1ll': [
            parameters.body_width * 0.5,
            -parameters.front_overhang + 0.11,
            0.000,
        ],
        'f1r': [
            parameters.body_width * -0.3,
            -parameters.front_overhang + 0.03,
            0.000,
        ],
        'f1rr': [
            parameters.body_width * -0.5,
            -parameters.front_overhang + 0.11,
            0.000,
        ],
        'f2': [
            0.000,
            0.000,
            0.000,
        ],
        'f2l': [
            parameters.body_width * 0.5,
            0.000,
            parameters.body_height * 0.473,
        ],
        'f2r': [
            parameters.body_width * -0.5,
            0.000,
            parameters.body_height * 0.473,
        ],
        'f3': [
            0.000,
            -parameters.front_overhang + 0.08,
            parameters.body_height * 0.545,
        ],
        'f3l': [
            parameters.body_width * 0.3,
            -parameters.front_overhang + 0.13,
            parameters.body_height * 0.545,
        ],
        'f3ll': [
            parameters.body_width * 0.5,
            -parameters.front_overhang + 0.19,
            parameters.body_height * 0.536,
        ],
        'f3r': [
            parameters.body_width * -0.3,
            -parameters.front_overhang + 0.13,
            parameters.body_height * 0.545,
        ],
        'f3rr': [
            parameters.body_width * -0.5,
            -parameters.front_overhang + 0.19,
            parameters.body_height * 0.536,
        ],
        'f4': [
            0.000,
            0.000,
            parameters.body_height * 0.591,
        ],
        'f4l': [
            parameters.body_width * 0.5,
            0.000,
            parameters.body_height * 0.582,
        ],
        'f4r': [
            parameters.body_width * -0.5,
            0.000,
            parameters.body_height * 0.582,
        ],
        'b1': [
            0.000,
            parameters.wheelbase * 0.20,
            0.000,
        ],
        'b1l': [
            parameters.body_width * 0.5,
            parameters.wheelbase * 0.20,
            0.000,
        ],
        'b1r': [
            parameters.body_width * -0.5,
            parameters.wheelbase * 0.20,
            0.000,
        ],
        'b2': [
            0.000,
            parameters.wheelbase * 0.50,
            0.000,
        ],
        'b2l': [
            parameters.body_width * 0.5,
            parameters.wheelbase * 0.50,
            0.000,
        ],
        'b2r': [
            parameters.body_width * -0.5,
            parameters.wheelbase * 0.50,
            0.000,
        ],
        'b3': [
            0.000,
            parameters.wheelbase * 0.86,
            0.000,
        ],
        'b3l': [
            parameters.body_width * 0.5,
            parameters.wheelbase * 0.86,
            0.000,
        ],
        'b3r': [
            parameters.body_width * -0.5,
            parameters.wheelbase * 0.86,
            0.000,
        ],
        'b4': [
            0.000,
            parameters.wheelbase * 1.00,
            0.000,
        ],
        'b4l': [
            parameters.body_width * 0.5,
            parameters.wheelbase * 1.00,
            parameters.body_height * 0.473,
        ],
        'b4r': [
            parameters.body_width * -0.5,
            parameters.wheelbase * 1.00,
            parameters.body_height * 0.473,
        ],
        'b5': [
            0.000,
            (parameters.wheelbase * 0.19) - 0.17,
            parameters.body_height * 0.591,
        ],
        'b5l': [
            parameters.body_width * 0.5,
            parameters.wheelbase * 0.19,
            parameters.body_height * 0.591,
        ],
        'b5r': [
            parameters.body_width * -0.5,
            parameters.wheelbase * 0.19,
            parameters.body_height * 0.591,
        ],
        'b6': [
            0.000,
            parameters.wheelbase * 0.50,
            parameters.body_height * 0.591,
        ],
        'b6l': [
            parameters.body_width * 0.5,
            parameters.wheelbase * 0.50,
            parameters.body_height * 0.591,
        ],
        'b6r': [
            parameters.body_width * -0.5,
            parameters.wheelbase * 0.50,
            parameters.body_height * 0.591,
        ],
        'b7': [
            0.000,
            parameters.wheelbase * 0.86,
            parameters.body_height * 0.591,
        ],
        'b7l': [
            parameters.body_width * 0.5,
            parameters.wheelbase * 0.86,
            parameters.body_height * 0.591,
        ],
        'b7r': [
            parameters.body_width * -0.5,
            parameters.wheelbase * 0.86,
            parameters.body_height * 0.591,
        ],
        'b8': [
            0.000,
            parameters.wheelbase * 1.00,
            parameters.body_height * 0.591,
        ],
        'b8l': [
            parameters.body_width * 0.5,
            parameters.wheelbase * 1.00,
            parameters.body_height * 0.591,
        ],
        'b8r': [
            parameters.body_width * -0.5,
            parameters.wheelbase * 1.00,
            parameters.body_height * 0.591,
        ],
        'r1': [
            0.000,
            parameters.wheelbase + parameters.rear_overhang - 0.000,
            0.000,
        ],
        'r1l': [
            parameters.body_width * 0.3,
            parameters.wheelbase + parameters.rear_overhang - 0.018,
            0.000,
        ],
        'r1ll': [
            parameters.body_width * 0.5,
            parameters.wheelbase + parameters.rear_overhang - 0.054,
            0.000,
        ],
        'r1r': [
            parameters.body_width * -0.3,
            parameters.wheelbase + parameters.rear_overhang - 0.018,
            0.000,
        ],
        'r1rr': [
            parameters.body_width * -0.5,
            parameters.wheelbase + parameters.rear_overhang - 0.054,
            0.000,
        ],
        'r2': [
            0.000,
            parameters.wheelbase + parameters.rear_overhang - 0.100,
            parameters.body_height * 0.591,
        ],
        'r2l': [
            parameters.body_width * 0.3,
            parameters.wheelbase + parameters.rear_overhang - 0.118,
            parameters.body_height * 0.591,
        ],
        'r2ll': [
            parameters.body_width * 0.5,
            parameters.wheelbase + parameters.rear_overhang - 0.154,
            parameters.body_height * 0.591,
        ],
        'r2r': [
            parameters.body_width * -0.3,
            parameters.wheelbase + parameters.rear_overhang - 0.118,
            parameters.body_height * 0.591,
        ],
        'r2rr': [
            parameters.body_width * -0.5,
            parameters.wheelbase + parameters.rear_overhang - 0.154,
            parameters.body_height * 0.591,
        ],
        'rf1': [
            0.000,
            (parameters.wheelbase * 0.38) - 0.17,
            parameters.body_height * 1.00,
        ],
        'rf1l': [
            parameters.body_width * 0.4,
            parameters.wheelbase * 0.38,
            parameters.body_height * 1.00,
        ],
        'rf1r': [
            parameters.body_width * -0.4,
            parameters.wheelbase * 0.38,
            parameters.body_height * 1.00,
        ],
        'rf2': [
            0.000,
            parameters.wheelbase * 0.60,
            parameters.body_height * 1.00,
        ],
        'rf2l': [
            parameters.body_width * 0.4,
            parameters.wheelbase * 0.60,
            parameters.body_height * 1.00,
        ],
        'rf2r': [
            parameters.body_width * -0.4,
            parameters.wheelbase * 0.60,
            parameters.body_height * 1.00,
        ],
        'rf3': [
            0.000,
            (parameters.wheelbase * 0.86) + 0.08,
            parameters.body_height * 1.00,
        ],
        'rf3l': [
            parameters.body_width * 0.4,
            parameters.wheelbase * 0.86,
            parameters.body_height * 1.00,
        ],
        'rf3r': [
            parameters.body_width * -0.4,
            parameters.wheelbase * 0.86,
            parameters.body_height * 1.00,
        ],
    }


def node_positions_wagon(parameters):
    return {
        'f1': [
            0.000,
            -parameters.front_overhang,
            0.000,
        ],
        'f1l': [
            parameters.body_width * 0.3,
            -parameters.front_overhang + 0.03,
            0.000,
        ],
        'f1ll': [
            parameters.body_width * 0.5,
            -parameters.front_overhang + 0.11,
            0.000,
        ],
        'f1r': [
            parameters.body_width * -0.3,
            -parameters.front_overhang + 0.03,
            0.000,
        ],
        'f1rr': [
            parameters.body_width * -0.5,
            -parameters.front_overhang + 0.11,
            0.000,
        ],
        'f2': [
            0.000,
            0.000,
            0.000,
        ],
        'f2l': [
            parameters.body_width * 0.5,
            0.000,
            parameters.body_height * 0.473,
        ],
        'f2r': [
            parameters.body_width * -0.5,
            0.000,
            parameters.body_height * 0.473,
        ],
        'f3': [
            0.000,
            -parameters.front_overhang + 0.08,
            parameters.body_height * 0.545,
        ],
        'f3l': [
            parameters.body_width * 0.3,
            -parameters.front_overhang + 0.13,
            parameters.body_height * 0.545,
        ],
        'f3ll': [
            parameters.body_width * 0.5,
            -parameters.front_overhang + 0.19,
            parameters.body_height * 0.536,
        ],
        'f3r': [
            parameters.body_width * -0.3,
            -parameters.front_overhang + 0.13,
            parameters.body_height * 0.545,
        ],
        'f3rr': [
            parameters.body_width * -0.5,
            -parameters.front_overhang + 0.19,
            parameters.body_height * 0.536,
        ],
        'f4': [
            0.000,
            0.000,
            parameters.body_height * 0.591,
        ],
        'f4l': [
            parameters.body_width * 0.5,
            0.000,
            parameters.body_height * 0.582,
        ],
        'f4r': [
            parameters.body_width * -0.5,
            0.000,
            parameters.body_height * 0.582,
        ],
        'b1': [
            0.000,
            parameters.wheelbase * 0.20,
            0.000,
        ],
        'b1l': [
            parameters.body_width * 0.5,
            parameters.wheelbase * 0.20,
            0.000,
        ],
        'b1r': [
            parameters.body_width * -0.5,
            parameters.wheelbase * 0.20,
            0.000,
        ],
        'b2': [
            0.000,
            parameters.wheelbase * 0.50,
            0.000,
        ],
        'b2l': [
            parameters.body_width * 0.5,
            parameters.wheelbase * 0.50,
            0.000,
        ],
        'b2r': [
            parameters.body_width * -0.5,
            parameters.wheelbase * 0.50,
            0.000,
        ],
        'b3': [
            0.000,
            parameters.wheelbase * 0.86,
            0.000,
        ],
        'b3l': [
            parameters.body_width * 0.5,
            parameters.wheelbase * 0.86,
            0.000,
        ],
        'b3r': [
            parameters.body_width * -0.5,
            parameters.wheelbase * 0.86,
            0.000,
        ],
        'b4': [
            0.000,
            parameters.wheelbase * 1.00,
            0.000,
        ],
        'b4l': [
            parameters.body_width * 0.5,
            parameters.wheelbase * 1.00,
            parameters.body_height * 0.473,
        ],
        'b4r': [
            parameters.body_width * -0.5,
            parameters.wheelbase * 1.00,
            parameters.body_height * 0.473,
        ],
        'b5': [
            0.000,
            (parameters.wheelbase * 0.19) - 0.17,
            parameters.body_height * 0.591,
        ],
        'b5l': [
            parameters.body_width * 0.5,
            parameters.wheelbase * 0.19,
            parameters.body_height * 0.591,
        ],
        'b5r': [
            parameters.body_width * -0.5,
            parameters.wheelbase * 0.19,
            parameters.body_height * 0.591,
        ],
        'b6': [
            0.000,
            parameters.wheelbase * 0.50,
            parameters.body_height * 0.591,
        ],
        'b6l': [
            parameters.body_width * 0.5,
            parameters.wheelbase * 0.50,
            parameters.body_height * 0.591,
        ],
        'b6r': [
            parameters.body_width * -0.5,
            parameters.wheelbase * 0.50,
            parameters.body_height * 0.591,
        ],
        'b7': [
            0.000,
            parameters.wheelbase * 0.86,
            parameters.body_height * 0.591,
        ],
        'b7l': [
            parameters.body_width * 0.5,
            parameters.wheelbase * 0.86,
            parameters.body_height * 0.591,
        ],
        'b7r': [
            parameters.body_width * -0.5,
            parameters.wheelbase * 0.86,
            parameters.body_height * 0.591,
        ],
        'b8': [
            0.000,
            parameters.wheelbase * 1.00,
            parameters.body_height * 0.591,
        ],
        'b8l': [
            parameters.body_width * 0.5,
            parameters.wheelbase * 1.00,
            parameters.body_height * 0.591,
        ],
        'b8r': [
            parameters.body_width * -0.5,
            parameters.wheelbase * 1.00,
            parameters.body_height * 0.591,
        ],
        'r1': [
            0.000,
            parameters.wheelbase + parameters.rear_overhang - 0.000,
            0.000,
        ],
        'r1l': [
            parameters.body_width * 0.3,
            parameters.wheelbase + parameters.rear_overhang - 0.018,
            0.000,
        ],
        'r1ll': [
            parameters.body_width * 0.5,
            parameters.wheelbase + parameters.rear_overhang - 0.054,
            0.000,
        ],
        'r1r': [
            parameters.body_width * -0.3,
            parameters.wheelbase + parameters.rear_overhang - 0.018,
            0.000,
        ],
        'r1rr': [
            parameters.body_width * -0.5,
            parameters.wheelbase + parameters.rear_overhang - 0.054,
            0.000,
        ],
        'r2': [
            0.000,
            parameters.wheelbase + parameters.rear_overhang - 0.100,
            parameters.body_height * 0.591,
        ],
        'r2l': [
            parameters.body_width * 0.3,
            parameters.wheelbase + parameters.rear_overhang - 0.118,
            parameters.body_height * 0.591,
        ],
        'r2ll': [
            parameters.body_width * 0.5,
            parameters.wheelbase + parameters.rear_overhang - 0.154,
            parameters.body_height * 0.591,
        ],
        'r2r': [
            parameters.body_width * -0.3,
            parameters.wheelbase + parameters.rear_overhang - 0.118,
            parameters.body_height * 0.591,
        ],
        'r2rr': [
            parameters.body_width * -0.5,
            parameters.wheelbase + parameters.rear_overhang - 0.154,
            parameters.body_height * 0.591,
        ],
        'rf1': [
            0.000,
            (parameters.wheelbase * 0.38) - 0.17,
            parameters.body_height * 1.00,
        ],
        'rf1l': [
            parameters.body_width * 0.4,
            parameters.wheelbase * 0.38,
            parameters.body_height * 1.00,
        ],
        'rf1r': [
            parameters.body_width * -0.4,
            parameters.wheelbase * 0.38,
            parameters.body_height * 1.00,
        ],
        'rf2': [
            0.000,
            parameters.wheelbase * 0.60,
            parameters.body_height * 1.00,
        ],
        'rf2l': [
            parameters.body_width * 0.4,
            parameters.wheelbase * 0.60,
            parameters.body_height * 1.00,
        ],
        'rf2r': [
            parameters.body_width * -0.4,
            parameters.wheelbase * 0.60,
            parameters.body_height * 1.00,
        ],
        'rf3': [
            0.000,
            (parameters.wheelbase * 0.86) + 0.08,
            parameters.body_height * 1.00,
        ],
        'rf3l': [
            parameters.body_width * 0.4,
            parameters.wheelbase * 0.86,
            parameters.body_height * 1.00,
        ],
        'rf3r': [
            parameters.body_width * -0.4,
            parameters.wheelbase * 0.86,
            parameters.body_height * 1.00,
        ],
        'rf4': [
            0.000,
            parameters.wheelbase + (parameters.rear_overhang * 0.3),
            parameters.body_height * 1.00,
        ],
        'rf4l': [
            parameters.body_width * 0.4,
            parameters.wheelbase + (parameters.rear_overhang * 0.3),
            parameters.body_height * 1.00,
        ],
        'rf4r': [
            parameters.body_width * -0.4,
            parameters.wheelbase + (parameters.rear_overhang * 0.3),
            parameters.body_height * 1.00,
        ],
    }


def node_positions_coupe(parameters):
    return {
        'f1': [
            0.000,
            -(parameters.front_overhang),
            0.000,
        ],
        'f1l': [
            parameters.body_width * 0.3,
            -parameters.front_overhang + 0.03,
            0.000,
        ],
        'f1ll': [
            parameters.body_width * 0.5,
            -parameters.front_overhang + 0.11,
            0.000,
        ],
        'f1r': [
            parameters.body_width * -0.3,
            -parameters.front_overhang + 0.03,
            0.000,
        ],
        'f1rr': [
            parameters.body_width * -0.5,
            -parameters.front_overhang + 0.11,
            0.000,
        ],
        'f2': [
            0.000,
            0.000,
            0.000,
        ],
        'f2l': [
            parameters.body_width * 0.5,
            0.000,
            parameters.body_height * 0.473,
        ],
        'f2r': [
            parameters.body_width * -0.5,
            0.000,
            parameters.body_height * 0.473,
        ],
        'f3': [
            0.000,
            -parameters.front_overhang + 0.08,
            parameters.body_height * 0.545,
        ],
        'f3l': [
            parameters.body_width * 0.3,
            -parameters.front_overhang + 0.13,
            parameters.body_height * 0.545,
        ],
        'f3ll': [
            parameters.body_width * 0.5,
            -parameters.front_overhang + 0.19,
            parameters.body_height * 0.536,
        ],
        'f3r': [
            parameters.body_width * -0.3,
            -parameters.front_overhang + 0.13,
            parameters.body_height * 0.545,
        ],
        'f3rr': [
            parameters.body_width * -0.5,
            -parameters.front_overhang + 0.19,
            parameters.body_height * 0.536,
        ],
        'f4': [
            0.000,
            0.000,
            parameters.body_height * 0.591,
        ],
        'f4l': [
            parameters.body_width * 0.5,
            0.000,
            parameters.body_height * 0.582,
        ],
        'f4r': [
            parameters.body_width * -0.5,
            0.000,
            parameters.body_height * 0.582,
        ],
        
        'b1': [
            0.000,
            parameters.wheelbase * 0.20,
            0.000,
        ],
        'b1l': [
            parameters.body_width * 0.5,
            parameters.wheelbase * 0.20,
            0.000,
        ],
        'b1r': [
            parameters.body_width * -0.5,
            parameters.wheelbase * 0.20,
            0.000,
        ],
        'b2': [
            0.000,
            parameters.wheelbase * 0.50,
            0.000,
        ],
        'b2l': [
            parameters.body_width * 0.5,
            parameters.wheelbase * 0.50,
            0.000,
        ],
        'b2r': [
            parameters.body_width * -0.5,
            parameters.wheelbase * 0.50,
            0.000,
        ],
        'b3': [
            0.000,
            parameters.wheelbase * 0.86,
            0.000,
        ],
        'b3l': [
            parameters.body_width * 0.5,
            parameters.wheelbase * 0.86,
            0.000,
        ],
        'b3r': [
            parameters.body_width * -0.5,
            parameters.wheelbase * 0.86,
            0.000,
        ],
        'b4': [
            0.000,
            parameters.wheelbase * 1.00,
            0.000,
        ],
        'b4l': [
            parameters.body_width * 0.5,
            parameters.wheelbase * 1.00,
            parameters.body_height * 0.473,
        ],
        'b4r': [
            parameters.body_width * -0.5,
            parameters.wheelbase * 1.00,
            parameters.body_height * 0.473,
        ],
        'b5': [
            0.000,
            (parameters.wheelbase * 0.19) - 0.17,
            parameters.body_height * 0.591,
        ],
        'b5l': [
            parameters.body_width * 0.5,
            parameters.wheelbase * 0.19,
            parameters.body_height * 0.591,
        ],
        'b5r': [
            parameters.body_width * -0.5,
            parameters.wheelbase * 0.19,
            parameters.body_height * 0.591,
        ],
        'b6': [
            0.000,
            parameters.wheelbase * 0.50,
            parameters.body_height * 0.591,
        ],
        'b6l': [
            parameters.body_width * 0.5,
            parameters.wheelbase * 0.50,
            parameters.body_height * 0.591,
        ],
        'b6r': [
            parameters.body_width * -0.5,
            parameters.wheelbase * 0.50,
            parameters.body_height * 0.591,
        ],
        'b7': [
            0.000,
            parameters.wheelbase * 0.86,
            parameters.body_height * 0.591,
        ],
        'b7l': [
            parameters.body_width * 0.5,
            parameters.wheelbase * 0.86,
            parameters.body_height * 0.591,
        ],
        'b7r': [
            parameters.body_width * -0.5,
            parameters.wheelbase * 0.86,
            parameters.body_height * 0.591,
        ],
        'b8': [
            0.000,
            parameters.wheelbase * 1.00,
            parameters.body_height * 0.591,
        ],
        'b8l': [
            parameters.body_width * 0.5,
            parameters.wheelbase * 1.00,
            parameters.body_height * 0.591,
        ],
        'b8r': [
            parameters.body_width * -0.5,
            parameters.wheelbase * 1.00,
            parameters.body_height * 0.591,
        ],
        
        'r1': [
            0.000,
            parameters.wheelbase + parameters.rear_overhang - 0.000,
            0.000,
        ],
        'r1l': [
            parameters.body_width * 0.3,
            parameters.wheelbase + parameters.rear_overhang - 0.018,
            0.000,
        ],
        'r1ll': [
            parameters.body_width * 0.5,
            parameters.wheelbase + parameters.rear_overhang - 0.054,
            0.000,
        ],
        'r1r': [
            parameters.body_width * -0.3,
            parameters.wheelbase + parameters.rear_overhang - 0.018,
            0.000,
        ],
        'r1rr': [
            parameters.body_width * -0.5,
            parameters.wheelbase + parameters.rear_overhang - 0.054,
            0.000,
        ],
        'r2': [
            0.000,
            parameters.wheelbase + parameters.rear_overhang - 0.100,
            parameters.body_height * 0.591,
        ],
        'r2l': [
            parameters.body_width * 0.3,
            parameters.wheelbase + parameters.rear_overhang - 0.118,
            parameters.body_height * 0.591,
        ],
        'r2ll': [
            parameters.body_width * 0.5,
            parameters.wheelbase + parameters.rear_overhang - 0.154,
            parameters.body_height * 0.591,
        ],
        'r2r': [
            parameters.body_width * -0.3,
            parameters.wheelbase + parameters.rear_overhang - 0.118,
            parameters.body_height * 0.591,
        ],
        'r2rr': [
            parameters.body_width * -0.5,
            parameters.wheelbase + parameters.rear_overhang - 0.154,
            parameters.body_height * 0.591,
        ],
        'rf1': [
            0.000,
            (parameters.wheelbase * 0.38) - 0.17,
            parameters.body_height * 1.00,
        ],
        'rf1l': [
            parameters.body_width * 0.4,
            parameters.wheelbase * 0.38,
            parameters.body_height * 1.00,
        ],
        'rf1r': [
            parameters.body_width * -0.4,
            parameters.wheelbase * 0.38,
            parameters.body_height * 1.00,
        ],
        'rf2': [
            0.000,
            parameters.wheelbase * 0.61,
            parameters.body_height * 1.00,
        ],
        'rf2l': [
            parameters.body_width * 0.4,
            parameters.wheelbase * 0.61,
            parameters.body_height * 1.00,
        ],
        'rf2r': [
            parameters.body_width * -0.4,
            parameters.wheelbase * 0.61,
            parameters.body_height * 1.00,
        ],
        'rf3': [
            0.000,
            (parameters.wheelbase * 0.86) + 0.08,
            parameters.body_height * 0.791,
        ],
        'rf3l': [
            parameters.body_width * 0.4,
            parameters.wheelbase * 0.86,
            parameters.body_height * 0.791,
        ],
        'rf3r': [
            parameters.body_width * -0.4,
            parameters.wheelbase * 0.86,
            parameters.body_height * 0.791,
        ],
        'rf4': [
            0.000,
            parameters.wheelbase + (parameters.rear_overhang * 0.5),
            parameters.body_height * 0.618,
        ],
        'rf4l': [
            parameters.body_width * 0.4,
            parameters.wheelbase + (parameters.rear_overhang * 0.5),
            parameters.body_height * 0.618,
        ],
        'rf4r': [
            parameters.body_width * -0.4,
            parameters.wheelbase + (parameters.rear_overhang * 0.5),
            parameters.body_height * 0.618,
        ],
    }


def node_positions_pickup(parameters):
    return {
        'f1': [
            0.000,
            -parameters.front_overhang,
            0.000,
        ],
        'f1l': [
            parameters.body_width * 0.3,
            -parameters.front_overhang + 0.03,
            0.000,
        ],
        'f1ll': [
            parameters.body_width * 0.5,
            -parameters.front_overhang + 0.11,
            0.000,
        ],
        'f1r': [
            parameters.body_width * -0.3,
            -parameters.front_overhang + 0.03,
            0.000,
        ],
        'f1rr': [
            parameters.body_width * -0.5,
            -parameters.front_overhang + 0.11,
            0.000,
        ],
        'f2': [
            0.000,
            0.000,
            0.000,
        ],
        'f2l': [
            parameters.body_width * 0.5,
            0.000,
            parameters.body_height * 0.473,
        ],
        'f2r': [
            parameters.body_width * -0.5,
            0.000,
            parameters.body_height * 0.473,
        ],
        'f3': [
            0.000,
            -parameters.front_overhang + 0.08,
            parameters.body_height * 0.545,
        ],
        'f3l': [
            parameters.body_width * 0.3,
            -parameters.front_overhang + 0.13,
            parameters.body_height * 0.545,
        ],
        'f3ll': [
            parameters.body_width * 0.5,
            -parameters.front_overhang + 0.19,
            parameters.body_height * 0.536,
        ],
        'f3r': [
            parameters.body_width * -0.3,
            -parameters.front_overhang + 0.13,
            parameters.body_height * 0.545,
        ],
        'f3rr': [
            parameters.body_width * -0.5,
            -parameters.front_overhang + 0.19,
            parameters.body_height * 0.536,
        ],
        'f4': [
            0.000,
            0.000,
            parameters.body_height * 0.591,
        ],
        'f4l': [
            parameters.body_width * 0.5,
            0.000,
            parameters.body_height * 0.582,
        ],
        'f4r': [
            parameters.body_width * -0.5,
            0.000,
            parameters.body_height * 0.582,
        ],
        'b1': [
            0.000,
            parameters.wheelbase * 0.20,
            0.000,
        ],
        'b1l': [
            parameters.body_width * 0.5,
            parameters.wheelbase * 0.20,
            0.000,
        ],
        'b1r': [
            parameters.body_width * -0.5,
            parameters.wheelbase * 0.20,
            0.000,
        ],
        'b2': [
            0.000,
            parameters.wheelbase * 0.50,
            0.000,
        ],
        'b2l': [
            parameters.body_width * 0.5,
            parameters.wheelbase * 0.50,
            0.000,
        ],
        'b2r': [
            parameters.body_width * -0.5,
            parameters.wheelbase * 0.50,
            0.000,
        ],
        'b3': [
            0.000,
            parameters.wheelbase * 0.86,
            0.000,
        ],
        'b3l': [
            parameters.body_width * 0.5,
            parameters.wheelbase * 0.86,
            0.000,
        ],
        'b3r': [
            parameters.body_width * -0.5,
            parameters.wheelbase * 0.86,
            0.000,
        ],
        'b4': [
            0.000,
            parameters.wheelbase * 1.00,
            0.000,
        ],
        'b4l': [
            parameters.body_width * 0.5,
            parameters.wheelbase * 1.00,
            parameters.body_height * 0.473,
        ],
        'b4r': [
            parameters.body_width * -0.5,
            parameters.wheelbase * 1.00,
            parameters.body_height * 0.473,
        ],
        'b5': [
            0.000,
            (parameters.wheelbase * 0.19) - 0.17,
            parameters.body_height * 0.591,
        ],
        'b5l': [
            parameters.body_width * 0.5,
            parameters.wheelbase * 0.19,
            parameters.body_height * 0.591,
        ],
        'b5r': [
            parameters.body_width * -0.5,
            parameters.wheelbase * 0.19,
            parameters.body_height * 0.591,
        ],
        'b6': [
            0.000,
            parameters.wheelbase * 0.50,
            parameters.body_height * 0.591,
        ],
        'b6l': [
            parameters.body_width * 0.5,
            parameters.wheelbase * 0.50,
            parameters.body_height * 0.591,
        ],
        'b6r': [
            parameters.body_width * -0.5,
            parameters.wheelbase * 0.50,
            parameters.body_height * 0.591,
        ],
        'b7': [
            0.000,
            parameters.wheelbase * 0.86,
            parameters.body_height * 0.591,
        ],
        'b7l': [
            parameters.body_width * 0.5,
            parameters.wheelbase * 0.86,
            parameters.body_height * 0.591,
        ],
        'b7r': [
            parameters.body_width * -0.5,
            parameters.wheelbase * 0.86,
            parameters.body_height * 0.591,
        ],
        'b8': [
            0.000,
            parameters.wheelbase * 1.00,
            parameters.body_height * 0.591,
        ],
        'b8l': [
            parameters.body_width * 0.5,
            parameters.wheelbase * 1.00,
            parameters.body_height * 0.591,
        ],
        'b8r': [
            parameters.body_width * -0.5,
            parameters.wheelbase * 1.00,
            parameters.body_height * 0.591,
        ],
        'r1': [
            0.000,
            parameters.wheelbase + parameters.rear_overhang - 0.000,
            0.000,
        ],
        'r1l': [
            parameters.body_width * 0.3,
            parameters.wheelbase + parameters.rear_overhang - 0.018,
            0.000,
        ],
        'r1ll': [
            parameters.body_width * 0.5,
            parameters.wheelbase + parameters.rear_overhang - 0.054,
            0.000,
        ],
        'r1r': [
            parameters.body_width * -0.3,
            parameters.wheelbase + parameters.rear_overhang - 0.018,
            0.000,
        ],
        'r1rr': [
            parameters.body_width * -0.5,
            parameters.wheelbase + parameters.rear_overhang - 0.054,
            0.000,
        ],
        'r2': [
            0.000,
            parameters.wheelbase + parameters.rear_overhang - 0.100,
            parameters.body_height * 0.591,
        ],
        'r2l': [
            parameters.body_width * 0.3,
            parameters.wheelbase + parameters.rear_overhang - 0.118,
            parameters.body_height * 0.591,
        ],
        'r2ll': [
            parameters.body_width * 0.5,
            parameters.wheelbase + parameters.rear_overhang - 0.154,
            parameters.body_height * 0.591,
        ],
        'r2r': [
            parameters.body_width * -0.3,
            parameters.wheelbase + parameters.rear_overhang - 0.118,
            parameters.body_height * 0.591,
        ],
        'r2rr': [
            parameters.body_width * -0.5,
            parameters.wheelbase + parameters.rear_overhang - 0.154,
            parameters.body_height * 0.591,
        ],
        'rf1': [
            0.000,
            (parameters.wheelbase * 0.38) - 0.17,
            parameters.body_height * 1.00,
        ],
        'rf1l': [
            parameters.body_width * 0.4,
            parameters.wheelbase * 0.38,
            parameters.body_height * 1.00,
        ],
        'rf1r': [
            parameters.body_width * -0.4,
            parameters.wheelbase * 0.38,
            parameters.body_height * 1.00,
        ],
        'rf2': [
            0.000,
            parameters.wheelbase * 0.50,
            parameters.body_height * 1.00,
        ],
        'rf2l': [
            parameters.body_width * 0.4,
            parameters.wheelbase * 0.50,
            parameters.body_height * 1.00,
        ],
        'rf2r': [
            parameters.body_width * -0.4,
            parameters.wheelbase * 0.50,
            parameters.body_height * 1.00,
        ],
    }


def transform_node_dictionary(pos, parameters, weight_body_main, weight_body_roof, cg_y_factor, cg_z_factor, inertia_yaw_factor, inertia_pitch_factor, inertia_roll_factor):
    d = {}
    body_height_half = parameters.body_height * 0.5
    for key, value in pos.items():
        x, y, z = value
        d[f'{key}_x'] = x
        d[f'{key}_y'] = y
        d[f'{key}_z'] = z
        if key.startswith('rf'):
            # roof node
            weight = weight_body_roof
        else:
            # main body node
            weight = weight_body_main
        d[f'{key}_m'] = node_mass_calculation(weight, x, y, z, cg_y_factor, cg_z_factor, inertia_yaw_factor, inertia_pitch_factor, inertia_roll_factor, body_height_half)
    return d


def node_mass_calculation(weight, x, y, z, cg_y_factor, cg_z_factor, inertia_yaw_factor, inertia_pitch_factor, inertia_roll_factor, body_height_half):
    return weight * (1 + cg_y_factor * y) * (1 + cg_z_factor * (z - body_height_half)) * (1 + inertia_yaw_factor * (x * x + y * y)) * (1 + inertia_pitch_factor * (x * x + z * z)) * (1 + inertia_roll_factor * (y * y + z * z))


def get_default_color(body_shape):
    colors = {
        "sedan": "Orange",
        "wagon": "Green",
        "coupe": "Yellow",
        "pickup": "Blue",
    }
    return colors[body_shape]
