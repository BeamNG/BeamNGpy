import matplotlib.pyplot as plt
import numpy as np
from beamngpy import BeamNGpy, Scenario, Vehicle, set_up_simple_logging
from beamngpy.sensors import Camera
from time import sleep

# Executing this file will demonstrate the annotation functionality of the camera sensor.

if __name__ == '__main__':
    set_up_simple_logging()

    # Start up the simulator.
    bng = BeamNGpy('localhost', 64256)
    bng.open(launch=True)

    scenario = Scenario('italy', 'annotation_bounding_boxes')

    # Create some vehicles which will appear in the camera images.
    ego = Vehicle('ego', model='etk800', color='White')
    scenario.add_vehicle(ego, pos=(237.90, -894.42, 246.10), rot_quat=(0.0173, -0.0019, -0.6354, 0.7720))
    car1 = Vehicle('car1', model='etk800', color='Green')
    scenario.add_vehicle(car1, pos=(246.94, -901.64, 247.58), rot_quat=(-0.0099, 0.0206, 0.9348, -0.3543))
    car2 = Vehicle('car2', model='etk800', color='Red')
    scenario.add_vehicle(car2, pos=(276.27, -881.42, 247.84), rot_quat=(-0.0106, 0.0405, 0.4845, 0.8738))
    car3 = Vehicle('car3', model='etki', color='Blue')
    scenario.add_vehicle(car3, pos=(261.52, -894.68, 248.04), rot_quat=(0.0026, 0.01758, -0.8344, 0.5508))
    car4 = Vehicle('car4', model='miramar', color='Black')
    scenario.add_vehicle(car4, pos=(267.06, -892.03, 248.32), rot_quat=(0.0065, 0.0194, -0.8501, 0.5262))

    scenario.make(bng)
    bng.set_deterministic()
    bng.set_steps_per_second(60)                            # Set simulator to 60hz temporal resolution
    bng.load_scenario(scenario)
    bng.hide_hud()
    bng.start_scenario()
    bng.switch_vehicle(ego)

    # Get the annotation class data.
    annotations = bng.get_annotations()                     # Gets a dictionary of RGB colours, indexed by material names.
    print(annotations)
    class_data = bng.get_annotation_classes(annotations)    # Gets a dictionary of material names, indexed by RGB colours (encoded as 32-bit).
    print(class_data)

    sleep(10)  # Some sleeping time to make sure the level fully loaded.

    # Create some camera sensors in the simulation.
    print("Camera test start.")

    # Create a camera sensor.
    cam1 = Camera('camera1', bng, ego, requested_update_time=-1.0, is_using_shared_memory=True, pos=(-0.3, 1, 2), dir=(0, -1, 0),
        field_of_view_y=70, near_far_planes=(0.1, 100), resolution=(1024, 1024), is_render_instance=True)

    sleep(2)

    images = cam1.get_full_poll_request()

    fig, ax = plt.subplots(1, 4, figsize=(30, 30))
    ax[0].imshow(np.asarray(images['colour'].convert('RGB')))
    ax[1].imshow(np.asarray(images['annotation'].convert('RGB')))
    ax[2].imshow(np.asarray(images['instance'].convert('RGB')))
    ax[3].imshow(np.asarray(images['depth'].convert('RGB')))
    plt.show()

    # Compute and display the bounding boxes for each vehicle instance.
    bounding_boxes = Camera.extract_bounding_boxes(images['annotation'], images['instance'], class_data)
    bounding_boxes = [b for b in bounding_boxes if b['class'] == 'CAR']
    print(bounding_boxes)

    # Draw the bounding boxes on top of the colour image.
    image_with_boxes = Camera.draw_bounding_boxes(bounding_boxes, images['colour'], width=3)
    plt.clf()
    plt.figure(figsize=(15, 15))
    plt.imshow(np.asarray(image_with_boxes.convert('RGB')))
    plt.show()

    # Export the bounding boxes to PASCAL-VOC XML format.
    xml = Camera.export_bounding_boxes_xml(bounding_boxes, filename='example.png', size=(1024, 1024, 3))
    print(xml)

    # Close the simulation.
    bng.close()
