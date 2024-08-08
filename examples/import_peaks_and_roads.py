from beamngpy import BeamNGpy, Scenario, Vehicle, set_up_simple_logging
from beamngpy.tools import Terrain_Importer

import time

def main():
    set_up_simple_logging()

    # Initialize BeamNG.
    print("Initializing BeamNG...")
    beamng = BeamNGpy('localhost', 64256)
    beamng.open(launch=True)
    scenario = Scenario('tech_ground', 'terrain_and_roads_importer')
    vehicle = Vehicle('ego_vehicle', model='etk800')
    scenario.add_vehicle(vehicle)

    # Start up BeamNG.
    print("Making scenario...")
    scenario.make(beamng)
    beamng.scenario.load(scenario)
    beamng.scenario.start()

    # Set some peaks/troughs, which will define the terrain.
    peaks = []
    peaks.append({ 'x': 100.0, 'y': 100.0, 'z': 250.0 })
    peaks.append({ 'x': 900.0, 'y': 100.0, 'z': 200.0 })
    peaks.append({ 'x': 100.0, 'y': 900.0, 'z': 0.0 })
    peaks.append({ 'x': -500.0, 'y': 500.0, 'z': 250.0 })
    peaks.append({ 'x': 800.0, 'y': -300.0, 'z': 150.0 })
    peaks.append({ 'x': -500.0, 'y': -300.0, 'z': 0.0 })
    peaks.append({ 'x': -800.0, 'y': -800.0, 'z': 250.0 })
    peaks.append({ 'x': -100.0, 'y': -500.0, 'z': 250.0 })
    peaks.append({ 'x': 600.0, 'y': 300.0, 'z': 150.0 })
    peaks.append({ 'x': -900.0, 'y': 900.0, 'z': 0.0 })

    # Set some roads.
    roads1 = []
    roads1.append({ 'x': -800.0, 'y': 100.0, 'width': 7.0 })
    roads1.append({ 'x': -700.0, 'y': 100.0, 'width': 7.0 })
    roads1.append({ 'x': -600.0, 'y': 100.0, 'width': 7.0 })
    roads1.append({ 'x': -500.0, 'y': 100.0, 'width': 7.0 })
    roads1.append({ 'x': -400.0, 'y': 100.0, 'width': 7.0 })
    roads1.append({ 'x': -300.0, 'y': 100.0, 'width': 7.0 })
    roads1.append({ 'x': -200.0, 'y': 100.0, 'width': 7.0 })
    roads1.append({ 'x': -100.0, 'y': 100.0, 'width': 7.0 })
    roads1.append({ 'x': 0.0, 'y': 100.0, 'width': 7.0 })
    roads1.append({ 'x': 100.0, 'y': 100.0, 'width': 7.0 })
    roads1.append({ 'x': 200.0, 'y': 100.0, 'width': 7.0 })
    roads1.append({ 'x': 300.0, 'y': 100.0, 'width': 7.0 })
    roads1.append({ 'x': 400.0, 'y': 100.0, 'width': 7.0 })
    roads1.append({ 'x': 500.0, 'y': 100.0, 'width': 7.0 })

    roads2 = []
    roads2.append({ 'x': 800.0, 'y': 100.0, 'width': 7.0 })
    roads2.append({ 'x': 700.0, 'y': 200.0, 'width': 7.0 })
    roads2.append({ 'x': 600.0, 'y': 300.0, 'width': 7.0 })
    roads2.append({ 'x': 500.0, 'y': 400.0, 'width': 7.0 })
    roads2.append({ 'x': 400.0, 'y': 500.0, 'width': 7.0 })
    roads2.append({ 'x': 300.0, 'y': 600.0, 'width': 7.0 })

    roads = []
    roads.append(roads1)
    roads.append(roads2)

    # Allow the user to open the world editor.
    print("Press F11 on BeamNG to open the world editor...")
    time.sleep(5)

    # Import the peaks and roads to BeamNG.
    DOI = 150.0
    margin = 6.0
    Terrain_Importer.peaks_and_road_import(beamng, peaks, roads, DOI, margin)

    # Wait for 30 seconds, then clear the terrain and roads (ready for another iteration, if you like).
    time.sleep(60)
    #Terrain_Importer.reset(beamng)

    # Execute BeamNG until the user closes it.
    print("Completed.")
    while(True):
        pass
    beamng.close()


if __name__ == '__main__':
    main()
