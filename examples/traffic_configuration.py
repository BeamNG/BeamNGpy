from time import sleep

from beamngpy import BeamNGpy, set_up_simple_logging
from beamngpy.tools import TrafficConfig
from beamngpy.sensors import VehicleSensorConfig, MapSensorConfig

def main():
    set_up_simple_logging()
    beamng = BeamNGpy('localhost', 25252)
    bng = beamng.open()

    traffic = TrafficConfig(bng, "/traffic_config_ex.json")
    # With TrafficConfig.vehicles[vehicle name set from traffic configuration] you can access the related Vehicle object
    veh_sensors = VehicleSensorConfig("vehicle_sensors", bng, traffic.vehicles["etk"], "/traffic_veh_sens_config.json")
    map_sensors = MapSensorConfig("map_sensors", bng, "/traffic_map_sens_config.json")

    for _ in range(20):
        sleep(1)
        for s in range(len(veh_sensors.sensors)):
            sensor = veh_sensors.sensors[s]
            print(sensor.name)
            print(sensor.poll())
        for s in range(len(map_sensors.sensors)):
            sensor = map_sensors.sensors[s]
            print(sensor.name)
            print(sensor.poll())

    veh_sensors.remove()
    map_sensors.remove()
    bng.close()
    print("Example finished.")

if __name__ == "__main__":
    main()