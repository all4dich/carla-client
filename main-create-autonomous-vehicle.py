import os

import carla
import random

CARLA_HOST = os.environ['CARLA_HOST']

def main():
 client = carla.Client(CARLA_HOST, 2000)
 client.set_timeout(10.0)
 world = client.get_world()

 blueprint_library = world.get_blueprint_library()

 # Get the blueprint for 'vehicle.tesla.model3'
 vehicle_bp = blueprint_library.find('vehicle.tesla.model3')

 # Get a random spawn point from the map
 spawn_point = random.choice(world.get_map().get_spawn_points())

 # Spawn the vehicle
 vehicle = world.spawn_actor(vehicle_bp, spawn_point)
 print(f"Spawned vehicle: {vehicle.type_id}")

 # Get the traffic manager
 traffic_manager = client.get_trafficmanager()
 traffic_manager.set_synchronous_mode(True)

 # Set the vehicle to autopilot mode
 vehicle.set_autopilot(True, traffic_manager.get_port())

 print("Vehicle is now in autopilot mode.")

 # Keep the script running to observe the autonomous vehicle
 input("Press Enter to destroy vehicle and exit...")
 vehicle.destroy()
 print("Vehicle destroyed.")


if __name__ == '__main__':
 main()
