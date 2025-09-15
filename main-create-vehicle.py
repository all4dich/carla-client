import carla
import random


def main():
    client = carla.Client('192.168.0.70', 2000)
    client.set_timeout(10.0)
    world = client.get_world()

    blueprint_library = world.get_blueprint_library()

    blueprints = [bp for bp in world.get_blueprint_library().filter('*')]
    for blueprint in blueprints:
        print(blueprint.id)
        for attr in blueprint:
            print('  - {}'.format(attr))

    # CORRECT WAY 1: Get a specific blueprint (e.g., 'model3')
    vehicle_bp = blueprint_library.find('vehicle.sprinter.mercedes')
    # Or a random vehicle blueprint
    # vehicle_bp = random.choice(blueprint_library.filter('vehicle.tesla.*'))

    # CORRECT WAY 2: Create a carla.Transform object
    # Option A: Get a random spawn point from the map
    spawn_point = random.choice(world.get_map().get_spawn_points())

    # Option B: Define a specific transform (location and rotation)
    # spawn_point = carla.Transform(carla.Location(x=10.0, y=20.0, z=1.0), carla.Rotation(yaw=90.0))

    # Spawn the actor with the correct arguments
    vehicle = world.spawn_actor(vehicle_bp, spawn_point)
    print(f"Spawned vehicle: {vehicle.type_id}")

    # Get the traffic manager
    traffic_manager = client.get_trafficmanager()
    traffic_manager.set_synchronous_mode(True)

#    # ou might want to do something with the vehicle, e.g., set autopilot
#    vehicle.set_autopilot(True, traffic_manager.get_port())
#
#    # Set the vehicle to follow lanes
#    traffic_manager.auto_lane_change(vehicle, False)
#    traffic_manager.distance_to_leading_vehicle(vehicle, 3.0)
#    traffic_manager.vehicle_percentage_speed_difference(vehicle, 0.0)
#
#    # Set the vehicle to respect traffic lights
#    traffic_manager.ignore_lights_percentage(vehicle, 0.0)
#    traffic_manager.ignore_signs_percentage(vehicle, 0.0)
#
#    print("Vehicle is now in autopilot mode.")

    # Clean up (important in CARLA)
    # input("Press Enter to destroy vehicle and exit...")
    # vehicle.destroy()
    # print("Vehicle destroyed.")


if __name__ == '__main__':
    main()
