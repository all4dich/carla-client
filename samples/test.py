import carla
import time

def main():
    # Connect to the CARLA server
    client = carla.Client('localhost', 1403)
    client.set_timeout(10.0)  # Timeout for server connection (seconds)

    try:
        # Get the world
        world = client.get_world()

        # Get the blueprint library
        blueprint_library = world.get_blueprint_library()
        
        # Filter for a vehicle (Tesla ford)
        vehicle_bps = blueprint_library.filter('vehicle.taxi.ford')
        
        # Check if the blueprint exists
        if not vehicle_bps:
            print("Error: No Tesla Model 3 blueprint found. Available vehicles:")
            for bp in blueprint_library.filter('vehicle.*.*'):
                print(bp.id)
            return
        
        vehicle_bp = vehicle_bps[0]
        print(f"Selected blueprint: {vehicle_bp.id}")

        # Get a spawn point (use a predefined spawn point to avoid invalid locations)
        spawn_points = world.get_map().get_spawn_points()
        if not spawn_points:
            print("Error: No spawn points available in the map.")
            return
        
        spawn_point = spawn_points[0]  # Use the first valid spawn point
        print(f"Spawning at: {spawn_point.location}")

        # Spawn the vehicle
        ego_vehicle = world.spawn_actor(vehicle_bp, spawn_point)
        print("Ego vehicle spawned successfully!")

        # Set initial speed to zero
        ego_vehicle.set_target_velocity(carla.Vector3D(0, 0, 0))
        ego_vehicle.enable_constant_velocity(carla.Vector3D(0, 0, 0))  # Ensure zero speed

        # Keep the script running to maintain the vehicle
        while True:
            world.wait_for_tick()
            time.sleep(0.1)

    except Exception as e:
        print(f"An error occurred: {e}")

    finally:
        # Clean up
        if 'ego_vehicle' in locals():
            ego_vehicle.destroy()
            print("Ego vehicle destroyed.")

if __name__ == '__main__':
    main()