import carla
import sys
import math
import time

CARLA_HOST = "192.168.0.70"
def main():
    # Connect to the CARLA server
    client = carla.Client(CARLA_HOST,   2000)
    client.set_timeout(10.0)

    # Get the world
    world = client.get_world()

    # Print some information about the world
    print("World: ", world)
    print("Map name: ", world.get_map().name)

    # Get vehicle by id passed as --vehicle-id=123
    ego = None
    vehicle_id = None
    for arg in sys.argv[1:]:
        if arg.startswith("--vehicle-id="):
            try:
                vehicle_id = int(arg.split("=", 1)[1])
            except ValueError:
                print("Invalid vehicle id value.")
            break

    if vehicle_id is not None:
        actor = world.get_actor(vehicle_id)
        if actor and actor.type_id.startswith("vehicle."):
            ego = actor
        else:
            print(f"Actor id {vehicle_id} is not a vehicle or does not exist.")
    else:
        vehicle_id = 220
        ego = world.get_actor(vehicle_id)
        print(f"Using default vehicle id={vehicle_id}.")
    if ego is None:
        print("No vehicle found.")
    else:
        print(f"Using vehicle id={ego.id} type={ego.type_id} role_name={ego.attributes.get('role_name','')}")
        # Sample a few times
        for _ in range(10000):
            ctrl = ego.get_control()
            vel = ego.get_velocity()
            speed = math.sqrt(vel.x**2 + vel.y**2 + vel.z**2)  # m/s
            print(f"Vehicle={ego.id} Throttle={ctrl.throttle:.3f} Steering={ctrl.steer:.3f} Velocity(m/s)={speed:.2f}")
            time.sleep(0.1)
main()