import carla
import math, time

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

    # Get the driving (ego) vehicle
    vehicles = world.get_actors().filter('vehicle.*')
    ego = None
    for v in vehicles:
        if 'role_name' in v.attributes and v.attributes['role_name'] in ('hero', 'ego'):
            ego = v
            break
    if ego is None and vehicles:
        ego = vehicles[0]

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