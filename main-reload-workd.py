import carla

CARLA_HOST = "192.168.0.99"
def main():
    # Connect to the CARLA server
    client = carla.Client(CARLA_HOST, 1403)
    client.set_timeout(10.0)

    # Get the world
    world = client.get_world()

    # Print some information about the world
    print("World: ", world)
    print("Map name: ", world.get_map().name)

    # Reload the world
    world = client.reload_world()
    print("World reloaded.")


main()