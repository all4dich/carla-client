import carla
import random


def main():
    client = carla.Client('192.168.0.70', 2000)
    client.set_timeout(10.0)
    world = client.get_world()

    blueprint_library = world.get_blueprint_library()

    # Get all actors currently in the world
    actor_list = world.get_actors()

    # Filter for vehicles and destroy them
    for actor in actor_list:
        if 'vehicle' in actor.type_id:
            actor.destroy()
            print(f"Destroyed vehicle: {actor.type_id}")

    print("All vehicles deleted.")


if __name__ == '__main__':
    main()