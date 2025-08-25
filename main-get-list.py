import carla
import os
CARLA_HOST = "192.168.0.70"
def main():
    # Connect to the CARLA server
    client = carla.Client(CARLA_HOST, 2000)
    client.set_timeout(10.0)

    # Get the world
    world = client.get_world()

    # Print some information about the world
    print("World: ", world)
    print("Map name: ", world.get_map().name)


    # List all actors in the world
    actors = world.get_actors()
    vehicle_actors = actors.filter('vehicle.*')
        
    print(f"🌍 Found {len(vehicle_actors)} vehicles on the map.")

    print("Actors in the world:")
    for actor in actors:
        print(" - ", actor)
    
    # Get the player vehicle
    player = world.get_actors().filter('vehicle.*')[0]
    print("Player vehicle: ", player)
    # Print player vehicle's attributes
    print("Player vehicle attributes:")
    for attr in player.attributes:
        print(f" - {attr}: {player.attributes[attr]}")
main()