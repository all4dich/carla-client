import carla
import argparse
import random
import time
import numpy as np
import cv2

argparse = argparse.ArgumentParser(description="CARLA Client Example")
argparse.add_argument('--host', type=str, default='localhost', help='CARLA server host address')
argparse.add_argument('--port', type=int, default=2000, help='CARLA server port number')
args = argparse.parse_args()
CARLA_HOST = "192.168.0.72"

def process_img(image):
    i = np.array(image.raw_data)
    i2 = i.reshape((image.height, image.width, 4))
    i3 = i2[:, :, :3]
    cv2.imshow("", i3)
    cv2.waitKey(1)
    return i3/255.0

def main():
    client = carla.Client(CARLA_HOST, 2000)
    client.set_timeout(10.0)

    world = client.get_world()
    blueprint_library = world.get_blueprint_library()

    try:
        # Get a random vehicle blueprint
        vehicle_bp = random.choice(blueprint_library.filter('vehicle.tesla.model3'))

        # Set a random spawn point
        spawn_point = random.choice(world.get_map().get_spawn_points())

        # Spawn the vehicle
        vehicle = world.spawn_actor(vehicle_bp, spawn_point)
        vehicle.set_autopilot(True)
        print(f"Spawned vehicle: {vehicle.id}")

        # Get the camera blueprint
        camera_bp = blueprint_library.find('sensor.camera.rgb')
        camera_bp.set_attribute('image_size_x', '640')
        camera_bp.set_attribute('image_size_y', '480')
        camera_bp.set_attribute('fov', '110')

        # Define the camera's position relative to the vehicle (behind and slightly above)
        camera_transform = carla.Transform(carla.Location(x=-5.0, z=2.0), carla.Rotation(pitch=8.0))

        # Spawn the camera and attach it to the vehicle
        camera = world.spawn_actor(camera_bp, camera_transform, attach_to=vehicle)
        print(f"Spawned camera: {camera.id}")

        # Start listening for camera data
        camera.listen(lambda image: process_img(image))

        print("Camera is now streaming video. Press Ctrl+C to stop.")

        # Keep the script running to allow the camera to stream
        while True:
            world.wait_for_tick()
            time.sleep(0.01) # Small delay to prevent busy-waiting

    except Exception as e:
        print(f"An error occurred: {e}")

    finally:
        print("Destroying actors...")
        actors_to_destroy = []
        if 'camera' in locals() and camera.is_alive:
            actors_to_destroy.append(camera)
        if 'vehicle' in locals() and vehicle.is_alive:
            actors_to_destroy.append(vehicle)

        client.apply_batch([carla.command.DestroyActor(x) for x in actors_to_destroy])
        print("Actors destroyed.")

main()