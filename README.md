# carla-client

A Python client for interacting with the CARLA simulator. This project provides a set of scripts to manage and control actors within a CARLA simulation environment.

## Features

*   Connect to a CARLA server.
*   Spawn and manage vehicles.
*   Control autonomous vehicles.
*   Retrieve simulation data like actor lists and sensor data.
*   Containerized for easy deployment with Docker.

## Prerequisites

*   [Docker](https://www.docker.com/) and [Docker Compose](https://docs.docker.com/compose/)
*   A running instance of the [CARLA Simulator](https://carla.readthedocs.io/en/latest/getting_started/)

## Getting Started

1.  **Clone the repository:**
    ```bash
    git clone <repository-url>
    cd carla-client
    ```

2.  **Configure CARLA Host:**
    The scripts connect to the CARLA simulator using the `CARLA_HOST` environment variable. Make sure to set it to the IP address of your running CARLA instance. You can set this in your shell or in the `docker-compose.yaml` file.

    ```bash
    export CARLA_HOST=localhost
    ```

3.  **Build and run with Docker (optional):**
    A `Dockerfile` and `docker-compose.yaml` are provided for running the client in a containerized environment.

    ```bash
    docker-compose up --build
    ```

## Available Scripts

The core functionality is exposed through a series of Python scripts:

*   `main-create-vehicle.py`: Spawns a new vehicle at a random spawn point.
*   `main-create-autonomous-vehicle.py`: Spawns a new vehicle and enables its autopilot.
*   `main-delete-all-vehicles.py`: Removes all vehicles from the simulation.
*   `main-get-list.py`: Prints a list of all actors in the world.
*   `main-read-info.py`: Reads and prints information about the simulation world.
*   `main-reload-world.py`: Reloads the current CARLA world.
*   `main-get-video.py`: (Functionality to be implemented) Intended to retrieve video streams.
*   `main-start.py` / `main-stop.py`: (Functionality to be implemented) Intended for managing the client lifecycle.

## Usage

To run any of the scripts, you can use `python` directly if you have the `carla` library installed, or use `docker-compose` to run them inside the container.

**Example: Create a new vehicle**

```bash
# Ensure CARLA_HOST is set
export CARLA_HOST=localhost

# Run the script
python main-create-vehicle.py
```

This will connect to the CARLA server at `$CARLA_HOST`, spawn a `vehicle.sprinter.mercedes`, and print its ID.
