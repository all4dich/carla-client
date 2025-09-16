# carla-client

A Python client for interacting with the CARLA simulator. This project provides a set of scripts to manage and control actors within a CARLA simulation environment.

## Features

*   Connect to a CARLA server.
*   Spawn and manage vehicles.
*   Control autonomous and manual vehicles.
*   Retrieve simulation data like actor lists and sensor data.
*   Containerized for easy deployment with Docker.
*   Load custom maps.

## Prerequisites

*   [Docker](https://www.docker.com/) and [Docker Compose](https://docs.docker.com/compose/)
*   A running instance of the [CARLA Simulator](https://carla.readthedocs.io/en/latest/getting_started/)
*   Python dependencies, including `pygame` and `numpy`.

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

*   `main.py`: A general-purpose script to connect to the CARLA server, list actors, and set a vehicle to autopilot.
*   `manual_control.py`: Allows for detailed manual control of a vehicle using a keyboard, complete with a comprehensive HUD.
*   `automatic_control.py`: Controls a vehicle automatically using a defined agent (e.g., Behavior, Basic).
*   `map_load.py`: Loads a specified map into the CARLA simulator.
*   `main-create-vehicle.py`: Spawns a new vehicle at a random spawn point.
*   `main-create-autonomous-vehicle.py`: Spawns a new vehicle and enables its autopilot.
*   `main-delete-all-vehicles.py`: Removes all vehicles from the simulation.
*   `main-get-list.py`: Prints a list of all actors in the world.
*   `main-read-info.py`: Reads and prints information about the simulation world.
*   `main-reload-world.py`: Reloads the current CARLA world.

## Modules

*   `modules/`: This directory contains the Python modules used by the control scripts, such as the agents for `automatic_control.py`.

## Usage

To run any of the scripts, you can use `python` directly if you have the `carla` library and other dependencies installed, or use `docker-compose` to run them inside the container.

**Example: Run the main script**

```bash
# Ensure CARLA_HOST is set
export CARLA_HOST=localhost

# Run the script
python main.py
```

This will connect to the CARLA server, list the actors, and perform the default actions in the script.
