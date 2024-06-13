# MAE148 / Spring 2024 / Team 03

### Project
Our project had two goals:
1. Control an RC car type vehicle with hand signals.
2. Detect construction pallets and pick them up with a forklift-esque mechanism.

This repository contains all code necessary to achieve these goals.

### Features

* Fully modularized: Can be installed onto any system with the appropriate prerequisites
* Includes all dependencies required for ROS2 development and testing using ROS2 Humble
* Automated, one line installation and setup, powered by Docker
* Ease of accessibility, powered by a custom Makefile

### Prerequisites

1. [Docker](https://docs.docker.com/get-docker/)
   1. Install Docker using above link
   2. Ensure Docker is enabled to run on startup and added to the `docker` group. Restart your computer after running these commands:

      ```bash
      sudo systemctl enable docker.service
      sudo systemctl enable containerd.service
      sudo usermod -aG docker $USER
      ```
2. Make, which can be installed using `sudo apt install build-essential` on most Debian based systems.
3. Basic knowledge of container based applications, which you can read about [here](https://docker-curriculum.com/).

### Setup

1. Clone this repo.

```sh
git clone https://github.com/ankbhatia19/MAE148-HandSignals.git
```

2. `cd` into the newly created folder. Run `make`.

### Usage

`cd` inside the ROS2-Containerized folder (Where the Makefile and Dockerfile files are located). The Makefile provides a set of commands available for ease of accessibility.

* `make`: Enters the container, allowing you to run ROS2 commands.
  * If ROS2 has not been downloaded, installation is automatically started
  * Automatically creates container if a container has not yet been created
  * Creates a new bash terminal which is running inside the container.
* `make clean`: Stops currently running container.
* `make status`: Indicates whether the container is running or not.

Additional commands can be seen by running `make help`.

### Running Project Nodes
* Launch the container using `make`. Navigate to `workspaces/148_ws` once inside the container.
* Source the necessary overlays using:
```sh
. /opt/ros/humble/setup.bash
. install/setup.bash
```
* Build the project:
```sh
colcon build
```
* And finally launch:
```sh
ros2 launch handsigs_bringup handsigs.launch.py
```

### Contact

For questions about this project or otherwise, contact the developers on Discord using Discord ID `Waycey`.
