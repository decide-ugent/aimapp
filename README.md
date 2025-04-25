# Map DM Nav Project 

This repository contains the code for the Actove Inference (AIF) Decision Making, localisation and mapping (Map DM Nav) project. 
It requires ubuntu 22.04 and ros2 humble or is configured to run inside a Docker container for ease of setup and consistent environments.

## Overview

This project focuses on exploring or reaching goals in a fully unknown environment using AIF

Using Docker allows you to run the project and its dependencies without complex installation on your host machine.


## Docker
### Prerequisites

Before you begin, ensure you have the following installed:

1.  **Git:** To clone the repository.
2.  **Docker:** Docker Engine or Docker Desktop. Installation instructions can be found on the [official Docker website](https://docs.docker.com/get-docker/).

### Setup and Installation

1.  **Clone the Repository:**
```bash
git clone map_dm_nav git_link
cd map_dm_nav
```

2.  **Build the Docker Image:**
    * This command builds the image using the `Dockerfile` in the current directory and tags it as `map_dm_nav`.
```bash
docker build -t map_dm_nav .
```

### Usage

#### Start the docker
**Make the script executable (if needed):**
```bash
chmod +x launch_docker.sh
```

**Run the container:**
```bash
bash launch_docker.sh 
```
* mounts volumes (for input/output data), so the map_dm_nav repo can be modified locally and run in your docker.
once it finished colcon build 
press ctrl+p ctrl+q to detach the window and let the docker run in the background

to access the container:
```bash
docker exec -it map_dm_nav bash
```

## Locally
### Prerequisites

Before you begin, ensure you have the following installed:

1.  **Ubuntu 22.04** 
2.  **Ros2 humble** 

### Setup and Installation

```bash
cd map_dm_nav/map_dm_nav
pip install -e .
```
```bash
cd /home/YOUR_ROS_WP/src/ \
&& git clone https://github.com/ROBOTIS-GIT/turtlebot3_simulations -b humble \
&& git clone https://github.com/aws-robotics/aws-robomaker-small-warehouse-world -b ros2 \
&& git clone https://github.com/aws-robotics/aws-robomaker-small-house-world -b ros2 
```

Make sure the robot we are calling exist (we created a waffle robot with 3 cameras, to change the turtle version, go in spawn_turtle_launch.py)
-> look at the `COPY` in the Dockerfile to copy paste the modifications done to the worlds and turtlebot3 in the appropriate locations (to have all our worlds + top view camera -optional-).


## Start the project
Start the world
```
source install/setup.bash 
ros2 launch map_dm_nav warehouse_launch.py
```

OR
```
export GAZEBO_MODEL_PATH=`pwd`/models
source /usr/share/gazebo/setup.sh
ros2 launch aws_robomaker_small_house_world small_house.launch.py gui:=true
```

**Spawn the agent**
```
source install/setup.bash 
ros2 launch map_dm_nav spawn_turtle_launch.py x:=0.0 y:=0.0
```
**Start Nav2** (optional) -- If started has to start almost simultaneously as the world and spawn due to simulation time reliance.
```
source install/setup.bash 
ros2 launch map_dm_nav nav2_humble_launch.py
```

**Start the agent**
```
source install/setup.bash 
ros2 launch map_dm_nav agent_launch.py
```

**Record position GT/believed_odom over time** (optional)
```
source install/setup.bash 
ros2 run map_dm_nav simulation_overview_data_save.py
```

**Start Rviz** (optional)
```
rviz2 -d src/map_dm_nav/map_dm_nav/rviz/nav2_default_view.rviz 
```

