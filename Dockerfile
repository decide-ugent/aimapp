#Install Ubuntu 20.04
FROM osrf/ros:humble-desktop

# Set environment variable to use host's X11 display
ENV DISPLAY=:0

# Make sure everything is up to date before building from source
RUN apt-get update \
  && apt-get upgrade -y \
  && apt-get clean \
  && apt-get -y install python3-pip \ 
  && pip install --upgrade pip

# Install Xvfb and necessary dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    xvfb \
    libxcb-xinerama0 \
    libxkbcommon-x11-0 \
    x11-apps \   
    && apt-get clean

#python 3.8 already in ros humble 
# ARG DEBIAN_FRONTEND=noninteractive
# RUN apt-get update && apt-get install -y --no-install-recommends python3.8 python3-tk libcairo2-dev
    
RUN apt-get update && apt-get install -q -y --no-install-recommends \
    dirmngr \
    gnupg2 \
    lsb-release \
    python3-colcon-ros \ 
    ros-humble-realtime-tools \ 
    ros-humble-test-msgs \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    python3-pyqt5 \  
    && apt-get clean \

RUN apt-get update && apt-get install -y software-properties-common \
    && add-apt-repository ppa:openrobotics/gazebo11-gz-cli \
    && apt-get update \
    && sudo apt-get install gazebo11 -y --no-install-recommends

# Make sure to install gazebo 11.12.0 instead of 11.11.0
# RUN apt-get install -q -y --no-install-recommends lsb-release wget gnupg \
#     && wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
#     && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null \
#     && apt-get update \
#     && apt-get upgrade -y gazebo11

#Install dependent packages
RUN mkdir -p /home/ros2_ws/src \
    && cd /home/ros2_ws/src/ \
    && git clone https://github.com/ROBOTIS-GIT/turtlebot3_simulations -b humble \
    && git clone https://github.com/aws-robotics/aws-robomaker-small-warehouse-world -b ros2 \
    && git clone https://github.com/aws-robotics/aws-robomaker-small-house-world -b ros2 \
    && rosdep fix-permissions && rosdep update --include-eol-distros \
    && rosdep install --from-paths ./ -i -y --rosdistro humble \
      --ignore-src

#Modify turtlebot3 launch to accept a different namespace as param
COPY modif_pcks/spawn_turtlebot3.launch.py /home/ros2_ws/src/turtlebot3_simulations/turtlebot3_gazebo/launch/spawn_turtlebot3.launch.py
COPY modif_pcks/robot_state_publisher.launch.py /home/ros2_ws/src/turtlebot3_simulations/turtlebot3_gazebo/launch/robot_state_publisher.launch.py
#waffle with 3 cameras for an almost 360 (a 360 camera or more cameras and my simulation crashes)
COPY modif_pcks/turtlebot3_waffle_pi_plus.urdf /home/ros2_ws/src/turtlebot3_simulations/turtlebot3_gazebo/urdf/turtlebot3_waffle_pi_plus.urdf
COPY modif_pcks/turtlebot3_waffle_pi_plus /home/ros2_ws/src/turtlebot3_simulations/turtlebot3_gazebo/models/turtlebot3_waffle_pi_plus
#a top view camera to record what is hapenning 
COPY modif_pcks/fisheye /home/ros2_ws/src/aws-robomaker-small-warehouse-world/models/fisheye
COPY modif_pcks/fisheye /home/ros2_ws/src/aws-robomaker-small-house-world/models/fisheye
#my own modified worlds
COPY modif_pcks/aws_robomaker_warehouse_smaller /home/ros2_ws/src/aws-robomaker-small-warehouse-world/worlds/aws_robomaker_warehouse_smaller
COPY modif_pcks/warehouse_mini /home/ros2_ws/src/aws-robomaker-small-warehouse-world/worlds/warehouse_mini
COPY modif_pcks/no_roof_small_warehouse /home/ros2_ws/src/aws-robomaker-small-warehouse-world/worlds/no_roof_small_warehouse
COPY modif_pcks/small_house.world /home/ros2_ws/src/aws-robomaker-small-house-world/worlds/small_house.world


RUN cd /home/ros2_ws/ \
    && . /opt/ros/humble/setup.sh \
    && colcon build
# --merge-install

RUN apt-get install -y xauth
COPY aimapp/aimapp/setup.py /home/setup/setup.py
COPY aimapp/aimapp/requirements.txt /home/setup/requirements.txt

RUN cd /home/setup && pip install .

RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "source /home/ros2_ws/install/setup.bash" >> ~/.bashrc
RUN echo "export NUMBA_THREADING_LAYER=workqueue" >> ~/.bashrc

COPY entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]


#command: docker build -t aimapp .
