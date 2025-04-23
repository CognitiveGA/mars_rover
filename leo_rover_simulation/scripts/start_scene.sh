#! /usr/bin/env bash

W_PATH="/home/shady/Documents/unige_robotics_msc/0x02_second_year_2nd_semster/cogar/cogar_ros2_ws"

# Reset PYTHONPATH to include only necessary paths
export PYTHONPATH="${W_PATH}/install/lib/python3.8/site-packages:/opt/ros/humble/lib/python3.8/site-packages"

# Source ROS 2 humble
source /opt/ros/humble/setup.bash

# Source Gazebo environment
source /usr/share/gazebo/setup.sh

# Source the workspace setup if available
SIM_SETUP="${W_PATH}/install/setup.bash"
if [ -f "${SIM_SETUP}" ]; then
    source "${SIM_SETUP}"
else
    echo "Error: ${SIM_SETUP} not found."
    exit 1
fi

export GAZEBO_RESOURCE_PATH=${ROS2_WS_PATH}/src/leo_rover_simulation/leo_description:${GAZEBO_RESOURCE_PATH}
export GAZEBO_MODEL_PATH=${ROS2_WS_PATH}/src/leo_rover_simulation/leo_description:${GAZEBO_MODEL_PATH}
export GAZEBO_MODEL_DATABASE_URI=""
ros2 launch leo_description main_pro.launch.py
