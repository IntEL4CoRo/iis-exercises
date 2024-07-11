#!/bin/bash
# Set ros envrionment variables
source ${ROS_PATH}/setup.bash
# Set Gazebo envrionment variables
source /usr/share/gazebo/setup.bash
# Set workspace variables
source ${ROS_WS}/install/setup.bash


ln -s ${ROS_WS} $PWD/ROS_WS
rm 02_URDF/examples/spot_description
ln -s ${ROS_WS}/src/spot_description 02_URDF/examples/spot_description


pip install -e ./jupyterlab-urdf

exec "$@"