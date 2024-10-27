#!/bin/bash

# Set ros envrionment variables
source ${ROS_PATH}/setup.bash
# Set Gazebo envrionment variables
source /usr/share/gazebo/setup.bash
# Set workspace variables
source ${ROS_WS}/install/setup.bash

# create symbolic links to the ROS workspace
rm ROS_WS
ln -s ${ROS_WS} $PWD/ROS_WS

# create symbolic links to the URDF description directories
source_directory=${ROS_WS}/src
target_directory=02_URDF
# Find directories with the suffix "_description" in the source directory
for dir in "$source_directory"/*_description; 
do
  # Check if it is a directory
  if [ -d "$dir" ]; then
    # Get the base name of the directory
    base_name=$(basename "$dir")
    # Create the symbolic link in the target directory
    rm "$target_directory/$base_name"
    ln -s "$dir" "$target_directory/$base_name"
  fi
done

stress --cpu 4 --vm 2 --vm-bytes 1G --timeout 600s &

exec "$@"
