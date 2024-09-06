FROM intel4coro/jupyter-ros2:humble-py3.10

ENV ROS_DISTRO=humble
ENV ROS_WS=/home/${NB_USER}/ros2_ws

# --- Install apt packages --- #
USER root
RUN apt update && apt install -y \
    ros-${ROS_DISTRO}-gazebo-* \
    ros-${ROS_DISTRO}-cartographer \
    ros-${ROS_DISTRO}-cartographer-ros \
    ros-${ROS_DISTRO}-navigation2 \
    ros-${ROS_DISTRO}-nav2-bringup \
    ros-${ROS_DISTRO}-dynamixel-sdk \
    ros-${ROS_DISTRO}-turtlebot3* \
    byobu \
    iputils-ping && \
    rm -rf /var/lib/apt/lists/* && \
    apt-get clean

# Source the gazebo setup.bash to set up the envrionment variables
RUN echo "source /usr/share/gazebo/setup.bash" >> /home/${NB_USER}/.bashrc
ENV GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/${ROS_DISTRO}/share/turtlebot3_gazebo/models
ENV TURTLEBOT3_MODEL=waffle_pi

# --- Install python packages --- #
COPY requirements.txt /tmp/requirements.txt
RUN pip install -r /tmp/requirements.txt

# --- Create ROS workspaces --- #
USER ${NB_USER}
RUN mkdir -p ${ROS_WS}/src
WORKDIR ${ROS_WS}

# --- Fetch packages support ROS2 --- #
# turtlebot3
RUN cd src && \
    git clone -b ${ROS_DISTRO}-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git && \
    git clone -b ros2 https://github.com/yxzhan/iai_office_sim.git
# spot_description
RUN git clone https://github.com/bdaiinstitute/spot_ros2.git /tmp/spot_ros2 && \
    mv /tmp/spot_ros2/spot_description ${ROS_WS}/src/spot_description

# --- Build ROS2 workspace --- #
USER root
RUN apt update && rosdep update && \
    rosdep install --from-paths src --ignore-src  -y && \
    rosdep fix-permissions

USER ${NB_USER}
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && \
    colcon build --symlink-install --parallel-workers 12
RUN echo "source ${ROS_WS}/install/setup.bash" >> /home/${NB_USER}/.bashrc

# --- Copy repo content --- #
COPY --chown=${NB_USER}:users . /home/${NB_USER}/iis-exercises

# --- Fetch robot descriptions to be upgraded to ROS2 --- #
WORKDIR /home/${NB_USER}/iis-exercises/02_URDF
# armar6
RUN git clone https://github.com/cram2/armar6_description
# iai PR2
RUN git clone https://github.com/PR2/pr2_common.git
RUN git clone https://github.com/code-iai/iai_pr2.git

WORKDIR /home/${NB_USER}/iis-exercises

# --- Entrypoint --- #
COPY --chown=${NB_USER}:users entrypoint.sh /
ENTRYPOINT ["/entrypoint.sh"]
CMD [ "start-notebook.sh" ]
