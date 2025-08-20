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

# Source the gazebo setup.bash to set up the environment variables
RUN echo "source /usr/share/gazebo/setup.bash" >> /home/${NB_USER}/.bashrc
ENV GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/${ROS_DISTRO}/share/turtlebot3_gazebo/models
ENV TURTLEBOT3_MODEL=waffle_pi

# --- Create ROS workspaces --- #
USER ${NB_USER}
RUN mkdir -p ${ROS_WS}/src
WORKDIR ${ROS_WS}

# --- Fetch packages support ROS2 --- #
# turtlebot3
WORKDIR ${ROS_WS}/src
RUN git clone -b ${ROS_DISTRO} https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git

# iai_office_sim
RUN git clone -b ros2 https://github.com/yxzhan/iai_office_sim.git
# Fix the texture path not include in gazebo resource path
ENV GAZEBO_RESOURCE_PATH=$GAZEBO_RESOURCE_PATH:${ROS_WS}/src/iai_office_sim/resource
ENV GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:${ROS_WS}/src/iai_office_sim/models

# Fix the texture path not include in gazebo resource path,
USER root
RUN cp $ROS_WS/src/iai_office_sim/resource/Media/materials/textures/* /usr/share/gazebo-11/media/materials/textures/
USER ${NB_USER}

# spot_description
RUN git clone https://github.com/bdaiinstitute/spot_ros2.git /tmp/spot_ros2 && \
    mv /tmp/spot_ros2/spot_description ${ROS_WS}/src/spot_description

# --- Build ROS2 workspace --- #
WORKDIR ${ROS_WS}
USER root
RUN rosdep update && \
    rosdep install --from-paths src --ignore-src  -y && \
    rosdep fix-permissions

USER ${NB_USER}
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && \
    colcon build --symlink-install --parallel-workers 12
RUN echo "source ${ROS_WS}/install/setup.bash" >> /home/${NB_USER}/.bashrc

# --- Fetch robot descriptions to be upgraded to ROS2 --- #
RUN mkdir -p ${HOME}/tmp
WORKDIR ${HOME}/tmp
# armar6
RUN git clone https://github.com/cram2/armar6_description
# iai PR2
RUN git clone https://github.com/PR2/pr2_common.git && \
    mv pr2_common/pr2_description . && \
    rm -rf pr2_common
RUN git clone https://github.com/code-iai/iai_pr2.git && \
    mv iai_pr2/iai_pr2_description . && \
    rm -rf iai_pr2

# HD kitchen model
RUN git clone https://github.com/Multiverse-Framework/Multiverse-World.git && \
    mv Multiverse-World/iai_apartment ./ && \
    rm -rf Multiverse-World

# --- Install python packages --- #
USER ${NB_USER}
COPY requirements.txt /tmp/requirements.txt
RUN pip install -r /tmp/requirements.txt

# --- Copy repo content --- #
COPY --chown=${NB_USER}:users . /home/${NB_USER}/iis-exercises
# RUN mv  ${HOME}/tmp/* /home/${NB_USER}/iis-exercises/02_URDF
RUN for dir in ${HOME}/tmp/*; do ln -s "$dir" ${HOME}/iis-exercises/02_URDF/$(basename "$dir"); done

WORKDIR ${HOME}/iis-exercises

# --- Entrypoint --- #
COPY --chown=${NB_USER}:users entrypoint.sh /
ENTRYPOINT ["/entrypoint.sh"]
CMD [ "start-notebook.sh" ]

# Install VirtualGL
USER root
RUN wget -q -O- https://packagecloud.io/dcommander/virtualgl/gpgkey | \
  gpg --dearmor >/etc/apt/trusted.gpg.d/VirtualGL.gpg
RUN wget https://raw.githubusercontent.com/VirtualGL/repo/main/VirtualGL.list -O /etc/apt/sources.list.d/VirtualGL.list
RUN apt-get update && \
    apt-get install -y virtualgl mesa-utils
USER ${NB_USER}