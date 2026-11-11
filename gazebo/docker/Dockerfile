# Heron USV simulator (Clearpath heron_simulator + uuv_simulator) on ROS Melodic /
# Gazebo 9, set up for headless runs driven from MATLAB over ROS 1.
FROM osrf/ros:melodic-desktop-full

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

# The ROS apt key shipped in old images has expired; fetch the renewed one.
RUN rm -f /etc/apt/sources.list.d/ros1-latest.list /etc/apt/sources.list.d/ros-latest.list \
 && apt-get update || true \
 && apt-get install -y --no-install-recommends curl ca-certificates gnupg2 \
 && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
      | gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg \
 && echo "deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros/ubuntu bionic main" \
      > /etc/apt/sources.list.d/ros-latest.list \
 && apt-get update \
 && apt-get install -y --no-install-recommends \
      git python-catkin-tools \
      ros-melodic-uuv-simulator \
      ros-melodic-hector-gazebo-plugins \
      ros-melodic-imu-filter-madgwick \
      ros-melodic-interactive-marker-twist-server \
      ros-melodic-robot-localization \
      ros-melodic-teleop-twist-joy \
      ros-melodic-joy \
      ros-melodic-lms1xx \
      ros-melodic-heron-controller \
 && rm -rf /var/lib/apt/lists/*

# Clearpath sources, pinned to the commits used for the system identification
# (heron melodic-devel 3ccbef6, heron_simulator kinetic-devel 46cb837).
WORKDIR /opt/heron_ws/src
RUN git clone https://github.com/heron/heron.git \
 && git -C heron checkout 3ccbef6 \
 && git clone -b kinetic-devel https://github.com/heron/heron_simulator.git \
 && git -C heron_simulator checkout 46cb837

# Local changes: EKF config as in ~/CHOMP_ws (use_control, yaw-rate fusion);
# no joystick node in the headless setup.
COPY config/robot_localization.yaml heron/heron_control/config/robot_localization.yaml
RUN sed -i 's|<arg name="joystick"  value="true" />|<arg name="joystick"  value="false" />|' \
      heron_simulator/heron_gazebo/launch/spawn_heron.launch
COPY ocean_gazebo ocean_gazebo

WORKDIR /opt/heron_ws
RUN source /opt/ros/melodic/setup.bash \
 && catkin config --extend /opt/ros/melodic --install \
 && catkin build --no-status \
 && rm -rf build logs

COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh \
 && echo "source /opt/heron_ws/install/setup.bash" >> /root/.bashrc

ENTRYPOINT ["/entrypoint.sh"]
CMD ["roslaunch", "ocean_gazebo", "heron_headless.launch"]
