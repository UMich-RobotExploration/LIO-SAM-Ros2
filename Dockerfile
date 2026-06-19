FROM osrf/ros:humble-desktop-full-jammy

RUN apt-get update \
    && apt-get install -y curl \
    && curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - \
    && apt-get update \
    && apt install -y python3-colcon-common-extensions \
    && apt-get install -y ros-humble-navigation2 \
    && apt-get install -y ros-humble-robot-localization \
    && apt-get install -y ros-humble-robot-state-publisher \
    && apt install -y ros-humble-perception-pcl \
  	&& apt install -y ros-humble-pcl-msgs \
  	&& apt install -y ros-humble-vision-opencv \
  	&& apt install -y ros-humble-xacro \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update \
    && apt install -y software-properties-common \
    && add-apt-repository -y ppa:borglab/gtsam-release-4.1 \
    && apt-get update \
    && apt install -y libgtsam-dev libgtsam-unstable-dev \
    && rm -rf /var/lib/apt/lists/*

SHELL ["/bin/bash", "-c"]

RUN mkdir -p ~/ros2_ws/src \
    && cd ~/ros2_ws/src \
    && git clone --branch ros2 https://github.com/UMich-RobotExploration/LIO-SAM-Ros2.git \
    && cd .. \
    && source /opt/ros/humble/setup.bash \
    && colcon build

# UDP-only FastDDS profile so the container can exchange data with a ROS 2
# bag/sensor running on the host (shared memory does not cross the container
# IPC namespace). Path matches FASTRTPS_DEFAULT_PROFILES_FILE in docker-compose.yaml.
RUN mkdir -p /usr/local/share/middleware_profiles \
    && printf '%s\n' \
    '<?xml version="1.0" encoding="UTF-8" ?>' \
    '<dds>' \
    '  <profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">' \
    '    <transport_descriptors>' \
    '      <transport_descriptor>' \
    '        <transport_id>CustomUdpTransport</transport_id>' \
    '        <type>UDPv4</type>' \
    '      </transport_descriptor>' \
    '    </transport_descriptors>' \
    '    <participant profile_name="participant_profile" is_default_profile="true">' \
    '      <rtps>' \
    '        <userTransports>' \
    '          <transport_id>CustomUdpTransport</transport_id>' \
    '        </userTransports>' \
    '        <useBuiltinTransports>false</useBuiltinTransports>' \
    '      </rtps>' \
    '    </participant>' \
    '  </profiles>' \
    '</dds>' \
    > /usr/local/share/middleware_profiles/rtps_udp_profile.xml

RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc \
    && echo "source /root/ros2_ws/install/setup.bash" >> /root/.bashrc

WORKDIR /root/ros2_ws
