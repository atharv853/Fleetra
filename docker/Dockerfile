# Local rebuild of jderobot/robotics-backend-ros2:amazon-warehouse-multi-robot
# (that image is no longer on Docker Hub).
#
# The original Dockerfile (CustomRobots/amazon_robot/docker/Dockerfile) was
#   FROM rosplanning/navigation2:main.release
# i.e. ROS 2 Foxy + Navigation2 *main branch* built from source (Aug 2020).
# That base tag is gone too, so it is reproduced here with the same layout:
#   /opt/underlay_ws  - Nav2's underlay.repos deps pinned to Aug 2020
#   /opt/overlay_ws   - navigation2 @ 98e994f (main, 2020-08-26)
#   /opt/warehouse_ws - CustomRobots/amazon_robot (multirobot-testing), unmodified
# Apt Foxy Nav2 can NOT be used: amazon_robot_controller needs createBond(),
# buildTreeFromText() etc. which only existed on Nav2 main in 2020.
#
# Build context: CustomRobots/amazon_robot (with the world submodule checked out).
FROM osrf/ros:foxy-desktop
ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

# Refresh the ROS apt key (the key baked into old Foxy images expired in 2025)
RUN rm -f /etc/apt/sources.list.d/ros2*.list; \
    apt-get update || true; \
    apt-get install -y --no-install-recommends curl ca-certificates && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=amd64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu focal main" > /etc/apt/sources.list.d/ros2.list && \
    apt-get update && \
    apt-get install -y --no-install-recommends \
      ros-foxy-gazebo-ros-pkgs ros-foxy-xacro ros-foxy-angles \
      ros-foxy-image-transport ros-foxy-cv-bridge \
      python3-colcon-common-extensions python3-rosdep python3-vcstool git \
      qtbase5-dev mesa-utils procps && \
    rm -rf /var/lib/apt/lists/*

# ---- underlay (tools/underlay.repos of nav2 @ 98e994f, pinned by date) ----
WORKDIR /opt/underlay_ws/src
RUN git clone https://github.com/BehaviorTree/BehaviorTree.CPP.git && \
      git -C BehaviorTree.CPP checkout d17a07895c4cc297ff591fbb0b3d95174fe85ad9 && \
    git clone -b lifecycle_support_2 https://github.com/SteveMacenski/bond_core.git && \
      git -C bond_core checkout dd3234e8e23d440b354df38d7522658df2458647

# ---- overlay: navigation2 main @ 2020-08-26 ----
WORKDIR /opt/overlay_ws/src
RUN git clone https://github.com/ros-navigation/navigation2.git && \
    git -C navigation2 checkout 98e994fa6912e87cf547a15cbe0f26fd67a3f01a && \
    touch navigation2/nav2_system_tests/COLCON_IGNORE

RUN apt-get update && rosdep update --rosdistro foxy && \
    rosdep install -y -r --rosdistro foxy --ignore-src \
      --from-paths /opt/underlay_ws/src /opt/overlay_ws/src ; \
    rm -rf /var/lib/apt/lists/*

WORKDIR /opt/underlay_ws
RUN . /opt/ros/foxy/setup.bash && \
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF

WORKDIR /opt/overlay_ws
RUN . /opt/underlay_ws/install/setup.bash && \
    MAKEFLAGS=-j6 colcon build --parallel-workers 3 \
      --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF

# ---- the JdeRobot exercise workspace (same path as the original image) ----
COPY . /opt/warehouse_ws/src/
WORKDIR /opt/warehouse_ws/
RUN apt-get update && \
    rosdep install -y -r --rosdistro foxy --ignore-src \
      --from-paths /opt/underlay_ws/src /opt/overlay_ws/src /opt/warehouse_ws/src ; \
    rm -rf /var/lib/apt/lists/*
RUN . /opt/overlay_ws/install/setup.bash && colcon build --symlink-install

# Original entrypoint from the repo: sources foxy, overlay_ws, warehouse_ws
COPY docker/ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod 755 /ros_entrypoint.sh
ENV GAZEBO_MODEL_PATH=/opt/warehouse_ws/src/amazon_robot_gazebo/models:/opt/warehouse_ws/src/aws-robomaker-small-warehouse-world/models
# CycloneDDS: Fast-DDS discovery on Foxy is unreliable with this many nodes
# (2 full Nav2 stacks + Gazebo + 2 RViz). JdeRobot's own docker-compose.yaml
# has RMW_IMPLEMENTATION=rmw_cyclonedds_cpp (commented out) for the same reason.
RUN apt-get update && apt-get install -y --no-install-recommends ros-foxy-rmw-cyclonedds-cpp && \
    rm -rf /var/lib/apt/lists/*
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
