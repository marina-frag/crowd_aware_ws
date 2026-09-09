# Reuse the existing HuNavSim environment; do not mix host Jazzy and Humble DDS.
ARG BASE_IMAGE=pmb2_hunav_simulation:latest
FROM ${BASE_IMAGE}
USER root
SHELL ["/bin/bash", "-c"]
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-nav2-costmap-2d ros-humble-nav2-lifecycle-manager \
    ros-humble-gazebo-plugins ros-humble-joint-state-publisher ros-humble-robot-state-publisher \
    ros-humble-xacro ros-humble-rviz2 ros-humble-teleop-twist-keyboard \
    python3-numpy python3-yaml && rm -rf /var/lib/apt/lists/*
WORKDIR /opt/dwal_ws
RUN git clone https://github.com/gmoustri/dwal_planner.git src/dwal_planner && \
    git -C src/dwal_planner checkout 95b1aa5e1f0259ae8a467c731bf994fc0ebc4f6d
# The reused base image can contain an older diagnostic_updater without the shared library.
RUN apt-get update && apt-get install -y --no-install-recommends ros-humble-diagnostic-updater && \
    rm -rf /var/lib/apt/lists/*
# Download only this scene's models at build time, never via the legacy model server at launch.
RUN git clone --depth 1 --filter=blob:none --sparse https://github.com/osrf/gazebo_models.git /opt/gazebo_models && \
    git -C /opt/gazebo_models sparse-checkout set cafe cafe_table ground_plane && \
    test -f /opt/gazebo_models/cafe/meshes/cafe.dae && \
    test -f /opt/gazebo_models/cafe_table/meshes/cafe_table.dae && \
    test -f /opt/gazebo_models/ground_plane/model.sdf
COPY src/iwalk_description src/iwalk_description
COPY src/crowd_aware_simulation src/crowd_aware_simulation
# Build against the simulator's ROS distribution. No algorithm changes to DWAL.
RUN source /opt/ros/humble/setup.bash && source /home/kiro_ws/install/setup.bash && \
    source /home/hunav_gz_ws/install/setup.bash && \
    colcon build --symlink-install --packages-select iwalk_description dwal_planner crowd_aware_simulation \
    --cmake-args -DCMAKE_BUILD_TYPE=Release
RUN source /opt/ros/humble/setup.bash && \
    python3 -c "import subprocess; p='/opt/ros/humble/lib/nav2_lifecycle_manager/lifecycle_manager'; s=subprocess.check_output(['ldd',p],text=True); print(s); assert 'not found' not in s, 'Missing lifecycle-manager runtime libraries'"
COPY docker/dwal/entrypoint.sh /dwal_entrypoint.sh
ENTRYPOINT ["/bin/bash", "/dwal_entrypoint.sh"]
CMD ["ros2", "launch", "crowd_aware_simulation", "dwal_cafe.launch.py"]
