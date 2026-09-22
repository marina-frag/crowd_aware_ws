# Reuse the existing HuNavSim environment; do not mix host Jazzy and Humble DDS.
ARG BASE_IMAGE=pmb2_hunav_simulation:latest
FROM ${BASE_IMAGE}
USER root
SHELL ["/bin/bash", "-c"]
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-nav2-costmap-2d ros-humble-nav2-lifecycle-manager \
    ros-humble-nav2-controller ros-humble-nav2-dwb-controller \
    ros-humble-nav2-regulated-pure-pursuit-controller \
    ros-humble-nav2-velocity-smoother ros-humble-nav2-collision-monitor \
    ros-humble-gazebo-plugins ros-humble-joint-state-publisher ros-humble-robot-state-publisher \
    ros-humble-xacro ros-humble-rviz2 ros-humble-teleop-twist-keyboard ros-humble-laser-filters \
    libboost-all-dev libsuitesparse-dev libeigen3-dev libopenblas0-pthread \
    python3-numpy python3-yaml python3-pytest && rm -rf /var/lib/apt/lists/*
WORKDIR /opt/dwal_ws
RUN git clone https://github.com/gmoustri/dwal_planner.git src/dwal_planner && \
    git -C src/dwal_planner checkout 95b1aa5e1f0259ae8a467c731bf994fc0ebc4f6d
RUN git clone https://github.com/gmoustri/bayesian_shared_control.git src/bayesian_shared_control && \
    git -C src/bayesian_shared_control checkout 9d0df25f8a966db449ec94c073349d09e328d517
COPY patches /tmp/crowd_aware_patches
RUN git -C src/dwal_planner apply --check /tmp/crowd_aware_patches/dwal_dynamic_radius.patch && \
    git -C src/dwal_planner apply /tmp/crowd_aware_patches/dwal_dynamic_radius.patch && \
    git -C src/bayesian_shared_control apply --check /tmp/crowd_aware_patches/bayesian_shared_control_safety_semantics.patch && \
    git -C src/bayesian_shared_control apply /tmp/crowd_aware_patches/bayesian_shared_control_safety_semantics.patch
# The PMB2 image carries CoHAN sources but intentionally builds only messages.
# Pin that checkout, enable the exact packages reused here, and add its Humble
# costmap_converter dependency at a fixed revision.
RUN git -C /home/kiro_ws/src/CoHAN-Nav2 fetch origin 094f095a4cb0dfb774730c6020750c72adef0773 && \
    git -C /home/kiro_ws/src/CoHAN-Nav2 checkout --detach 094f095a4cb0dfb774730c6020750c72adef0773 && \
    git -C /home/kiro_ws/src/CoHAN-Nav2 apply --check /tmp/crowd_aware_patches/cohan_hateb_local_frame.patch && \
    git -C /home/kiro_ws/src/CoHAN-Nav2 apply /tmp/crowd_aware_patches/cohan_hateb_local_frame.patch && \
    mv /home/kiro_ws/src/CoHAN-Nav2/src/ros2_helpers/COLCON_IGNORE /tmp/ros2_helpers.COLCON_IGNORE && \
    mv /home/kiro_ws/src/CoHAN-Nav2/src/agent_path_prediction/COLCON_IGNORE /tmp/agent_path_prediction.COLCON_IGNORE && \
    mv /home/kiro_ws/src/CoHAN-Nav2/src/cohan_layers/COLCON_IGNORE /tmp/cohan_layers.COLCON_IGNORE && \
    mv /home/kiro_ws/src/CoHAN-Nav2/src/hateb_local_planner/COLCON_IGNORE /tmp/hateb_local_planner.COLCON_IGNORE && \
    git clone https://github.com/rst-tu-dortmund/costmap_converter.git /home/kiro_ws/src/costmap_converter_humble && \
    git -C /home/kiro_ws/src/costmap_converter_humble checkout --detach 9565858432664e9cfe0b3556e1809336c4ab06d4
RUN source /opt/ros/humble/setup.bash && source /home/kiro_ws/install/setup.bash && \
    cd /home/kiro_ws && colcon build --symlink-install \
    --packages-select costmap_converter_msgs costmap_converter ros2_helpers agent_path_prediction cohan_layers hateb_local_planner \
    --cmake-args -DCMAKE_BUILD_TYPE=Release
# The reused base image can contain an older diagnostic_updater without the shared library.
RUN apt-get update && apt-get install -y --no-install-recommends ros-humble-diagnostic-updater && \
    rm -rf /var/lib/apt/lists/*
# Download only this scene's models at build time, never via the legacy model server at launch.
RUN git init /opt/gazebo_models && \
    git -C /opt/gazebo_models remote add origin https://github.com/osrf/gazebo_models.git && \
    git -C /opt/gazebo_models fetch --depth 1 --filter=blob:none origin 8163eb4b5e7e21985c6591d1c0bfb56468c0093f && \
    git -C /opt/gazebo_models sparse-checkout init --cone && \
    git -C /opt/gazebo_models sparse-checkout set cafe cafe_table ground_plane && \
    git -C /opt/gazebo_models checkout --detach FETCH_HEAD && \
    test -f /opt/gazebo_models/cafe/meshes/cafe.dae && \
    test -f /opt/gazebo_models/cafe_table/meshes/cafe_table.dae && \
    test -f /opt/gazebo_models/ground_plane/model.sdf
COPY src/iwalk_description src/iwalk_description
COPY src/crowd_aware_interfaces src/crowd_aware_interfaces
COPY src/crowd_aware_simulation src/crowd_aware_simulation
# Build against the simulator's ROS distribution.
RUN source /opt/ros/humble/setup.bash && source /home/kiro_ws/install/setup.bash && \
    source /home/hunav_gz_ws/install/setup.bash && \
    colcon build --symlink-install --packages-select \
    iwalk_description crowd_aware_interfaces dwal_planner bayesian_shared_control crowd_aware_simulation \
    --cmake-args -DCMAKE_BUILD_TYPE=Release
RUN source /opt/ros/humble/setup.bash && \
    python3 -c "import subprocess; p='/opt/ros/humble/lib/nav2_lifecycle_manager/lifecycle_manager'; s=subprocess.check_output(['ldd',p],text=True); print(s); assert 'not found' not in s, 'Missing lifecycle-manager runtime libraries'"
COPY docker/dwal/entrypoint.sh /dwal_entrypoint.sh
ENTRYPOINT ["/bin/bash", "/dwal_entrypoint.sh"]
CMD ["ros2", "launch", "crowd_aware_simulation", "dwal_cafe.launch.py"]
