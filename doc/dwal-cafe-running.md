# Vanilla DWAL / i-WALK / HuNavSim cafe

This adds a complete launch configuration, not a tested runtime deployment. There is no ROS,
Gazebo or Docker on the authoring machine. Offline geometry/configuration checks pass;
the Docker build and runtime `check` command below are the remaining acceptance gates.

## Run

From the workspace root, with Docker running and an X11/XWayland desktop:

```bash
bash scripts/run_dwal_cafe.sh build
bash scripts/run_dwal_cafe.sh run
```

The first command reuses `pmb2_hunav_simulation:latest` if already present. Otherwise it
builds upstream commit `770c5af36e860edb471fe2431eedcf6c9c3bb5d4`, which may take a while.
An existing base image must provide `hunav_agent_manager`, `hunav_gazebo_wrapper`,
the cafe and its actor assets. Its version is not proven by its image tag.
DWAL is pinned to `95b1aa5e1f0259ae8a467c731bf994fc0ebc4f6d`.
The i-WALK source used during preparation was commit `05db41c`.

All ROS processes run together inside one Humble/Gazebo Classic container. The host
Jazzy installation is neither sourced nor modified. Do not run the old
`display.launch.py`, PMB2 container or kiro_nav launch for this test.
The runtime uses Docker's own network and domain 73, so use `docker exec` for inspection.

In a second terminal, from the workspace root:

```bash
bash scripts/run_dwal_cafe.sh check
bash scripts/run_dwal_cafe.sh teleop
```

The check waits up to 45 seconds for valid odometry, finite laser returns, marked
obstacles in the costmap, sampled paths, and near-cluster messages. An empty cluster
message is allowed because it can legitimately describe a blocked scene.
This check does not prove that all pedestrians are lidar-visible: visually inspect
moving scan returns and costmap cells where a pedestrian crosses the scan plane.

The robot initially stays still; pedestrians move without a navigation goal.
Teleop uses the existing `teleop_twist_keyboard`: `i` forward, `j/l` turn, `k` stop.
Do not use the shifted holonomic keys: this is a differential-drive navigation test.
Always press `k` before leaving teleop: the existing planar plugin retains its last
command and does not implement a command timeout. DWAL does not stop teleop motion.

## What is reused

- The upstream `cafe.world`, its cafe shell, ground and five table includes.
- The upstream `agents_cafe.yaml`: five agents, their original goals and behaviors.
- HuNavSim's loader, agent manager, world generator and Gazebo plugin.
- Gazebo's planar-move and GPU-ray ROS plugins (no custom motor/odometry node).
- Nav2's standalone costmap executable and lifecycle manager.
- Upstream DWAL generator and clustering executables, with no algorithm changes.
- Standard robot_state_publisher and joint_state_publisher; RViz shows the original model.

DWAL publishes paths/clusters and markers, **not cmd_vel**. This launch demonstrates
its vanilla obstacle/path generation behavior. Autonomous selection of a cluster,
goal navigation and front-following are not implemented here. No DWB critics or
HATEB are launched, and no new people-to-costmap bridge is introduced.

## Geometry and sensor

The source `iwalk.urdf.xacro` is not edited. At launch a temporary copy is rerooted at
`sim_base`, the rear-axle ground projection. The original `base_link` to wheel,
mesh and `base_footprint` transforms are preserved. In the supplied URDF the old
`base_footprint` is offset longitudinally from the rear axle; it cannot be silently
used as the odometry pose reference for a DWAL footprint centered at the rear axle.

```
odom -> sim_base -> base_link -> original wheels and base_footprint
                -> laser_frame
```

The planar Gazebo model publishes `/odom` with child `sim_base` and the corresponding
TF. The robot state publisher owns all internal transforms. No duplicate odometry or
TF publishers are launched. There is no map/AMCL requirement for this local test.

The simulator flattens the zero-joint visuals into one rigid model and uses a
conservative box collision envelope. It does not model wheel torque, caster dynamics,
wheel slip or wheel animation. It is an ideal base for isolated planner testing.

The provisional lidar is at `(0.85, 0, 0.35)` metres relative to `sim_base`, forward
of the mesh, at a low enough plane to see people and table legs. It is a 360-degree
GPU laser with 720 samples at 15 Hz and 10 m range. Its rear rays may hit the robot;
this is normal self-occlusion. This mounting is for simulation, not a physical design.
GPU rendering is used so Gazebo actors can produce returns. Software OpenGL is
selected in the launcher to avoid depending on NVIDIA container configuration.
A working X display is required even for a GPU lidar without the Gazebo GUI.

One envelope is computed from the expanded model's visual/collision geometry and
lidar body; both DWAL and Nav2 receive that polygon with 0.02 m padding. At commit
05db41c it is `x=[-0.103, 0.865], y=[-0.327, 0.327]` metres. It intentionally includes
the existing conservative body collision box and provisional lidar overhang, not
just the mesh outline. The source body collision box extends further forward than
the mesh. The derivation does not claim to replace physical measurements.

The current HuNav wrapper uses its existing robot-agent approximation; this does not
make pedestrian social-force interactions polygon-aware. DWAL's collision checking
uses the actual configured polygon.

## Interfaces

| Data | Publisher | Consumer |
|---|---|---|
| `/clock` | Gazebo ROS | Simulation nodes |
| `/cmd_vel` | Optional existing teleop | Gazebo planar plugin |
| `/odom` | Gazebo planar plugin | DWAL |
| `/scan` | Gazebo GPU-ray ROS plugin | Nav2 obstacle layer |
| `/local_costmap/costmap` | Nav2 standalone costmap | DWAL |
| `/dwal_planner/sampled_paths` | DWAL generator | DWAL clustering |
| `/dwal_planner/clusters_near`, `clusters_far` | DWAL clustering | Future selector |
| DWAL marker topics | DWAL nodes | RViz |

Humble's standalone costmap fixes its node name to `/costmap/costmap`; only its
published costmap topics are remapped to `/local_costmap/...`. Parameters and lifecycle
management target that actual node. Full OccupancyGrid publication is enabled because
DWAL does not subscribe to incremental costmap updates. Odom and costmap share `odom`.
The local window is 10x10 m and comfortably encloses the configured 1.0/1.8 m levels.
Unknown/out-of-map handling remains upstream behavior; this is not a safety validation.

## Debug

```bash
bash scripts/run_dwal_cafe.sh shell
# Or inspect a running simulation:
docker exec -it iwalk_dwal /bin/bash /dwal_entrypoint.sh bash
source /opt/dwal_ws/install/setup.bash
ros2 node list
ros2 lifecycle get /costmap/costmap
ros2 topic hz /scan
ros2 topic hz /dwal_planner/sampled_paths
```

The launch prints its temporary directory (`/tmp/dwal_cafe_*`). It contains the generated
URDF, original cafe plus robot, HuNav-generated world, and both YAML configurations.
If the build fails, retain the **first compiler error**; if launch fails, retain the
first node error and output of `check`. A world-generation timeout is an error, not
permission to launch a stale world. Stop with Ctrl+C; the container is removed.

Offline geometry regression check (requires Python xacro, numpy and yaml):

```bash
python3 src/crowd_aware_simulation/test/check_scene.py \
  "$PWD/src/iwalk_description" /path/to/hunav_gazebo_wrapper/share
```

Verified during authoring: Python syntax, Bash syntax, expanded URDF geometry, unchanged
physical transforms, unique TF root, envelope containment, matching planner/costmap
footprints and frames, preservation of original cafe includes, odom/sensor plugin wiring.
Not verified here: Docker build, ROS lifecycle activation, Gazebo rendering, crowd
visibility in lidar, DWAL runtime output. Use the runtime gate before calling it working.

## Runtime fixes from the first launch log

The user confirmed all three packages built successfully in Humble. The first runtime
log then identified rejected YAML aliases and a missing libdiagnostic_updater.so.
The parameter dumper now disables all anchors/aliases (covered by the offline test),
and the image explicitly installs diagnostic_updater and runs an ldd dependency gate.
Gazebo also waited on its legacy online model database. Cafe, cafe_table and ground_plane
are now bundled from osrf/gazebo_models during image build, with launch-time existence
checks and the online model database disabled. Actor resources still come from HuNavSim.
These runtime fixes have passed offline checks; a new container run remains necessary.
