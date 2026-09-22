# Crowd-Aware iWalk — Development Readiness Report

**Review date:** 2026-09-21  
**Workspace:** `crowd_aware_ws`  
**Review brief:** `setup review crowd aware.md`

## 1. Executive conclusion

The workspace is ready for incremental development, but the current system is a simulation and DWAL candidate-generation fixture rather than a complete autonomous, crowd-aware Nav2 stack.

Verified working now:

- The iWalk model launches in the café together with HuNav pedestrians.
- Odometry, simulated lidar, the Nav2 obstacle costmap, DWAL sampled paths, and DWAL clusters are operational.
- The live runtime smoke test passes all five existing checks: odometry, lidar, costmap, paths, and clusters.
- The robot can be driven manually through `/cmd_vel` using keyboard teleoperation.
- The generated TF tree and the derived footprint are internally consistent.
- A Docker image provides the known-working ROS 2 Humble and Gazebo Classic environment.

Not implemented yet:

- A full Nav2 planner/controller pipeline.
- A DWAL component that selects a trajectory and publishes a safe command.
- DWB or HATEB running on the same iWalk scenario.
- `bt_navigator` or a robot mission behavior tree.
- Pedestrian tracking, context estimation, intended-user management, or front-following.
- A real motor, ros2_control, localization, or battery integration.
- A controlled-stop and stale-input watchdog path.
- A genuinely headless simulation mode.
- A common evaluation harness for comparing controllers.

Recommended development order:

1. Turn the existing DWAL fixture into one safe and reproducible command-generating baseline.
2. Establish common Nav2, tracking, command, and safety interfaces.
3. Bring DWB and HATEB onto the same iWalk scenario.
4. Add simulator-derived pedestrian tracks, intended-user state, and context.
5. Implement and evaluate dynamic-radius and time-aware DWAL extensions.

## 2. Repository condition

The actual source workspace currently contains two ROS packages:

- `iwalk_description`
- `crowd_aware_simulation`

The package structure described in the root `README.md` is aspirational. The described interface, context, front-following, DWB critic, HATEB extension, bringup, and evaluation packages do not exist yet.

Documentation and dependency inconsistencies found during review:

- The README points to `docs/decisions.md`, but the existing file is `doc/decisions.md`.
- The README describes a `dependencies.repos` file, but no such manifest exists.
- `.gitmodules` references `src/kiro_nav`, but that submodule is not present.
- The README presents DWB and HATEB as the project approaches, while the currently executable work concerns DWAL.
- The project decision file targets Ubuntu 24.04 and ROS 2 Jazzy, while the working simulator container uses ROS 2 Humble and Gazebo Classic.

These inconsistencies should be corrected early. The repository needs one source of truth for architecture, experiment definitions, ROS distribution, external repositories, and pinned dependency revisions.

No repository code was changed during the review itself. Existing user modifications and untracked files were preserved.

## 3. Current runtime

The current launch file is `src/crowd_aware_simulation/launch/dwal_cafe.launch.py`.

It starts:

- HuNav loader, world generator, agent manager, and Gazebo plugin;
- `gzserver` and optionally `gzclient`;
- `robot_state_publisher` and `joint_state_publisher`;
- one standalone `nav2_costmap_2d` node;
- one lifecycle manager that manages only the costmap;
- `dwal_generator` and `dwal_clustering`;
- RViz when enabled.

The live runtime also contained `teleop_twist_keyboard` during inspection.

It does not start:

- `planner_server`;
- `controller_server`;
- `bt_navigator`;
- `behavior_server`;
- `smoother_server`;
- `velocity_smoother`;
- `collision_monitor`;
- `waypoint_follower`;
- `map_server`;
- AMCL.

The live ROS action list was empty. This confirms that the current launch is a DWAL visualization and teleoperation fixture, not a complete NavigateToPose system.

The existing runtime check reported:

```text
odometry: PASS
lidar: PASS
costmap: PASS
paths: PASS
clusters: PASS
```

The check validates:

- `/odom` uses `odom` and `sim_base` frames;
- `/scan` contains finite lidar returns in `laser_frame`;
- the local costmap uses `odom` and contains lethal obstacle cells;
- DWAL publishes non-empty sampled paths;
- a near-cluster message is received.

It does not validate autonomous goal completion, command selection, physical stopping, visibility of every pedestrian, stale-input safety, target tracking, or full Nav2 operation.

At the end of the audit, an expected `iwalk_dwal` container was running. It was inspected in place and was not stopped or replaced.

## 4. DWAL status

DWAL is currently not loaded as a `nav2_core::Controller` plugin.

It operates as two normal ROS nodes:

| Node | Inputs | Outputs |
|---|---|---|
| `dwal_generator` | `/odom`, `/local_costmap/costmap` | sampled paths and path markers |
| `dwal_clustering` | sampled paths | near/far cluster groups and markers |

Confirmed DWAL topics:

- `/dwal_planner/sampled_paths`
- `/dwal_planner/sampled_pathMarkers`
- `/dwal_planner/clusters_near`
- `/dwal_planner/clusters_far`
- `/dwal_planner/cluster_markers_near`
- `/dwal_planner/cluster_markers_far`

DWAL also exposes two custom services:

- `/dwal_planner/toggle_cluster_spin`
- `/dwal_planner/toggle_cluster_slice`

There are no DWAL actions.

The critical limitation is that DWAL does not currently select a candidate and publish `/cmd_vel`. Keyboard teleoperation publishes directly to the Gazebo planar movement plugin. Consequently, the current DWAL output cannot yet be compared fairly with DWB or HATEB, which are command-producing controller implementations.

The Docker build pins `gmoustri/dwal_planner` to commit:

```text
95b1aa5e1f0259ae8a467c731bf994fc0ebc4f6d
```

The configured DWAL levels are `[1.0, 1.8]`. These are read during initialization. The existing implementation is not designed for a continuously changing radius through a simple runtime parameter update.

## 5. Nav2 components and their role

Nav2 separates planning, control, task orchestration, recovery behavior, and command safety into different servers. Reference: [Nav2 Navigation Servers](https://docs.nav2.org/jazzy/getting_started/navigation_concepts/navigation_servers/).

| Component | Purpose | Project need |
|---|---|---|
| Planner Server | Produces a global path | Required for autonomous start-to-goal experiments |
| Controller Server | Hosts local controller plugins such as DWB or HATEB | Required for DWB/HATEB |
| BT Navigator | Orchestrates navigation actions and recovery logic | Required for autonomous mission execution |
| Behavior Server | Provides recovery/task behaviors | Useful after the basic navigation path works |
| Smoother Server | Improves global path geometry | Optional initially |
| Velocity Smoother | Applies velocity and acceleration constraints | Recommended |
| Collision Monitor | Final command-level safety filter | Strongly recommended |
| Waypoint Follower | Executes multiple waypoints | Not required initially |
| Route Server | Follows a predefined navigation graph | Not required for this project phase |
| Map Server and AMCL | Static map and localization | Needed for mapped global navigation |
| Lifecycle Manager | Configures and activates lifecycle nodes | Already used only for the standalone costmap |

[DWB](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/controller_plugins/dwb_controller/) is a Nav2 controller plugin hosted by Controller Server. HATEB follows the same broad integration pattern. Current DWAL does not.

A safe future command pipeline should be:

```text
selected controller or shared controller
                  ↓
          velocity smoother
                  ↓
          collision monitor
                  ↓
 ros2_control diff_drive_controller
                  ↓
            motor hardware
```

Controllers should not all publish directly to the same `/cmd_vel`. Use explicit remappings, a command mux, or Nav2 controller selection. Nav2 provides a [ControllerSelector BT node](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/core_servers/bt_plugins/actions/ControllerSelector/) for choosing among configured controller plugins.

## 6. Lidar implementation

The lidar is generated by `src/crowd_aware_simulation/scripts/prepare_scene.py`; it is not defined in the original iWalk Xacro.

Current configuration:

- frame: `laser_frame`;
- parent frame: `sim_base`;
- position: `x=0.85`, `y=0`, `z=0.35` metres;
- orientation: zero roll, pitch, and yaw;
- sensor: Gazebo `gpu_ray`;
- field of view: 360 degrees;
- samples: 720;
- update rate: 15 Hz;
- range: 0.05–10 m;
- output: `/scan` as `sensor_msgs/LaserScan`.

The sensor plugin is hosted inside `gzserver`; there is no separate physical lidar driver node. The live `/lidar_ros` node is provided by the Gazebo ROS ray plugin.

The Nav2 obstacle layer consumes `/scan`, ray-traces free space, marks obstacles, inflates costs, and publishes `/local_costmap/costmap`. DWAL consumes this costmap rather than the raw scan.

The current lidar pose is duplicated in:

1. the generated URDF fixed joint;
2. the SDF/Gazebo sensor pose;
3. the generated geometry metadata.

Before relocating the sensor, these values should be refactored into one `laser_xyz` and `laser_rpy` configuration source.

Procedure after measuring the real mount:

1. Express the measured pose relative to the selected robot base/rear-axle frame.
2. Update the single pose source.
3. Regenerate the runtime URDF and Gazebo world.
4. Verify `sim_base → laser_frame` using TF tools.
5. Inspect `/scan` in RViz for ground returns and self-occlusion.
6. Confirm that the scan plane intersects furniture and simulated people appropriately.
7. Regenerate the footprint if the physical sensor housing extends beyond it.
8. Run the offline scene test and the live smoke check.

For the real robot, the measured fixed joint should live in the main robot description, and an actual lidar driver should publish `/scan`.

## 7. Footprint and TF tree

The original Xacro describes `base_footprint` as the point on the ground under the rear axle. Its fixed transform places `base_link` 0.39 m relative to it.

The simulation generator creates a runtime `sim_base` at the rear-axle projection, reverses/reroots the necessary fixed relationship, and preserves all physical relative transforms.

The generated conservative footprint is approximately:

```text
front:  x =  0.865 m
rear:   x = -0.103 m
left:   y =  0.327 m
right:  y = -0.327 m
padding = 0.02 m
```

Therefore, the footprint is referenced to the rear axle, but it is not geometrically centered around it. Most of the physical body extends forward.

The same generated polygon is used by DWAL and the Nav2 costmap. This is important: inconsistent controller and costmap footprints would invalidate both safety behavior and comparisons.

The runtime TF tree is real and is assembled by two publishers:

```text
Gazebo planar plugin:
odom → sim_base

robot_state_publisher:
sim_base → base_link → wheels/base_footprint
         → laser_frame
```

For the physical robot, the normal tree should be:

```text
map → odom → base frame → sensors and wheel links
```

- AMCL commonly provides `map → odom` when using a static map.
- Wheel odometry or an EKF provides `odom → base`.
- Robot State Publisher provides transforms from the base to sensors and robot links.
- Exactly one component should publish `odom → base`.

The offline regression checks that the generated model has a unique TF root, preserves physical transforms, uses the same conservative footprint for DWAL and the costmap, and contains the expected sensor and odometry plugins.

## 8. Why the robot stops at obstacles

The current stopping behavior against static café objects comes from Gazebo physics, not from DWAL or a shared controller.

The scene generator creates a physical `navigation_envelope` collision box around the iWalk model. The ideal planar movement plugin commands the model, but Gazebo contact resolution prevents this collision body from passing through static café collision geometry.

This physical collision can technically be disabled by removing the generated collision element or altering Gazebo collision masks. That should only be done for a diagnostic penetration experiment. It should remain enabled during normal development.

The HuNav world generator is currently configured with `use_collision=False`. Therefore:

- static café geometry physically blocks the robot;
- pedestrians can be visible to the simulated lidar;
- pedestrians should not be assumed to physically block or push the robot in the same way.

Gazebo physics is not a substitute for command-level safety. A future stack should add Nav2 [Collision Monitor](https://docs.nav2.org/jazzy/tutorials/general_tutorials/using_collision_monitor/using_collision_monitor/) after the selected controller and velocity smoother.

## 9. Docker and headless execution

The working image uses ROS 2 Humble and Gazebo Classic because the reused PMB2/HuNav environment was built on that stack. The host and project target ROS 2 Jazzy.

The current runner:

- requires `DISPLAY`;
- mounts the X11 socket;
- uses software OpenGL;
- uses ROS domain ID 73;
- runs the container with `--rm`;
- supports `build`, `run`, `shell`, `teleop`, and `check` modes.

Docker should remain part of development now rather than being added only at the end. Source code should stay in the repository and images should be rebuilt from it. Development changes should not live only inside a running container.

The current setup is not genuinely headless. Disabling RViz and `gzclient` does not remove the rendering requirement of the GPU lidar.

Two possible headless strategies are:

1. Install Xvfb and run the launch under `xvfb-run` with `gui:=false rviz:=false`.
2. Replace the GPU-ray sensor with a CPU/no-rendering ray sensor, after verifying that its timing and output are acceptable.

Gazebo Classic reached end of life in January 2025. ROS 2 Jazzy is normally paired with modern Gazebo Harmonic. References: [Gazebo Classic](https://classic.gazebosim.org/) and [ROS/Gazebo installation compatibility](https://gazebosim.org/docs/garden/ros_installation/).

For the immediate weekly demonstrations, retaining the known-working Humble/Classic fixture is reasonable. Migration should be recorded as a milestone and should not block the first controller experiments.

## 10. Behavior trees and battery

There is currently no robot behavior tree and no `bt_navigator`. HuNav pedestrian behavior is unrelated to robot mission control.

The Python battery example in the review brief should not become a replacement for Nav2 BT Navigator. It also contains indentation, message-type, and undefined-class problems.

Nav2 already provides an [`IsBatteryLow`](https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/core_servers/bt_plugins/conditions/IsBatteryLow/) BT condition. It can be used in a navigation or mission tree when a valid `sensor_msgs/BatteryState` topic exists.

The BT does not need raw BMS details. A battery driver does need:

- the BMS communication protocol;
- pack voltage range and cell count;
- state-of-charge source/calibration;
- current sign convention;
- presence, fault, and health information;
- timestamps and communication timeout;
- low and critical thresholds;
- defined behavior for stale or invalid data.

Development should begin with a simulated `BatteryState` publisher and BT tests. Real BMS integration belongs to the hardware phase.

## 11. Hardware control and EKF

There is no `iwalk_hardware` package or ros2_control setup. Current motion comes from an ideal Gazebo planar plugin, not wheel dynamics. The simulation does not represent motor torque, encoder error, wheel slip, caster dynamics, or realistic wheel motion.

The intended hardware architecture is:

```text
controller_manager
  ├── iWalk ros2_control hardware plugin
  │     └── motor/CAN/VESC/encoder communication
  └── diff_drive_controller
        ├── consumes velocity commands
        ├── commands wheel velocities
        └── publishes wheel odometry
```

`iwalk_hardware` should normally be a ros2_control hardware plugin loaded by Controller Manager, not a normal node that directly calls the differential-drive controller. The official [`diff_drive_controller`](https://control.ros.org/jazzy/doc/ros2_controllers/diff_drive_controller/doc/userdoc.html) already supplies differential-drive kinematics, odometry, command timeouts, and limits.

The EKF and differential-drive controller are not alternatives:

- `diff_drive_controller` controls the wheels and produces wheel odometry;
- `robot_localization` EKF fuses wheel odometry, IMU, and optional visual/other odometry into a better robot state estimate.

Use wheel odometry first. Add an EKF when the real IMU or other state sensors are integrated, especially to improve heading and mitigate encoder noise or slip. See the [`robot_localization` state estimation documentation](https://github.com/cra-ros-pkg/robot_localization/blob/rolling-devel/doc/state_estimation_nodes.rst).

If the EKF publishes `odom → base_link`, disable the same TF publication in the differential-drive controller to avoid duplicate TF authorities.

## 12. Crowd perception and context design

Context estimation should be a continuous, timestamped dataflow rather than a synchronous service on the controller hot path.

```text
HuNav states or real sensor trackers
                 ↓
      canonical pedestrian tracks
                 ↓
         context estimator ──────────┐
                 ↓                   │
      context state/profile          │
                 ↓                   │
front-following manager ← intended-user track
                 ↓
      desired front-following pose
                 ↓
       DWB / HATEB / DWAL adapter
```

### Track adapter

- In simulation, convert HuNav `/human_states` to the canonical tracking message.
- On the real robot, convert camera/RGB-D/lidar tracker output to the same interface.
- Reuse `cohan_msgs/TrackedAgents` initially if it provides enough information for HATEB.
- Add project-specific interfaces only for genuinely missing concepts such as confidence, group membership, intended-user identity, or desired formation state.

### Context estimator

Subscribe to:

- pedestrian tracks;
- local costmap/geometry;
- robot pose and TF;
- tracking confidence and staleness.

Publish a timestamped context/profile at approximately 5–10 Hz. Services are suitable for manual override, reset, or query, not continuous controller input.

Recommended context set:

- `OPEN_AREA`
- `NARROW_CORRIDOR`
- `DOORWAY`
- `JUNCTION`
- `DENSE_CROWD`
- `OCCLUSION`
- `UNKNOWN`
- `STALE`

Situations may overlap—for example, a narrow corridor can also contain a dense crowd. A score/multi-label representation plus a dominant operating profile is more flexible than one exclusive enum. Add hysteresis and minimum dwell time to prevent rapid profile switching.

Profiles should be stored in YAML rather than hard-coded controller logic.

A 2D lidar can detect geometry and support leg or motion tracking, but it is not sufficient for robust semantic identity and re-identification after occlusion. Camera or RGB-D data helps classify and identify people. Vision should not block controller development: use HuNav ground-truth tracks first and replace that source with real perception later.

## 13. Controller experiment design

The proposed controller families are appropriate:

1. DWB with project-specific social and formation critics.
2. HATEB with pedestrian prediction and front-following constraints.
3. DWAL, including a dynamic-horizon extension.

The proposed eight conditions form a useful factorial experiment:

| Controller | Crowd semantics OFF | Crowd semantics ON |
|---|---:|---:|
| Fixed-radius DWAL | yes | yes |
| DWB | yes | yes |
| HATEB | yes | yes |
| Dynamic-radius DWAL | yes | yes |

“Crowd semantics OFF” must not mean disabling physical obstacle sensing.

- **OFF:** pedestrians remain anonymous lidar obstacles; no identities, velocities, predictions, context, or social scoring are provided.
- **ON:** the same pedestrians additionally have structured tracks, identities, predictions, context, and controller-specific social behavior.

Current gaps:

- Vanilla DWAL ignores structured pedestrian data, so ON and OFF would currently be identical.
- DWB ON requires social/formation critics or a shared social costmap layer.
- HATEB ON requires tracked agents; OFF should use empty tracks and disabled human constraints while retaining lidar obstacles.
- Dynamic-radius DWAL requires implementation.

All controllers must share:

- the same iWalk geometry and footprint;
- the same velocity and acceleration limits;
- the same start, goal, and global path;
- the same crowd scenarios and paired random seeds;
- the same sensing/noise assumptions;
- the same velocity-smoothing and collision-monitor path;
- the same failure criteria and logging.

A PMB2 HATEB demonstration cannot be used quantitatively against an iWalk DWAL demonstration because the robot, limits, topics, and system architecture differ.

## 14. Dynamic-radius DWAL

The existing DWAL levels are static initialization parameters. A continuously changing radius requires modifications to trajectory generation, clustering, marker allocation, and possibly message design.

A defensible dynamic horizon should include current speed and braking distance rather than only context and maximum velocity:

```math
R_stop(v,c) = r_front + m(c) + v τ(c) + v²/(2 a_brake)
```

```math
R_preview(v,c) = v T_preview(c)
```

```math
R_dynamic = clamp(
  max(R_stop, R_preview),
  R_min,
  min(R_sensor_reliable, R_costmap)
)
```

Where:

- `r_front` is the forward physical extent of the robot;
- `m(c)` is a context-dependent margin;
- `τ(c)` is a reaction/processing allowance;
- `a_brake` is verified braking deceleration;
- `T_preview(c)` is the desired prediction horizon;
- the upper bounds reflect reliable sensing and available costmap extent.

The first low-risk baseline should use several configured fixed DWAL levels and enable/select among them using the existing toggle services. Continuous radius changes should follow once the discrete design is understood and tested.

The stronger research direction is time-parameterized DWAL candidate scoring against predicted pedestrian trajectories and uncertainty. Dynamic radius can then be evaluated as one ablation rather than being the complete contribution.

## 15. Related laboratory repositories

### 15.1 `bayesian_shared_control`

This repository is immediately useful. It consumes near/far DWAL clusters, odometry, and an operator command, then produces an assisted command. It can turn the current path-generation fixture into a command-generating demonstration without implementing a selector from scratch.

The current public version successfully test-built against the existing Humble/DWAL image during this review.

Issues to address before integration:

- The launch node name/namespace and YAML parameter root appear inconsistent, so configured parameters may not apply.
- A declared cluster-topic parameter is ignored in favor of hard-coded near/far topics.
- There are no watchdogs for stale odometry, clusters, or user commands.
- Some no-cluster and reverse-motion cases are pass-through behavior rather than controlled safety behavior.
- `Twist` versus `TwistStamped` must be verified; the current Gazebo plugin consumes `Twist`.
- The repository lacks sufficient automated tests.

This is operator shared control, not front-following. It should be reused as the first DWAL selection/control baseline, not treated as the final research controller.

### 15.2 `kiro_nav`

This is useful as the laboratory's complete Nav2/HATEB reference. It contains controller, planner, smoother, behavior, BT Navigator, waypoint follower, velocity smoother, human bridge, and prediction components.

It is not a drop-in iWalk stack:

- frames and odometry topics differ;
- scan and tracked-agent interfaces differ;
- the configured footprint and limits are PMB-like;
- it expects localization, a map, and a global path;
- several dependencies are not pinned;
- the velocity-smoother/remapping chain needs verification;
- the repository documents an upstream HATEB assertion problem in some close crossings.

Use it as the HATEB architecture/reference demonstration. Then adapt HATEB to the common iWalk topics, footprint, limits, and safety chain.

### 15.3 `pmb2_hunav_simulation`

This is useful as a simulator and evaluation fixture, not as another controller.

It provides:

- a known PMB2/HuNav environment;
- crowd scenarios;
- conversion from HuNav states to tracked agents;
- the HuNav evaluator;
- the base environment already reused by the current iWalk Docker image.

The HuNav evaluator can provide:

- goal completion and final goal distance;
- elapsed time and path length;
- average/minimum pedestrian distance;
- intimate, personal, social, and group-space intrusion;
- robot-person collisions;
- freezing behavior;
- speed, acceleration, and excessive acceleration;
- social-force/work indicators.

It should be extended only for missing project-specific metrics such as front-formation error, intended-user identity retention, target loss, TTC, static collisions, and controller computation time.

## 16. Evaluation metrics

Recommended first dashboard:

- navigation success/failure;
- robot-person collisions;
- static-obstacle collisions;
- minimum distance to any person;
- time spent in intimate or personal space;
- completion time;
- path length;
- mean and 95th-percentile longitudinal formation error;
- mean and 95th-percentile lateral formation error;
- freeze count and total freeze time;
- linear/angular jerk or command variation;
- controller computation time and deadline misses.

Later metrics:

- intended-user retention percentage;
- identity-switch count;
- target-loss and reacquisition time;
- TTC and near-miss count;
- group splitting or socially invalid passing;
- emergency collision-monitor interventions;
- CPU and memory use;
- categorized failure reasons.

Experiments should use paired repeated random seeds and report distributions or confidence intervals. A visually successful single run is useful as a demonstration but not evidence that one controller is better.

## 17. Research contribution

At present, the work is mostly research infrastructure and systems integration. That is necessary, but integrating DWB, HATEB, Docker, DWAL, and HuNav is not by itself a strong algorithmic contribution.

Promising research directions are:

1. **Uncertainty-aware, time-indexed DWAL:** score candidate arcs against predicted pedestrian occupancy instead of only a current static costmap.
2. **Principled adaptive horizon:** derive sampling radius from braking distance, sensor reliability, context, and crowd motion.
3. **Front-following under occlusion:** retain intended-user identity and socially appropriate formation during temporary loss.
4. **Context-conditioned control:** adapt speed, formation, horizon, prediction, and social weights with hysteresis and measurable hypotheses.
5. **Fair cross-controller evaluation:** compare geometric sampling, critic scoring, and joint trajectory optimization under identical inputs and safety layers.

Dynamic radius alone is likely a modest contribution. Dynamic radius as part of an uncertainty-aware, time-dependent front-following controller, supported by controlled ablations, is much stronger.

## 18. Prioritized development backlog

### P0 — Reproducibility and safety

- Correct README paths and distinguish current implementation from planned architecture.
- Add a dependency manifest with exact repository SHAs.
- Record the Humble/Classic versus Jazzy/Harmonic decision and migration plan.
- Add genuine headless execution.
- Add command timeout and controlled-stop behavior.
- Extend smoke tests with TF, output command, and stale-data safety checks.

### P1 — First operational DWAL baseline

- Integrate and repair `bayesian_shared_control`.
- Give each command source a private input/output topic.
- Add explicit command selection or muxing.
- Add velocity smoothing and Collision Monitor.
- Guarantee zero command for stale odometry, stale clusters, lost controller, or no valid candidate.
- Log the selected DWAL candidate and its selection reason/score.

### P2 — Common autonomous baseline

- Create a minimal Nav2 bringup package.
- Add global planning and Controller Server.
- Run upstream DWB on the iWalk café.
- Adapt Kiro/HATEB to the same frames, topics, footprint, and limits.
- Ensure every controller reaches the same goal through the same command and safety pipeline.

### P3 — Crowd interfaces

- Add a HuNav-to-tracked-agents adapter.
- Define intended-user and desired-front-pose interfaces.
- Add a front-following manager.
- Add context estimation with confidence, staleness, scores, and hysteresis.
- Implement explicit semantic ON/OFF experimental switches.

### P4 — Research extensions

- Add front-following/social DWB critics.
- Add HATEB front-formation constraints.
- Add discrete and then continuous dynamic DWAL horizons.
- Add time-indexed pedestrian prediction and uncertainty.
- Run the complete eight-condition ablation matrix.

## 19. Realistic weekly demonstration

A credible short-term demonstration should contain three evidence levels:

1. **iWalk plus current DWAL:** show lidar, costmap, sampled arcs, clusters, and the passing smoke checks.
2. **iWalk plus DWAL shared selection:** integrate the existing shared controller after correcting namespace, message type, and watchdog issues so that it produces an assisted velocity command.
3. **Reference baselines:** run upstream DWB and the existing Kiro/PMB2 HATEB demonstration, clearly identifying the latter as a non-comparable reference until it is ported to iWalk.

The presentation should not yet claim which controller performs better. It can accurately report:

- which components run;
- which interfaces differ;
- which failures or integration problems were observed;
- what must be normalized before comparison;
- the proposed metrics and controlled experiment matrix.

## 20. Immediate first development task

The first implementation milestone should be the safe DWAL command pipeline:

1. Integrate the existing Bayesian shared controller.
2. Correct its parameter namespace and topic configuration.
3. Confirm the command message type expected by Gazebo.
4. Add stale-input watchdogs and a guaranteed zero-command path.
5. Route its output through a velocity smoother and Collision Monitor.
6. Extend the smoke check to validate the selected path, command output, timeout, and controlled stop.

This milestone reuses the working simulation and laboratory implementation, exposes the missing safety and interface assumptions, and creates the foundation needed before context recognition or dynamic-radius research can be evaluated.
