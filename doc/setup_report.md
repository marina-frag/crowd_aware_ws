# Local-only crowd-aware café implementation report

Date of validation: 2026-09-22
Repository revision at validation: `c69445ea4d994a40ae57a38f240efbfd4682dc4c` plus the working-tree changes described here.

## 1. Delivered scope

This implementation provides the requested eight local-controller conditions in the existing iWalk/HuNav café simulation:

| Controller family | Semantics OFF | Semantics ON |
|---|---:|---:|
| Fixed-radius DWAL plus Bayesian shared control | implemented | implemented |
| Nav2 DWB | implemented | implemented |
| CoHAN HATEB | implemented | implemented |
| Runtime dynamic-radius DWAL plus Bayesian shared control | implemented | implemented |

The launch selector is `controller:=fixed_dwal|dwb|hateb|dynamic_dwal` plus
`semantics:=off|on`. Only one controller publishes `/cmd_vel_selected` in a run. Every
condition uses the same downstream Nav2 velocity smoother, Nav2 collision monitor and
final timeout publisher before Gazebo.

The work intentionally remains local-only. It does not add AMCL, SLAM, map localization,
a global planner, a behavior-tree navigator, or a building navigation stack. The
deterministic task supplies a short path in `odom` directly to a Nav2 Controller Server.

The principal implementation files are:

- `src/crowd_aware_simulation/launch/dwal_cafe.launch.py`: common launch and condition selection.
- `src/crowd_aware_simulation/config/experiment.yaml`: common limits, task, context thresholds and radius profiles.
- `src/crowd_aware_simulation/scripts/prepare_scene.py`: generated URDF/world and per-condition Nav2/DWAL parameters.
- `src/crowd_aware_simulation/scripts/hunav_to_cohan_bridge.py`: HuNav ground-truth to CoHAN tracks.
- `src/crowd_aware_simulation/scripts/cohan_agents_info_adapter.py`: DWB-only compatibility side channel required by CoHAN social layers.
- `src/crowd_aware_simulation/scripts/context_estimator.py`: continuous explainable context estimates.
- `src/crowd_aware_simulation/scripts/dynamic_radius_policy.py` and `policy_core.py`: fixed/discrete/continuous horizon policy.
- `src/crowd_aware_simulation/scripts/local_task.py`: common path/action client and HuNav evaluator start/stop.
- `command_guard.py` and `final_cmd_watchdog.py`: bounded, timeout-safe command hand-off.
- `patches/dwal_dynamic_radius.patch`, `patches/bayesian_shared_control_safety_semantics.patch`, and
  `patches/cohan_hateb_local_frame.patch`: minimal changes to pinned laboratory dependencies.
- `scripts/run_dwal_cafe.sh`: build, launch, live check, bag recording and interactive teleoperation.

## 2. Exact run commands

Build once from the repository root:

```bash
bash scripts/run_dwal_cafe.sh build
```

The eight headless conditions, all using seed 1, are:

```bash
HEADLESS=true bash scripts/run_dwal_cafe.sh run fixed_dwal off 1
HEADLESS=true bash scripts/run_dwal_cafe.sh run fixed_dwal on 1

HEADLESS=true bash scripts/run_dwal_cafe.sh run dwb off 1
HEADLESS=true bash scripts/run_dwal_cafe.sh run dwb on 1

HEADLESS=true bash scripts/run_dwal_cafe.sh run hateb off 1
HEADLESS=true bash scripts/run_dwal_cafe.sh run hateb on 1

HEADLESS=true bash scripts/run_dwal_cafe.sh run dynamic_dwal off 1
HEADLESS=true bash scripts/run_dwal_cafe.sh run dynamic_dwal on 1
```

Use `GUI=true HEADLESS=false` to launch the Gazebo client and `RVIZ=true` to launch
RViz. The default dynamic mode is continuous. The implemented discrete baseline can be
selected with:

```bash
RADIUS_MODE=discrete bash scripts/run_dwal_cafe.sh run dynamic_dwal off 1
RADIUS_MODE=discrete bash scripts/run_dwal_cafe.sh run dynamic_dwal on 1
```

Run the matching read-only runtime gate promptly from a second terminal, while the task
is moving:

```bash
bash scripts/run_dwal_cafe.sh check fixed_dwal off 1
bash scripts/run_dwal_cafe.sh check fixed_dwal on 1
bash scripts/run_dwal_cafe.sh check dwb off 1
bash scripts/run_dwal_cafe.sh check dwb on 1
bash scripts/run_dwal_cafe.sh check hateb off 1
bash scripts/run_dwal_cafe.sh check hateb on 1
bash scripts/run_dwal_cafe.sh check dynamic_dwal off 1
bash scripts/run_dwal_cafe.sh check dynamic_dwal on 1
```

The checker is a live observer. If it is started after a short task has already ended,
the latched task result and robot displacement still pass, but its
`nonzero_selected_command` item correctly reports that it did not personally observe a
nonzero command.

Record a trial from another terminal:

```bash
bash scripts/run_dwal_cafe.sh record dynamic_dwal on 1
```

That bag includes odometry, lidar, HuNav states, controller/logging tracks, CoHAN agent
state, context, dynamic-radius state and speed limit, task/safety status, path, DWAL
samples/clusters, HATEB timing-related outputs, and all command stages.

Interactive teleoperation is deliberately separate from quantitative trials:

```bash
REFERENCE_MODE=teleop bash scripts/run_dwal_cafe.sh run fixed_dwal on 1
# In a second terminal:
bash scripts/run_dwal_cafe.sh teleop
```

The keyboard publishes `/reference_cmd`. It never publishes the final Gazebo command.

### Paired repeated experiments

Use identical seed sets and a fresh container for every condition. A simple terminal
workflow is:

```bash
for seed in 1 2 3 4 5; do
  for condition in \
    "fixed_dwal off" "fixed_dwal on" \
    "dwb off" "dwb on" \
    "hateb off" "hateb on" \
    "dynamic_dwal off" "dynamic_dwal on"; do
    set -- $condition
    timeout --signal=INT --kill-after=15s 90s \
      bash scripts/run_dwal_cafe.sh run "$1" "$2" "$seed"
  done
done
```

Ninety seconds is a wall-time guard, not command replay; the reference reacts to robot
progress. Each fresh launch resets the robot/world and passes the seed to
`gzserver --seed`. HuNav pedestrians react to the robot, so paired seeds reproduce
initial random choices but do not guarantee identical realized pedestrian trajectories
after the robot policies diverge. For formal data collection, run the bag command in a
second terminal and stop it only after `/experiment/task_result` and the evaluator
"metrics stored" log appear.

## 3. Working environment and dependency revisions

The working and validated environment is ROS 2 Humble on Ubuntu Jammy with Gazebo
Classic, not host Jazzy:

- `ROS_DISTRO=humble`
- Gazebo Classic `gzserver 11.10.2`; Debian `libgazebo11 11.10.2+dfsg-1`
- Nav2 `1.1.20` packages from the Humble apt repository:
  - `ros-humble-nav2-controller 1.1.20-1jammy.20260908.005456`
  - `ros-humble-nav2-dwb-controller 1.1.20-1jammy.20260908.012626`
  - `ros-humble-nav2-velocity-smoother 1.1.20-1jammy.20260908.000910`
  - `ros-humble-nav2-collision-monitor 1.1.20-1jammy.20260908.010715`
  - `ros-humble-navigation2 1.1.20-1jammy.20260612.214117`

Pinned source revisions, also recorded in `dependencies.repos`:

| Dependency | Revision | Use |
|---|---|---|
| `gmoustri/dwal_planner` | `95b1aa5e1f0259ae8a467c731bf994fc0ebc4f6d` | trajectory generation and clustering |
| `gmoustri/bayesian_shared_control` | `9d0df25f8a966db449ec94c073349d09e328d517` | shared selector |
| `sphanit/CoHAN-Nav2` | `094f095a4cb0dfb774730c6020750c72adef0773` | HATEB, social layers, prediction interfaces |
| `rst-tu-dortmund/costmap_converter` | `9565858432664e9cfe0b3556e1809336c4ab06d4` | HATEB dependency |
| `andvatistas/kiro_nav` | `a6de7d52386910a9db0abd439bcafb92b992e99c` | inspected HATEB integration reference |
| `andvatistas/pmb2_hunav_simulation` | `770c5af36e860edb471fe2431eedcf6c9c3bb5d4` | container base, HuNav adapter/evaluator reference |
| `osrf/gazebo_models` | `8163eb4b5e7e21985c6591d1c0bfb56468c0093f` | café, table and ground-plane assets |

The final pinned image build completed successfully. It does not mix Humble and Jazzy
packages.

## 4. Actual TF tree and publishers

The runtime robot tree preserves the requested authority split:

```text
odom
└── sim_base                         ideal_planar_base Gazebo plugin
    ├── base_link                    robot_state_publisher, fixed
    │   └── original iWalk links     robot_state_publisher from the rerooted URDF
    └── laser_frame                  robot_state_publisher, fixed
```

Observed transforms:

- `odom -> sim_base`: live planar transform from `ideal_planar_base`. One sample was
  translation `(3.315, -0.200, 0.020)` after a trial.
- `sim_base -> base_link`: `(0, 0, 0.095)`, identity rotation.
- `sim_base -> laser_frame`: `(0.850, 0, 0.350)`, identity rotation.

`/tf_static` had exactly one publisher, `robot_state_publisher`. `/tf` had three
publishers, but for disjoint transforms: `ideal_planar_base` owns the robot odometry
edge, `robot_state_publisher` owns non-fixed robot joints, and `hunav_agent_manager`
owns pedestrian transforms. The generated URDF regression test verifies a single robot
root and no duplicate child transform. No second odometry publisher was introduced.

The offline expansion also verified that rerooting preserves every original physical
relative transform.

## 5. Command and task flow

### DWB and HATEB

```text
local_task: odom-frame Path
  -> /follow_path (Nav2 FollowPath action)
  -> controller_server [DWB or HATEB]
  -> /cmd_vel_selected
  -> command_guard
  -> /cmd_vel_guarded
  -> nav2_velocity_smoother
  -> /cmd_vel_smoothed
  -> nav2_collision_monitor
  -> /cmd_vel_safe
  -> final_cmd_watchdog
  -> /cmd_vel
  -> Gazebo ideal_planar_base
```

### DWAL shared-control conditions

```text
local_task: same odom-frame Path
  -> /reference/follow_path
  -> reference/controller_server [Nav2 Regulated Pure Pursuit]
  -> /reference_cmd
  + DWAL generator -> sampled paths
  + DWAL clustering -> near/far clusters
  -> existing Bayesian shared_controller
  -> /cmd_vel_selected
  -> identical guard, smoother, collision monitor and watchdog chain
  -> /cmd_vel -> Gazebo
```

Thus the evaluated DWAL system includes the deterministic nominal-reference policy and
the Bayesian selector; system-level results must not be attributed to DWAL trajectory
generation alone.

The common task path is
`[(0,0), (0.8,0), (1.6,0.15), (2.4,0.15), (3.2,0)]` in `odom`, with 0.20 m goal
tolerance. It is progress-reactive, not time-replayed.

`/cmd_vel` was observed with exactly one publisher, `final_cmd_watchdog`, and exactly
one subscriber, `ideal_planar_base`, in every checked condition.

## 6. Runtime node inventory

Common nodes in all eight conditions:

| Node | Package / executable | Role |
|---|---|---|
| `/gazebo` plus `gzserver` | `gazebo_ros` / Gazebo Classic | world, clock and plugin host |
| `/ideal_planar_base` | `gazebo_plugins` planar move plugin | final command, odometry and `odom -> sim_base` |
| `/lidar_ros` | `gazebo_plugins` ray sensor | `/scan_raw` |
| `/hunav_loader` | `hunav_agent_manager/hunav_loader` | scenario parameters |
| `/hunav_agent_manager` | `hunav_agent_manager/hunav_agent_manager` | pedestrian behavior |
| `/hunav_plugin` | HuNav Gazebo plugin | simulated people and ground-truth states |
| `/robot_state_publisher` | `robot_state_publisher` | robot TF |
| `/joint_state_publisher` | `joint_state_publisher` | joint states |
| `/scan_self_filter` | `laser_filters/scan_to_scan_filter_chain` | removes iWalk envelope from lidar |
| `/hunav_to_cohan_bridge` | project Python node | ground truth to controller/logging tracks |
| `/context_estimator` | project Python node | 8 Hz context scores/profile |
| `/dynamic_radius_policy` | project Python node | fixed/dynamic horizon and speed limit |
| `/command_guard` | project Python node | limits, reverse rejection and freshness stop |
| `/velocity_smoother` | `nav2_velocity_smoother` | common acceleration-limited smoothing |
| `/collision_monitor` | `nav2_collision_monitor` | common lidar/footprint approach intervention |
| `/final_cmd_watchdog` | project Python node | single final publisher and crash timeout |
| `/hunav_evaluator_node` | `hunav_evaluator` | metrics collection and CSV output |
| `/local_task` | project Python node | path/action client and evaluator orchestration |
| `/command_pipeline_lifecycle_manager` | `nav2_lifecycle_manager` | smoother and collision monitor |

Condition-specific nodes:

| Condition | Additional nodes |
|---|---|
| DWB OFF | `/controller_server`, `/controller_lifecycle_manager` |
| DWB ON | DWB OFF nodes plus `/cohan_agents_info_adapter`; CoHAN social layers are hosted inside `/local_costmap/local_costmap` |
| HATEB OFF/ON | `/controller_server`, `/controller_lifecycle_manager`, `/agent_path_prediction` |
| Fixed/dynamic DWAL OFF/ON | `/costmap/costmap`, `/costmap_lifecycle_manager`, `/dwal_planner/dwal_generator`, `/dwal_planner/dwal_clustering`, `/shared_controller`, `/reference/controller_server`, `/reference/reference_lifecycle_manager` |

CoHAN/HATEB creates helper node interfaces with duplicate displayed names
(`controller_server` and `agent_path_prediction`). This is an upstream implementation
detail observed in `ros2 node list`; it did not create extra `/cmd_vel` publishers.

## 7. Lifecycle nodes and verified states

| Lifecycle node | Manager/owner | Observed |
|---|---|---|
| `/velocity_smoother` | `/command_pipeline_lifecycle_manager` | `active [3]` |
| `/collision_monitor` | `/command_pipeline_lifecycle_manager` | `active [3]` before the intentional crash-timeout test |
| `/controller_server` | `/controller_lifecycle_manager` | `active [3]` for DWB/HATEB |
| `/local_costmap/local_costmap` | Controller Server-owned local costmap | `active [3]` |
| `/costmap/costmap` | `/costmap_lifecycle_manager` | `active [3]` in DWAL conditions |
| `/reference/controller_server` | `/reference/reference_lifecycle_manager` | `active [3]` in autonomous DWAL conditions |
| `/scan_self_filter` | laser_filters executable | data path verified, but its duplicated node name made the CLI lifecycle query non-responsive |

All controller and command-pipeline managers use autostart. The smoke checker queries
the applicable controller/costmap state plus smoother and collision-monitor states.
The scan filter's formal state is the one lifecycle item not conclusively read back;
`/scan` remained valid at approximately 13.5 Hz.

## 8. Topic inventory

Rates below are observed examples from headless runs; configured rates are in
parentheses where useful.

### Common sensing, state and experiment topics

| Topic | Type | Publisher -> principal subscribers | Purpose / observed rate |
|---|---|---|---|
| `/clock` | `rosgraph_msgs/msg/Clock` | Gazebo -> every sim-time node | simulation time |
| `/tf` | `tf2_msgs/msg/TFMessage` | Gazebo planar plugin, RSP, HuNav -> TF listeners | dynamic TF |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | RSP -> TF listeners | fixed robot TF |
| `/odom` | `nav_msgs/msg/Odometry` | `ideal_planar_base` -> controllers, costmaps, context, radius, evaluator adapters | `18.2 Hz observed, 20 Hz configured |
| `/joint_states` | `sensor_msgs/msg/JointState` | JSP -> RSP | robot joints |
| `/robot_description` | `std_msgs/msg/String` | RSP | expanded URDF |
| `/scan_raw` | `sensor_msgs/msg/LaserScan` | `lidar_ros` -> self filter | raw lidar, 15 Hz configured |
| `/scan` | `sensor_msgs/msg/LaserScan` | self filter -> obstacle layers, collision monitor, checker | `13.5 Hz observed |
| `/human_states` | `hunav_msgs/msg/Agents` | HuNav plugin -> bridge and evaluator | simulation ground truth |
| `/robot_states` | `hunav_msgs/msg/Agent` | HuNav plugin -> evaluator | evaluator robot state |
| `/tracked_agents_logging` | `cohan_msgs/msg/TrackedAgents` | bridge -> context/logging | always present; never a controller input |
| `/tracked_agents` | `cohan_msgs/msg/TrackedAgents` | bridge -> ON controller consumers | present only in ON |
| `/agents_info` | `agent_path_prediction/msg/AgentsInfo` | HATEB internal agent helper or DWB adapter -> HATEB/CoHAN layers | derived moving/stopped state and sorted distances |
| `/crowd_context` | `crowd_aware_interfaces/msg/ContextState` | context estimator -> logger/radius checker | `7.3 Hz observed, 8 Hz configured |
| `/crowd_context/profile` | `std_msgs/msg/String` | context estimator -> DWAL selector in ON | dominant profile |
| `/experiment/path` | `nav_msgs/msg/Path` | local task -> logging | common local task |
| `/experiment/task_result` | `std_msgs/msg/String` | local task -> checker/logger | STARTING/RUNNING/FINISHED, transient local |
| `/experiment/speed_limit` | `std_msgs/msg/Float64` | radius policy -> command guard | common admissible speed |
| `/experiment/dynamic_radius_state` | `crowd_aware_interfaces/msg/DynamicRadiusState` | radius policy -> logger/checker | requested/applied radius, profile, reason |
| `/experiment/command_guard_status` | `std_msgs/msg/String` | guard -> logger | forward/stop intervention reason |
| `/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | laser filter and ROS nodes | health diagnostics |
| `/performance_metrics` | `gazebo_msgs/msg/PerformanceMetrics` | Gazebo | simulator performance |
| `/parameter_events` | `rcl_interfaces/msg/ParameterEvent` | ROS nodes | standard parameter events |
| `/rosout` | `rcl_interfaces/msg/Log` | ROS nodes | logs |

### Command topics

| Topic | Type | Publisher -> subscriber |
|---|---|---|
| `/reference_cmd` | `geometry_msgs/msg/Twist` | reference RPP or keyboard -> Bayesian selector |
| `/cmd_vel_selected` | `geometry_msgs/msg/Twist` | exactly one selected controller -> command guard |
| `/cmd_vel_guarded` | `geometry_msgs/msg/Twist` | command guard -> velocity smoother |
| `/cmd_vel_smoothed` | `geometry_msgs/msg/Twist` | velocity smoother -> collision monitor |
| `/cmd_vel_safe` | `geometry_msgs/msg/Twist` | collision monitor -> final watchdog |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | final watchdog -> Gazebo planar plugin |

`/cmd_vel` was approximately 18.2 Hz in one observed run, versus 20 Hz configured.

### Costmap and collision topics

| Topic | Type | Purpose |
|---|---|---|
| `/local_costmap/costmap` | `nav_msgs/msg/OccupancyGrid` | full local grid; context and DWAL input |
| `/local_costmap/costmap_raw` | `nav2_msgs/msg/Costmap` | Nav2 costmap representation |
| `/local_costmap/costmap_updates` | `map_msgs/msg/OccupancyGridUpdate` | incremental updates |
| `/local_costmap/footprint` | `geometry_msgs/msg/Polygon` | input footprint interface |
| `/local_costmap/published_footprint` | `geometry_msgs/msg/PolygonStamped` | collision monitor polygon source |
| `/FootprintApproach` | `geometry_msgs/msg/PolygonStamped` | collision-monitor visualization |
| `/speed_limit` | `nav2_msgs/msg/SpeedLimit` | Nav2 controller speed-limit interface |
| `/obstacles` | `costmap_converter_msgs/msg/ObstacleArrayMsg` | HATEB obstacle conversion interface |
| `/invisible_humans_detection/invisible_humans_obs` | `costmap_converter_msgs/msg/ObstacleArrayMsg` | installed HATEB interface; invisible-human feature disabled |
| `/invisible_humans_detection/passage` | `cohan_msgs/msg/PassageType` | installed HATEB interface |

### DWAL/shared-controller topics

| Topic | Type | Purpose |
|---|---|---|
| `/dwal_planner/sampled_paths` | `dwal_planner/msg/SampledCluster` | generated candidate trajectories and active levels |
| `/dwal_planner/clusters_near` | `dwal_planner/msg/ClusterGroup` | near-level clusters to Bayesian selector |
| `/dwal_planner/clusters_far` | `dwal_planner/msg/ClusterGroup` | outer-level clusters to Bayesian selector |
| `/dwal/dynamic_radius` | `std_msgs/msg/Float64` | runtime outer radius to generator |
| `/shared_controller/intent_plot` | `visualization_msgs/msg/MarkerArray` | selector diagnostic |
| `/shared_controller/path_human` | `nav_msgs/msg/Path` | nominal/reference path diagnostic |
| `/shared_controller/path_shared` | `nav_msgs/msg/Path` | selected path diagnostic |
| DWAL marker topics | `visualization_msgs/msg/MarkerArray` | candidate/cluster visualization |

### DWB/HATEB controller and visualization topics

| Topic(s) | Type | Purpose |
|---|---|---|
| `/plan`, `/global_plan`, `/local_plan`, `/via_points` | `nav_msgs/msg/Path` | controller plan diagnostics |
| `/local_plan_poses`, `/agents_local_plans_poses`, `/crossing_points` | `geometry_msgs/msg/PoseArray` | HATEB visualization |
| `/local_traj` | `cohan_msgs/msg/TrajectoryStamped` | robot trajectory |
| `/agents_local_trajs` | `cohan_msgs/msg/AgentTrajectoryArray` | agent trajectories in HATEB ON |
| `/agents_global_plans`, `/agents_local_plans` | `cohan_msgs/msg/AgentPathArray` | HATEB agent plans |
| `/plan_time`, `/traj_time` | `cohan_msgs/msg/AgentTimeToGoal` | HATEB time-to-goal diagnostics, not CPU computation time |
| `/agents_plans_time`, `/agents_trajs_time` | `cohan_msgs/msg/AgentTimeToGoalArray` | per-agent time-to-goal |
| `/teb_feedback` | `hateb_local_planner/msg/FeedbackMsg` | HATEB graph feedback |
| `/planning_mode` | `hateb_local_planner/msg/PlanningMode` | HATEB mode diagnostic |
| `/crossing_info` | `cohan_msgs/msg/CrossingInfo` | crossing diagnostic |
| `/time_to_goal` | `std_msgs/msg/Float32` | HATEB time estimate |
| `/hateb_log` | `std_msgs/msg/String` | HATEB log channel |
| `/teb_markers`, `/agent_arrow`, `/agent_marker`, `/mode_text` | visualization messages | HATEB RViz output |
| `/robot_next_pose`, `/agent_next_pose` | `geometry_msgs/msg/PoseStamped` | prediction diagnostics |
| `/people` | `people_msgs/msg/People` | CoHAN compatibility interface |

The predictor also exposes
`/agent_path_prediction/external_agent_paths` (`cohan_msgs/msg/AgentPathArray`),
`/predicted_agent_poses` (`visualization_msgs/msg/MarkerArray`),
`/front_pose` (`geometry_msgs/msg/PoseStamped`), and `/predicted_goal`
(`agent_path_prediction/msg/PredictedGoals`).

Lifecycle transition topics exist for each lifecycle node and use
`lifecycle_msgs/msg/TransitionEvent`; `/bond` uses `bond/msg/Status`.

## 9. Service inventory

Application-specific services observed:

| Service | Type | Provider / purpose |
|---|---|---|
| `/hunav_start_recording` | `hunav_msgs/srv/StartEvaluation` | evaluator start with goal, condition tag and seed/run id |
| `/hunav_stop_recording` | `std_srvs/srv/Empty` | evaluator stop, compute and write metrics |
| `/compute_agent` | `hunav_msgs/srv/ComputeAgent` | HuNav behavior update |
| `/compute_agents` | `hunav_msgs/srv/ComputeAgents` | HuNav group update |
| `/get_agents` | `hunav_msgs/srv/GetAgents` | HuNav agent state |
| `/move_agent` | `hunav_msgs/srv/MoveAgent` | HuNav model motion |
| `/reset_agents` | `hunav_msgs/srv/ResetAgents` | reset pedestrian state |
| `/agent_path_prediction/predict_agent_poses` | `agent_path_prediction/srv/AgentPosePredict` | HATEB prediction request |
| `/agent_path_prediction/reset_prediction_services` | `std_srvs/srv/Empty` | reset prediction state |
| `/agent_path_prediction/set_agent_goal` | `agent_path_prediction/srv/AgentGoal` | prediction goal |
| `/optimize` | `cohan_msgs/srv/Optimize` | HATEB optimization service |
| `/local_costmap/get_costmap` | `nav2_msgs/srv/GetCostmap` | retrieve local costmap |
| `/local_costmap/clear_around_local_costmap` | `nav2_msgs/srv/ClearCostmapAroundRobot` | standard Nav2 clear |
| `/local_costmap/clear_except_local_costmap` | `nav2_msgs/srv/ClearCostmapExceptRegion` | standard Nav2 clear |
| `/local_costmap/clear_entirely_local_costmap` | `nav2_msgs/srv/ClearEntireCostmap` | standard Nav2 clear |
| `/spawn_entity`, `/delete_entity` | Gazebo entity services | simulation models |
| `/get_model_list` | `gazebo_msgs/srv/GetModelList` | model inventory |
| `/pause_physics`, `/unpause_physics`, `/reset_world`, `/reset_simulation` | `std_srvs/srv/Empty` | Gazebo control |

Lifecycle managers expose `/is_active` (`std_srvs/srv/Trigger`) and `/manage_nodes`
(`nav2_msgs/srv/ManageLifecycleNodes`). Lifecycle nodes expose `/get_state`,
`/change_state`, state/transition discovery services. Every ROS node also exposes the
standard six parameter services: describe, get types, get, list, set and atomic set.
Those standard services are not application control APIs.

## 10. Actions

User-facing actions observed:

| Action | Type | Server | Client |
|---|---|---|---|
| `/follow_path` | `nav2_msgs/action/FollowPath` | DWB/HATEB Controller Server | local task |
| `/reference/follow_path` | `nav2_msgs/action/FollowPath` | reference RPP Controller Server | local task in DWAL conditions |
| `/agent_planner/compute_path_to_pose` | `nav2_msgs/action/ComputePathToPose` | CoHAN prediction stack | agent path prediction |

No `NavigateToPose` server, global planning action or full Nav2 navigation stack is
launched.

## 11. Plugin inventory

Actually loaded plugins, separated from merely installed plugins:

| Host | Loaded class/library | Conditions | Configuration |
|---|---|---|---|
| Gazebo | `libgazebo_ros_init.so`, `libgazebo_ros_factory.so` | all | launch command |
| Gazebo world | HuNav Gazebo plugin | all | generated world |
| iWalk Gazebo model | `libgazebo_ros_planar_move.so` | all | generated café world |
| iWalk lidar | `libgazebo_ros_ray_sensor.so` headless or GPU ray sensor graphical | all | generated café world |
| Controller Server | `dwb_core::DWBLocalPlanner` | DWB | generated `dwb_*.yaml` |
| Controller Server | `hateb_local_planner::HATebLocalPlannerROS` | HATEB | generated `hateb_*.yaml` |
| Reference Controller Server | `nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController` | autonomous DWAL | generated `reference.yaml` |
| Local costmap | `nav2_costmap_2d::ObstacleLayer`, `InflationLayer` | all controller costmaps | generated YAML |
| Local costmap | `cohan_layers::StaticAgentLayer`, `AgentVisibilityLayer` | DWB ON only | generated `dwb_on.yaml` |
| Collision monitor | polygon approach model plus scan source | all | `command_pipeline.yaml` |
| Laser filter chain | `laser_filters/LaserScanBoxFilter` | all | generated `scan_filter.yaml` |

Installed/available but not loaded in a given run include the other two controller
plugins, CoHAN social layers in OFF and non-DWB conditions, RViz unless requested, and
the Gazebo GUI unless requested. HATEB is not substituted by DWB or another controller.

## 12. Pedestrian and context interfaces

`/human_states` is HuNav simulator ground truth. This phase does not claim camera
perception, identity re-identification or semantic intention recognition.

The bridge follows the established `pmb2_hunav_simulation` approach and publishes
CoHAN `TrackedAgents` with stable simulator IDs, pose, twist and moving/stopped state:

- `/tracked_agents_logging` is always published for context/evaluation.
- `/tracked_agents` is created only in ON.
- In OFF, graph inspection showed zero `/tracked_agents` publishers.

DWB's CoHAN layers require an `AgentsInfo` state side channel normally created inside
HATEB. The DWB ON adapter derives only moving/stopped state, sorted distance and robot
pose from current ground-truth tracks and odometry. It is a compatibility/state adapter,
not an intention or future-motion predictor.

### Context output and heuristics

`ContextState` carries an `odom` header, valid bit, confidence, maximum input age, all
eight labels/scores, dominant profile and reason. It publishes at 8 Hz nominal:

- `OPEN_AREA`
- `NARROW_CORRIDOR`
- `DOORWAY`
- `JUNCTION`
- `DENSE_CROWD`
- `OCCLUSION`
- `UNKNOWN`
- `STALE`

Scores overlap. Crowd density counts tracks within 3 m. Four ray clearances estimate
open/narrow/doorway structure; eight rays estimate junction branches and occlusion
risk. Doorway and junction recognition are explicitly heuristic. No tracks means
occlusion score zero; absence of a track is never called a confirmed occlusion.

Ordinary profile changes use a 0.12 score margin and 1.5 s minimum dwell.
`UNKNOWN` and `STALE` bypass dwell. Missing/stale inputs set `valid=false` and
`confidence=0`; confidence is the dominant heuristic score multiplied by measured
freshness, not fabricated classifier confidence.

A runtime track-source failure produced:

- dominant profile `STALE`
- score `STALE=1.0`
- `valid=false`, `confidence=0`
- reason `stale inputs: tracks,costmap`
- semantic radius policy `speed_limit=0`
- final `/cmd_vel` zero.

All nodes use simulation time and clear freshness state after backward clock jumps.

## 13. DWAL, shared control and dynamic-radius policy

### Shared controller integration

Both DWAL families use the pinned `bayesian_shared_control` node. The patch corrects
topic parameters, supports the actual near/far `ClusterGroup` inputs, supports declared
Twist/TwistStamped modes, and adds freshness checking.

For this experiment:

- input/reference and output are `geometry_msgs/msg/Twist`;
- reverse is rejected;
- no-cluster pass-through is disabled;
- no valid cluster produces a controlled zero;
- stale odometry, both clusters, reference, or (ON only) tracks produce a controlled
  zero;
- a 50 ms selector watchdog keeps publishing zero after a producer stops.

ON retains the Bayesian selector and adds a social term to candidate score. For every
candidate path it propagates each tracked torso with constant velocity at the
candidate's time samples, finds closest predicted separation and blends a Gaussian
personal-space cost with the geometric/Bayesian score. `DENSE_CROWD` increases its
margin by 35%; `DOORWAY` and `OCCLUSION` by 20%. This is constant-velocity motion
prediction, not intention recognition.

### Fixed radius

Fixed DWAL uses levels 1.20 and 1.80 m. Runtime observations showed both radius state
and generated outer level remain 1.800 m. The dynamic generator subscription exists,
but fixed mode intentionally does not publish radius updates.

### Dynamic radius

The implementation supports:

1. discrete adaptation by conservatively snapping to the next configured level;
2. continuous outer-radius updates at runtime.

The policy is:

```text
R_stop(v,c)    = r_front + m(c) + v*tau(c) + v^2/(2*a_brake)
R_preview(v,c) = v*T_preview(c)
R_required     = max(R_stop, R_preview)
R_upper        = min(R_sensor_reliable, R_costmap)
R_dynamic      = clamp(R_required, R_min, R_upper)
```

Configured values include `r_front=0.865 m`, `a_brake=0.50 m/s^2`,
`R_min=1.40 m`, sensor bound 8.0 m and costmap bound 4.0 m. `v` is nonnegative forward
speed; reverse is outside this experiment and is stopped. The configured braking value
is an idealized simulation deceleration, not measured physical iWalk capability.

The DWAL radius denotes the trajectory horizon, while footprint collision is handled
separately, so adding the front physical extent does not double-count a DWAL body
radius. Runtime updates regenerate generator parameters and levels. Clustering reads
the levels from every sampled-path message, validates ordering/count and updates both
near/far group objects. Marker allocation uses the maximum configured radius, preventing
stale allocation when the active radius grows.

Increases are immediate. Decreases are rate limited to 0.50 m/s of radius change and
changes below 0.03 m are suppressed. Safety speed reduction is not delayed. If the
required radius exceeds reliable coverage, the policy solves both braking and preview
inequalities for a lower speed; if infeasible it publishes zero. Invalid bounds,
non-finite values or nonpositive braking deceleration also stop.

OFF uses the fixed `OPEN_AREA` margin/reaction/preview values without subscribing
context into the decision. ON uses the active profile's YAML values. A runtime ON sample
showed `JUNCTION`, requested 1.165 m, applied 1.400 m, upper bound 4.0 m and speed limit
0.20 m/s. The generator logged an initial runtime change from 1.80 to 1.45 and then
1.40 m; sampled paths and clustering reported 1.40 m.

## 14. Exact semantic ON/OFF behavior

| Family | OFF | ON |
|---|---|---|
| Fixed DWAL | Lidar/geometric costmap, near/far DWAL clusters, deterministic RPP reference and Bayesian selection. No `/tracked_agents` publisher; selector has no track/context subscriptions. | Same system plus structured tracks and context profile. Constant-velocity predicted human positions change candidate social score. |
| DWB | Standard DWB critics and local obstacle/inflation layers. Pedestrians remain lidar obstacles. | Same DWB critics plus CoHAN StaticAgentLayer and AgentVisibilityLayer. Current structured track pose/orientation and derived moving/stopped state write social costs into the local costmap consumed by BaseObstacle. It does not use future intention prediction or the context profile. |
| HATEB | Actual HATEB plugin with `planning_mode=0`. Human constraint booleans are false and their weights zero; the controller receives no `/tracked_agents` publisher. Robot kinematics, time/shortest-path, viapoint, static obstacle/costmap and nonlinear obstacle terms remain. A predictor service host points to an intentionally empty topic because pinned HATEB invokes reset services even in mode 0. | `planning_mode=1`, tracked-agent subscriber and prediction service active. Agent bands, agent/robot safety, relative velocity and visibility constraints are enabled; observed agent-robot safety weight was 5.0. |
| Dynamic DWAL | Fixed non-semantic braking/horizon values drive runtime radius and speed; no track/context reaches selector or radius decision. | Fixed-DWAL ON social scoring plus context-dependent margin, reaction allowance, preview and max speed in the dynamic-radius policy. |

The mechanisms are deliberately not claimed equivalent. DWB ON is spatial
track/state costmap shaping; HATEB ON jointly optimizes agent trajectories/constraints;
DWAL ON adds time-indexed constant-velocity social scoring, and dynamic DWAL ON also
changes horizon/speed by context.

## 15. Cross-condition consistency

One source, `experiment.yaml`, controls the common limits and task:

- max linear 0.30 m/s, angular 0.80 rad/s;
- linear acceleration/deceleration 0.50 m/s²;
- angular acceleration/deceleration 1.00 rad/s²;
- same `odom`, `sim_base`, `/odom`, `/scan`;
- same local path and goal tolerance;
- same seed parameter and HuNav café scenario;
- same command timeouts and downstream safety chain.

The footprint is generated from the expanded iWalk visual/collision envelope:

```text
[(0.865, 0.327), (0.865, -0.327),
 (-0.103, -0.327), (-0.103, 0.327)]
```

It is used by the Gazebo navigation collision box, standalone DWAL costmap, Nav2
controller local costmaps, DWAL generator and HATEB polygon footprint. Padding is
0.02 m. DWB consumes the same Nav2 polygon. The collision monitor receives that
costmap's published footprint. The 8 x 8 m rolling local costmap gives a 4 m reliable
radius; DWAL never applies a larger horizon.

Every condition retains the same Gazebo planar motion/odometry plugin. No map
localization or alternative motion model is launched.

## 16. Validation performed

### Build and offline checks

- Final Docker image build: passed, including pinned model assets and lifecycle-manager
  shared-library gate.
- Python syntax: passed for launch and project nodes.
- Bash syntax: passed for `run_dwal_cafe.sh`.
- XML package parse and `git diff --check`: passed.
- Policy tests: `6 passed`. They cover nominal horizon, infeasible-horizon speed
  reduction, invalid braking stop, stale dwell bypass, ordinary hysteresis and
  conservative discrete snapping.
- Scene regression: passed: physical transforms preserved, unique robot TF root,
  shared conservative footprint, odometry/costmap frames, full costmap updates, café
  assets and sensor/odometry plugins.

### Runtime safety/graph checks

Verified in running simulations:

- lidar, odometry, local costmap and ground-truth tracks;
- controller/reference/costmap/smoother/collision lifecycle activation;
- finite values at all five command stages;
- robot motion;
- one final publisher;
- OFF has no controller track publisher;
- ON has the expected controller subscriber;
- fixed radius remains 1.8 m;
- continuous radius reaches generator and clustering;
- stale tracks immediately produce STALE/invalid context, zero semantic speed limit and
  zero final command;
- after intentionally terminating collision monitor, a temporary 0.1 m/s
  `/cmd_vel_safe` input appeared at `/cmd_vel`, and after that publisher stopped the
  0.30 s final watchdog returned `/cmd_vel` to zero;
- HATEB local-frame patch removed hard-coded `map` TF use;
- the HATEB duplicate-pose timing failure was reproduced, fixed by maintaining the
  upstream 0.1 s minimum time difference, rebuilt, and the ON run subsequently reached
  the goal without assertion.

### Eight-condition seed-1 smoke results

These are smoke results, not comparative evidence. `action_status=4` is succeeded and
`6` is aborted. The HuNav `completed` field is its goal-completion metric.

| Condition | Live evidence | Task / evaluator result |
|---|---|---|
| DWB OFF | full live command/safety/isolation gate passed | action succeeded; HuNav completed |
| DWB ON | semantic publisher, adapter, social-layer subscriptions, robot motion and safe chain verified; observer attached after command ended missed nonzero sample | aborted after DWB reported all 619 trajectories blocked by social/obstacle cost; HuNav incomplete |
| Fixed DWAL OFF | radius/generator/clusters/isolation/motion verified; late observer missed nonzero sample | progress-checker abort; HuNav incomplete |
| Fixed DWAL ON | full live gate passed | progress-checker abort; HuNav incomplete |
| Dynamic DWAL OFF | full live gate passed | progress-checker abort; HuNav incomplete |
| Dynamic DWAL ON | full live gate passed | progress-checker abort; HuNav incomplete |
| HATEB OFF | command path passed in an earlier run; final-image rerun reached goal and late observer missed nonzero sample | action succeeded; HuNav completed |
| HATEB ON | full live gate passed after timing fix | action succeeded; HuNav completed |

Evaluator CSV summary:

| Condition | completed | time (s) | path length (m) | min person distance (m) | robot-on-person contact samples | stall time | avg acceleration |
|---|---:|---:|---:|---:|---:|---:|---:|
| DWB OFF | true | 12.30 | 3.015 | 0.000 | 28 | 0.694 | 0.116 |
| DWB ON | false | 7.30 | 1.499 | 0.134 | 0 | 0.691 | 0.089 |
| Fixed DWAL OFF | false | 16.25 | 1.968 | 0.048 | 0 | 0.698 | 0.093 |
| Fixed DWAL ON | false | 13.90 | 1.176 | 0.452 | 0 | 0.695 | 0.068 |
| Dynamic DWAL OFF | false | 16.80 | 2.012 | 0.512 | 0 | 8.748 | 0.106 |
| Dynamic DWAL ON | false | 14.30 | 1.158 | 0.622 | 0 | 8.640 | 0.086 |
| HATEB OFF | true | 10.90 | 2.994 | 0.000 | 16 | 0.694 | 0.030 |
| HATEB ON | true | 15.90 | 3.038 | 0.599 | 0 | 0.894 | 0.065 |

"Contact samples" is the evaluator's sampled count, not a binary trial count. The CSVs
also contain person-on-robot contacts, average/maximum distances, personal/social-space
intrusions, heading changes, speed, over-acceleration, pedestrian speeds, social work
and force metrics. Per-step CSVs are in `rosbags/`. Command-stage bags enable independent
smoothness and intervention analysis.

No statistical comparison or controller-superiority conclusion is made from one seed.

## 17. Runtime inventory differences

The graph was inspected for every family/mode during the eight launches. The
condition-dependent deltas are:

| Configuration | Controller action | Track graph | Extra topics/plugins |
|---|---|---|---|
| fixed DWAL OFF | `/reference/follow_path` | logging only | DWAL samples/clusters; fixed radius |
| fixed DWAL ON | `/reference/follow_path` | bridge -> shared controller | context profile also reaches selector |
| dynamic DWAL OFF | `/reference/follow_path` | logging only | runtime radius to generator; nonsemantic policy |
| dynamic DWAL ON | `/reference/follow_path` | bridge -> shared controller | runtime radius plus context-aware policy |
| DWB OFF | `/follow_path` | logging only | obstacle + inflation layers |
| DWB ON | `/follow_path` | bridge -> local costmap and DWB adapter | `/agents_info`, two CoHAN social layers |
| HATEB OFF | `/follow_path` | no controller track publisher | predictor service host, human constraints disabled |
| HATEB ON | `/follow_path` | bridge -> HATEB and predictor | agent plans/trajectories and HATEB human constraints |

This table is the distinct-configuration inventory; Sections 6–10 give the union graph
and exact interface types. Reproduce any inventory with:

```bash
docker exec iwalk_dwal /bin/bash /dwal_entrypoint.sh ros2 node list
docker exec iwalk_dwal /bin/bash /dwal_entrypoint.sh ros2 topic list -t
docker exec iwalk_dwal /bin/bash /dwal_entrypoint.sh ros2 service list -t
docker exec iwalk_dwal /bin/bash /dwal_entrypoint.sh ros2 action list -t
docker exec iwalk_dwal /bin/bash /dwal_entrypoint.sh ros2 topic info /cmd_vel -v
docker exec iwalk_dwal /bin/bash /dwal_entrypoint.sh ros2 topic info /tracked_agents -v
```

## 18. Remaining gaps and concrete blockers

- The single smoke seed is validation, not quantitative evaluation. Run the paired
  seed loop and retain bags before statistical claims.
- DWB ON is functionally semantic but too conservative in this seed: social costs made
  all sampled trajectories invalid after 1.50 m. That is a real decision difference,
  not a successful navigation result. Tune CoHAN amplitude/radius using repeated paired
  trials without removing the structured-track mechanism.
- All four DWAL systems moved and exercised the complete command chain, but the
  reference Controller Server aborted for insufficient progress in this scenario.
  The evaluated system includes RPP plus Bayesian selection; tuning the nominal/shared
  interaction remains required for task completion.
- The discrete radius branch was unit-checked but not given a separate runtime smoke;
  the requested final dynamic conditions used continuous adaptation.
- Exact internal per-cycle controller CPU computation time is not published by the
  apt-installed Humble Controller Server/DWB plugin, and HATEB's `/plan_time` and
  `/traj_time` are time-to-goal messages, not CPU timings. The delivered bags record
  command cadence and HATEB feedback, but this one requested metric remains a concrete
  instrumentation gap. Use `ros2_tracing` around `computeVelocityCommands` or rebuild
  the Controller Server with timing instrumentation before reporting computation-time
  comparisons; do not relabel command period as computation time.
- The laser filter data path is verified, but the duplicated upstream
  `scan_self_filter` node name prevented a conclusive lifecycle state response.
- Context geometry, doorway, junction and occlusion outputs are transparent baseline
  heuristics. They are not learned semantic recognition. Constant-velocity propagation
  is motion prediction, not intention recognition.
- The 0.50 m/s² braking value is configured/validated simulation behavior only.
- Headless HATEB occasionally missed its desired 20 Hz control loop on this host; this
  should be treated as a compute-platform limitation in timing-sensitive comparisons.

## Appendix A: hidden action transport endpoints

These are ROS action implementation endpoints, not additional user-facing actions.

For `/follow_path` (or `/reference/follow_path` in DWAL):

- topic `.../_action/feedback`:
  `nav2_msgs/action/FollowPath_FeedbackMessage`
- topic `.../_action/status`: `action_msgs/msg/GoalStatusArray`
- service `.../_action/send_goal`:
  `nav2_msgs/action/FollowPath_SendGoal`
- service `.../_action/get_result`:
  `nav2_msgs/action/FollowPath_GetResult`
- service `.../_action/cancel_goal`: `action_msgs/srv/CancelGoal`

For `/agent_planner/compute_path_to_pose`:

- topic `.../_action/feedback`:
  `nav2_msgs/action/ComputePathToPose_FeedbackMessage`
- topic `.../_action/status`: `action_msgs/msg/GoalStatusArray`
- service `.../_action/send_goal`:
  `nav2_msgs/action/ComputePathToPose_SendGoal`
- service `.../_action/get_result`:
  `nav2_msgs/action/ComputePathToPose_GetResult`
- service `.../_action/cancel_goal`: `action_msgs/srv/CancelGoal`

An orphan `/navigate_to_pose/_action/status` transport topic was visible from the
HuNav/CoHAN environment, but no `/navigate_to_pose` action server appeared in
`ros2 action list` and it is not used by this implementation.

## Final summary

All eight selectable conditions, the common local path, HuNav ground-truth tracking,
continuous context estimator, fixed/discrete/continuous radius policy, existing
Bayesian shared controller, common Nav2 smoothing/collision pipeline, evaluator
orchestration and single final command authority are implemented. The final pinned
Humble/Gazebo Classic image builds; unit/offline gates pass; every condition generated
commands and robot motion; semantic inputs were isolated in OFF and consumed through
documented, controller-specific mechanisms in ON.

The main remaining experimental issues are DWB-ON conservatism, DWAL progress failures,
lack of repeated-seed statistics, and exact internal controller CPU timing
instrumentation. These are reported as gaps rather than presented as completed results.

## Addendum: optional DWB/HATEB teleop-as-reference mode

The statement in Section 2 that interactive teleoperation is separate from quantitative
trials still applies. It is now extended with an optional interactive reference mode for
DWB and HATEB. The default remains `REFERENCE_MODE=autonomous`, the repeated-experiment
loop remains automatic, and the deterministic `local_task` path and all eight automatic
condition configurations are unchanged.

Launch one of the four new interactive combinations with exactly:

```bash
REFERENCE_MODE=teleop bash scripts/run_dwal_cafe.sh run dwb off 1
REFERENCE_MODE=teleop bash scripts/run_dwal_cafe.sh run dwb on 1
REFERENCE_MODE=teleop bash scripts/run_dwal_cafe.sh run hateb off 1
REFERENCE_MODE=teleop bash scripts/run_dwal_cafe.sh run hateb on 1
```

Then, in a second terminal:

```bash
bash scripts/run_dwal_cafe.sh teleop
```

The keyboard still publishes `geometry_msgs/msg/Twist` only on `/reference_cmd`.
For these DWB/HATEB teleop runs, `teleop_path_adapter` reads `linear.x` and
`angular.z`, starts at the current `odom -> sim_base` odometry pose, and integrates a
short differential-drive arc into a sampled `nav_msgs/msg/Path` in `odom`. It sends
that path to the existing `/follow_path` Controller Server. Lateral input is not used,
and reverse `linear.x` is rejected because this experiment has `allow_reverse=false`.

The runtime flow is:

```text
keyboard teleop
  -> /reference_cmd (Twist intention)
  -> teleop_path_adapter
  -> /experiment/path (short local Path for observation)
  -> /follow_path (serialized Nav2 FollowPath goal)
  -> controller_server [the selected DWB or HATEB plugin]
  -> /cmd_vel_selected
  -> command_guard
  -> /cmd_vel_guarded
  -> nav2_velocity_smoother
  -> /cmd_vel_smoothed
  -> nav2_collision_monitor
  -> /cmd_vel_safe
  -> final_cmd_watchdog
  -> /cmd_vel
  -> Gazebo ideal_planar_base
```

The adapter runs its input/state check at 10 Hz but does not replace an action goal on
every check. It replaces on a material command change or after the configured 0.75 s
local-reference refresh period. Replacement is serialized: the current goal is
canceled and its result is received before the next goal is sent. This prevents
overlapping active `FollowPath` goals; live action status contained historical canceled
goals and exactly one executing goal.

The default teleop-input timeout is 0.35 s and is configurable through, for example,
`TELEOP_INPUT_TIMEOUT=0.5 REFERENCE_MODE=teleop ...`. A zero, negative-reverse,
non-finite, missing, or stale keyboard intention cancels the goal. Missing/stale or
invalid odometry, an inactive/stale Controller Server lifecycle state, unavailable
action server, rejected goal, or stale action feedback does the same. The adapter also
publishes a fail-closed Boolean heartbeat on
`/experiment/teleop_reference_active`. Only in DWB/HATEB teleop mode, `command_guard`
requires this fresh true heartbeat; a false/stale heartbeat forces zero even if an old
controller goal failed to cancel. The gate is disabled by default and is not enabled in
automatic runs or fixed/dynamic-DWAL teleop runs.

No keyboard velocity is published to `/cmd_vel_selected`, `/cmd_vel_guarded`,
`/cmd_vel_smoothed`, `/cmd_vel_safe`, or `/cmd_vel`. DWB/HATEB remains the only
publisher of `/cmd_vel_selected`; it still applies its OFF/ON costmap or human-aware
configuration, obstacle and human constraints, and kinematic limits. The complete
existing downstream guard, smoother, collision monitor, and watchdog chain remains in
place, and `final_cmd_watchdog` remains the only `/cmd_vel` publisher.

### Files added or minimally modified

- Added `src/crowd_aware_simulation/scripts/teleop_path_adapter.py`: Twist freshness,
  odometry/lifecycle/action-feedback validation, differential-drive Path projection,
  serialized `FollowPath` ownership, and the fail-closed gate heartbeat.
- Added `src/crowd_aware_simulation/test/test_teleop_path_adapter.py`: straight, curved,
  and in-place-rotation projection regression tests.
- Modified `src/crowd_aware_simulation/launch/dwal_cafe.launch.py`: select the adapter
  and guard gate only for explicit DWB/HATEB teleop runs; declare the timeout argument.
- Modified `src/crowd_aware_simulation/scripts/command_guard.py`: add the optional,
  default-disabled teleop-reference heartbeat gate.
- Modified `src/crowd_aware_simulation/CMakeLists.txt`: install the adapter executable.
- Modified `scripts/run_dwal_cafe.sh`: pass the configurable
  `TELEOP_INPUT_TIMEOUT` value (default 0.35 s) to launch.
- Appended this addendum to `report.md`. Controller parameters, metrics, semantic and
  dynamic-radius logic, HuNav configuration, model/world, seeds, dependency revisions,
  and experiment paths were not changed.

### Validation performed on 2026-09-22

- `bash scripts/run_dwal_cafe.sh build`: passed. The image rebuilt
  `iwalk_description`, `crowd_aware_interfaces`, `dwal_planner`,
  `bayesian_shared_control`, and `crowd_aware_simulation`; only the already present
  DWAL deprecation/compiler warnings were emitted.
- `python3 -m pytest -q` inside the Humble image for `test_policy_core.py` and
  `test_teleop_path_adapter.py`: `9 passed`.
- Existing `check_scene.py`: passed the physical-transform, TF-root, footprint,
  odometry/costmap-frame, full-map, café, and sensor/plugin checks.
- Python compilation, shell syntax, and `git diff --check`: passed.
- Default automatic DWB OFF and HATEB OFF were launched without `REFERENCE_MODE`.
  Both launched `local_task`, sent the unchanged deterministic path to their existing
  Controller Server, moved the robot, reached the goal, and stored evaluator output.
- The existing DWB OFF live smoke checker passed every reported item except
  `nonzero_selected_command`. It was started after the short command-producing part of
  the task and therefore exited 1, while its latched result showed
  `FINISHED action_status=4`, `robot_motion: PASS`, `controller_active: PASS`, all
  command-stage topics `PASS`, and `single_final_publisher: PASS`. This is the late-
  observer limitation already described in Section 2, not reported as a full smoke
  pass.
- DWB OFF, DWB ON, HATEB OFF, and HATEB ON were each launched with
  `REFERENCE_MODE=teleop`. A repeatable 10 Hz `Twist` publisher was used on
  `/reference_cmd` in place of manual keystrokes for runtime measurement. In every
  case the adapter produced live paths, the selected controller accepted serialized
  `/follow_path` goals, and nonzero controller output was observed. The actual
  interactive `teleop` subcommand was inspected but was not manually driven in this
  automated terminal session.
- Graph inspection showed one `/cmd_vel_selected` publisher named
  `controller_server`, consumed by `command_guard`, and one `/cmd_vel` publisher named
  `final_cmd_watchdog`, consumed by `ideal_planar_base`. `/reference_cmd` connected to
  `teleop_path_adapter`, not to any final command topic.
- DWB ON retained the two CoHAN social-layer subscriptions on `/tracked_agents` plus
  `cohan_agents_info_adapter`. HATEB OFF retained no `/tracked_agents` publisher while
  its configured predictor service host remained present. HATEB ON retained the HuNav
  bridge publisher and the HATEB/predictor subscriptions.
- Removing the `/reference_cmd` publisher was checked live in DWB OFF and HATEB OFF.
  After the timeout, `/experiment/teleop_reference_active` was false,
  `/experiment/command_guard_status` was `STOP inactive teleop reference`, and every
  field of the final `/cmd_vel` sample was zero.
- Action status was inspected while driving: previous goals were terminal canceled
  (with one observed historic abort in HATEB), and exactly one goal had executing
  status. No competing selected-command or final-command publisher appeared.
- The new adapter itself exited cleanly on Ctrl-C after explicit interrupt handling was
  added and rebuilt.

### Limitations

- A `Path` has geometry but no timing, so the projected arc encodes the requested
  direction, curvature, and horizon distance; DWB/HATEB still chooses the actual
  instantaneous velocity. This is intentional so keyboard input remains a reference
  rather than a controller bypass.
- Goal serialization creates a brief controller stop during a reference replacement.
  The 0.75 s refresh avoids high-rate action flooding and can be tuned as an adapter
  parameter if a different controller/host needs a different responsiveness tradeoff.
- The pinned HATEB Controller Server continued to exit with signal 11 during global
  Ctrl-C teardown in both automatic and teleop launches, after completing normal goal
  execution; several pre-existing Python nodes also report double-shutdown exceptions.
  These teardown behaviors predate and are outside this feature. The new adapter
  terminates cleanly, Docker removes the run container, and no runtime process remains.
- No quantitative teleop trials were performed. Teleop is an optional interactive
  demonstration/input mode and must not be mixed into the repeated automatic dataset
  unless explicitly selected and documented.

## Evaluation Scenario Suite

The evaluation suite keeps one launch flow and one scene-preparation path. Common
controller limits, profiles, and timeouts remain in `experiment.yaml`; each scenario
has a small configuration selecting its robot task, shared world, and HuNav agents.

- `open_area.world`: `open_area`, `crossing`, `dense_crowd`, `group_blocking`, and
  `target_confusion`
- `narrow_corridor.world`: `narrow_corridor`
- `doorway.world`: `doorway`
- `junction.world`: `junction`
- `occlusion.world`: `occlusion`
- `base_cafe.world`: backward-compatible `cafe` integration/stress test

Run with `SCENARIO=<name> HEADLESS=true bash scripts/run_dwal_cafe.sh run
<controller> <on|off> <seed>`; for example, `SCENARIO=crossing HEADLESS=true bash
scripts/run_dwal_cafe.sh run hateb on 1`.

These scenarios exercise local-planner behavior and crowd semantics, not
front-following. `target_confusion` provides validated deterministic assets, but full
evaluation intentionally remains unavailable until an intended-user tracker and
front-following interface are integrated.
