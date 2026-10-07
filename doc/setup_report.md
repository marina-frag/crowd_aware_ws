# Current local DWAL setup

Validated: 2026-10-05. The authoritative dynamic-DWAL design, equations,
runtime evidence, and file/function mapping are in
[`clearer_dwal_report1.md`](../clearer_dwal_report1.md).

## Supported DWAL conditions

- `fixed_dwal`: pinned upstream `dwal_generator`,
  `dwal_clustering`, and Bayesian `shared_controller`, using the fixed
  1.20 m and 1.80 m levels.
- `dynamic_dwal`: the project `dynamic_dwal_node`, which owns per-curvature
  arc-line generation, variable free length, admissibility, contiguous
  clustering, teleop-intent selection, and final braking validation.

There is no scalar-radius dynamic policy. Dynamic mode has neither a common
radius nor common near/far levels.

Both modes send `/cmd_vel_selected` through:

```text
command_guard -> velocity_smoother -> collision_monitor
 -> final_cmd_watchdog -> /cmd_vel
```

Only `final_cmd_watchdog` publishes `/cmd_vel`.

## Topics

| Topic | Role |
|---|---|
| `/reference_cmd` | teleop/reference intent |
| `/odom` | measured robot motion |
| `/scan` | current observed clearance |
| `/local_costmap/costmap` | full-footprint lethal collision checks |
| `/tracked_agents` | ON-only structured human input |
| `/dynamic_dwal/diagnostics` | complete per-cycle path/cluster/controller record |
| `/dynamic_dwal/markers` | all candidates, cluster colors, and selected path |
| `/cmd_vel_selected` | assisted controller output |
| `/cmd_vel` | final watchdog output to Gazebo |

OFF does not subscribe to structured tracks; people enter the current
scan/costmap as physical obstacles. ON requires fresh tracks, explicitly selects
the HUMAN TORSO segment, and rechecks prediction plus feasible braking for the
final selected motion.

## Commands

```bash
bash scripts/run_dwal_cafe.sh build

SCENARIO=dense_crowd REFERENCE_MODE=teleop \
  HEADLESS=true GUI=false RVIZ=true \
  bash scripts/run_dwal_cafe.sh run dynamic_dwal off 1

# Other terminals:
bash scripts/run_dwal_cafe.sh teleop
bash scripts/run_dwal_cafe.sh check dynamic_dwal off 1
```

Repeat the launch/check with `on` after OFF passes.

For the fixed baseline:

```bash
SCENARIO=dense_crowd bash scripts/run_dwal_cafe.sh run fixed_dwal off 1
bash scripts/run_dwal_cafe.sh check fixed_dwal off 1
```

The dynamic runtime acceptance check is intentionally restricted to
`dense_crowd`. It drives straight, curved, zero, rotation, and stale-reference
phases and validates the live ROS graph, markers, real cluster membership,
safety pipeline, final publisher, and odometry motion.
