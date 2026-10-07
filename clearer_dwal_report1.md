# Dynamic DWAL: final architecture, mathematics, and validation

Validation date: 2026-10-07. Runtime scope: `open_area` and `dense_crowd`,
with dynamic semantics OFF and ON where stated below.

## 1. Result

There is one active dynamic implementation: the C++ `dynamic_dwal_node`. It generates per-curvature arc-line paths, computes an independent free length for every path, forms clusters from the actual contiguous admissible samples, selects a real cluster member from teleoperation intent, and publishes the assisted command to the common safety pipeline.

The fixed condition remains the pinned upstream baseline:

- `dwal_planner` at `95b1aa5e1f0259ae8a467c731bf994fc0ebc4f6d`;
- `bayesian_shared_control` at `9d0df25f8a966db449ec94c073349d09e328d517`;
- upstream `dwal_generator`, `dwal_clustering`, and `shared_controller` executables;
- unchanged fixed levels `[1.20, 1.80]` m.

The former scalar-radius implementation is deleted. The removed files are:

- `patches/dwal_dynamic_radius.patch`;
- `src/crowd_aware_simulation/scripts/dynamic_radius_policy.py`;
- `src/crowd_aware_interfaces/msg/DynamicRadiusState.msg`.

Their launch entries, parameters, install rules, message generation, topic recording, and radius-only policy helpers were also removed. Context-dependent speed limiting still needed by non-DWAL conditions was separated into `context_speed_limit.py`; it does not set a radius or path horizon.

## 2. Executables and topic graph

### fixed_dwal

```text
/reference_cmd + /odom
        |
        v
pinned dwal_generator --> /dwal_planner/sampled_paths
        |
pinned dwal_clustering --> /dwal_planner/clusters_near
                         + /dwal_planner/clusters_far
        |
patched pinned shared_controller
        |
        v
/cmd_vel_selected
```

### dynamic_dwal

```text
/reference_cmd + /odom + /scan + /local_costmap/costmap
                         [+ /tracked_agents in ON only]
        |
        v
dynamic_dwal_node
  - per-path generator
  - admissibility and clustering
  - teleop shared selector
  - final braking recheck
  - diagnostics and RViz markers
        |
        v
/cmd_vel_selected
```

Both conditions then use exactly the same command chain:

```text
/cmd_vel_selected
 -> command_guard -> /cmd_vel_guarded
 -> nav2_velocity_smoother -> /cmd_vel_smoothed
 -> nav2_collision_monitor -> /cmd_vel_safe
 -> final_cmd_watchdog -> /cmd_vel
 -> Gazebo ideal_planar_base
```

`final_cmd_watchdog` is the only publisher of `/cmd_vel`. Teleop never bypasses the selector or this safety chain.

The launch mapping is in `src/crowd_aware_simulation/launch/dwal_cafe.launch.py`; generated parameters are in `scripts/prepare_scene.py`; common limits are in `config/experiment.yaml`.

## 3. Proven cause of the premature terminations

The failure was in scan certification, not in clustering and not merely in
RViz. The pre-fix live cycle reproduced the recorded values exactly:
`test_linear=0.15 m/s`, `required_stop_length=0.1325 m`, 16 samples over
approximately `[-0.667,+0.667]`, and real clusters
`[[2], [5,6,7,8], [10], [12]]`. Several candidates stopped at `0.03–0.10 m`.

The runtime geometry was verified rather than assumed:

- odometry is `odom -> sim_base`; `sim_base` is the rear-axle projection used
  by the footprint and planar-base plugin;
- `sim_base -> laser_frame` is translation `(0.885, 0, 0.35) m` and zero RPY;
- the URDF-derived navigation footprint is
  `x=[-0.103,0.865], y=[-0.327,0.327] m`, padded once by `0.02 m` in this node;
- rear wheel radius/separation are `0.095/0.61 m` in both the URDF and the
  kinematic filter; simulation uses `ideal_planar_base`, so ERPM/hardware
  parameters were not changed;
- the rolling costmap is `160 x 160`, resolution `0.05 m`, in `odom`. The
  open-area probe contained zero unknown cells. Its origin and yaw are applied.

The filtered scan had 346 NaNs while `/scan_raw` had zero NaNs. The NaN bands
were rays 0–172 and 547–719, exactly the body returns removed by the configured
box filter. For `k=-0.6666667`, the first false failure was:

```text
s = 0.03672574996 m
path pose = (0.03672208, -0.00044957, -0.02448383)
footprint point = (0.86380580, -0.36780783) in sim_base
query ray = 173, angle = -1.62835571 rad
neighbor ray = 172, filtered = NaN, raw = 0.29553530 m
```

Ray 172 intersects the physical front face at `x=0.865 m`; it is a proven
self-filtered robot return. The bug was that the query ray's self-occlusion
boolean was reused for all three neighboring rays. A tangent query ray was
therefore declared unknown because only its neighbor was self-occluded.

A second case appeared after fixing that: the box filter invalidates the whole
ray when its eventual return is on the body, including known-clear points before
that return. Self-filter proof must intersect the exact invalid ray against the
robot over the sensor range, not only over the shorter segment to the query.
Finally, the `-pi/+pi` neighbors of this full 360-degree scan must wrap; treating
index `-1` or `720` as outside-FOV caused another false stop.

All three faults are fixed. Every neighboring ray is classified separately;
only geometrically proven body-filter rays are delegated to the already-passed
costmap check; true outside-FOV, invalid external ray, and insufficient measured
clearance still fail closed. Costmap boundary, unknown cell, lethal collision,
outside-FOV, invalid ray, and insufficient clearance now have distinct reasons
and record their first failure pose/point, arc length, cell, ray index, angle,
measured range, and required range. No footprint or braking margin was reduced.

## 4. Sampling and standstill behavior

The pure functions are in `include/crowd_aware_simulation/dynamic_dwal_core.hpp`; their caller is `dynamic_dwal_node.cpp::tick`.

Let all linear speeds be m/s, angular speeds rad/s, acceleration limits m/s2 or rad/s2, curvature `k` in m^-1, length in m, and time in s.

For measured forward speed `v_m`, requested teleop speed `v_ref`, window duration `Delta t`, acceleration `a+`, and braking deceleration `a-`:

```text
v_min = max(0, v_m - a- Delta t)
v_max = min(v_limit, v_m + a+ Delta t)
v_cmd = clamp(v_ref, v_min, v_max)
v_test = max(v_m, v_cmd)
```

This is `computeTestSpeed`. The maximum prevents a lower request from hiding kinetic energy already present in odometry.

For measured `omega_m`:

```text
omega_min = max(-omega_limit, omega_m - alpha Delta t)
omega_max = min(+omega_limit, omega_m + alpha Delta t)
k_min = max(-k_limit, omega_min / v_test)
k_max = min(+k_limit, omega_max / v_test)
```

This is `computeCurvatureWindow`. `sampleSymmetricCurvatures` always publishes
the complete configured family `[-max_curvature,+max_curvature]`, paired exactly
around zero and including zero and both endpoints. Dynamic-window membership,
wheel speed and braking are tested independently for every member. Ineligible
members stay visible but never enter clusters or command selection. Teleop
angular intent only ranks eligible real members.

There is no division when `v_test <= zero_linear_epsilon`. A nonzero translational request at standstill creates `v_cmd > 0` through the reachable linear window and then samples a full curvature interval. A pure angular request takes the explicit in-place-rotation branch and is checked with angular reachability, wheel limits, costmap/scan footprint motion, ON prediction, and angular braking. A zero request commands a controlled zero after checking the braking motion from the measured state.

## 5. Arc-line geometry and variable length

For initial curvature `k != 0`, the path follows a circular arc until the heading reaches `sign(k) pi/2`:

```text
s_a = pi / (2 |k|)
theta(s) = k min(s, s_a)
x(s) = sin(theta) / k
y(s) = (1 - cos(theta)) / k
```

For `s > s_a`, a tangent line continues from the arc endpoint:

```text
x(s) = x(s_a) + (s-s_a) cos(theta(s_a))
y(s) = y(s_a) + (s-s_a) sin(theta(s_a))
```

For `k=0`, `x=s, y=0, theta=0`. These are `arcLinePose`, `localCurvature`, and `arcLineTransition`, matching the pinned upstream geometry.

`tracePath` advances with a sweep step bounded by translation plus footprint
rotation. At every step it checks the complete padded footprint against costmap
boundaries, unknown cells, lethal cells, and scan-known space. Eligible paths
use `sweep_resolution=0.02 m`. Already-ineligible visualization members may use
the separately configured `ineligible_sweep_resolution=0.05 m`; its larger
motion bound expands costmap and laser-clearance checks, so it is conservative
rather than a collision-check relaxation.

The integration step is forced to land exactly on `s_transition`. Stored output
contains regular samples, the exact transition when certified, and the last
certified-free point. The first failing pose is diagnostic only and is never
stored as safe. `free_length` is the last certified length, while
`first_failure_length` is the next failing check. Static braking continues to
use strict `required_stop_length < free_length`.

Each sample retains its own:

```text
L_i(t) = min(L_max, L_known,i(t), d_i(t))
```

where:

- `L_max` is `max_search_length`, the configured search cap in metres;
- `L_known,i` is the first arc length where the swept footprint is no longer scan-known/certified, in metres;
- `d_i` is the first arc length where the padded footprint intersects a lethal costmap cell, in metres.

There are no shared near/far lengths and no shared dynamic radius in dynamic mode.

## 6. Admissibility and clusters

The static stopping requirement is:

```text
D_stop(v_test) =
  v_test T_reaction + v_test^2 / (2 a-) + D_margin
```

A sample is statically admissible only when:

- angular speed is inside the angular dynamic window;
- differential-drive wheel speeds are within limits;
- `|k| a- <= alpha-`, so simultaneous angular braking is feasible;
- the required stop is before the upstream arc-to-line curvature discontinuity;
- `D_stop < L_i` (strict inequality).

In ON, `predictionAdmissible` additionally evaluates robot and human states at the same future time through reaction and feasible braking. Human position uses torso pose and velocity, not `segments.front()`. Missing torso data is fail-closed. The human disk includes base uncertainty, time-growing uncertainty, and half a prediction-step relative-motion margin.

`clusterContiguous` scans the sorted curvature samples. Every maximal run of adjacent admissible indices becomes one cluster, and every rejected sample breaks a run. Cluster members are the actual path indices and can have different `L_i`. No representative replaces their visualization or their use by the controller.

## 7. Shared-control selection and final command

For each admissible real member, the controller forms:

```text
v = v_cmd
omega_i = k_i v
J_i = |omega_i - omega_ref| / omega_limit + 1e-5 |k_i|
```

It first rejects members whose final `omega_i` is outside the reachable angular window or violates wheel-speed limits. It ranks the remaining members by `J_i`, so teleop expresses intent only during selection; it does not collapse sampling.

Before publication, `checkFinalMotion` simulates the measured braking state and then the selected command through reaction plus independent feasible linear/angular deceleration. It rechecks the footprint against scan/costmap and, in ON, against time-consistent predicted humans.

When `v_cmd != v_test`, all speeds spanning the interval are rechecked at a maximum spacing of `final_speed_resolution=0.025 m/s`. This is required because a lower speed is not automatically safer relative to a moving person. Only a passing candidate is published. If there is no admissible cluster member, no safe final motion, stale input, or invalid data, the node publishes zero to `/cmd_vel_selected`; it never forwards raw teleop.

## 8. OFF and ON semantics

OFF:

- the node deliberately creates no `/tracked_agents` subscription;
- people are seen through the current scan and current obstacle costmap, like other moving physical obstacles;
- their current observed position is reconsidered every cycle;
- no future structured-track prediction is used.

ON:

- fresh `/tracked_agents` in `odom` is mandatory;
- only HUMAN agents and their TORSO segment are used;
- constant-velocity prediction, uncertainty growth, reaction, and feasible braking use the same future timestamps;
- both path admissibility and the final command/braking trajectory are checked;
- missing/stale tracks, wrong frame, or missing torso data commands zero.

## 9. Diagnostics and RViz

`DynamicDwalState.msg` records every cycle:

- sampled count and curvature range;
- requested, measured, reachable/test speed and dynamic windows;
- every path's curvature, poses, arc lengths, last certified free length,
  first failure length/pose/point/cell/ray, and separate termination/rejection reason;
- static/prediction/final admissibility;
- every cluster's real path indices;
- selected path/cluster, command, in-place flag, and controller reason;
- age/freshness for odometry, costmap, scan, reference, and tracks.

`publishMarkers` emits all sampled paths. Admissible members use a stable color
per real cluster; prediction/static/kinematic rejections are distinct, and
out-of-window members are dim gray. Candidate/selected widths default to
`0.005/0.010 m`. Certified transitions and first failure points have their own
markers. Optional labels contain index, curvature, certified length, cluster,
and rejection reason.

Zero/rotation cycles show checked braking swept footprints and the predicted
stop footprint; in-place rotation never invents a translational arc. A previous
fan can only persist as dim `stale_*` markers with a bounded `0.75 s` lifetime.
The supplied RViz config displays `/dynamic_dwal/markers`; its three fixed-DWAL
legacy displays are renamed and disabled by default so “Near/Far clusters” are
not mistaken for dynamic clusters.

## 10. Build, run, teleop, RViz, and checks

From the repository root:

```bash
bash scripts/run_dwal_cafe.sh build
```

OFF first, with RViz and a headless Gazebo server:

```bash
SCENARIO=dense_crowd REFERENCE_MODE=teleop \
  HEADLESS=true GUI=false RVIZ=true \
  bash scripts/run_dwal_cafe.sh run dynamic_dwal off 1
```

In a second terminal:

```bash
bash scripts/run_dwal_cafe.sh teleop
```

In a third terminal while teleop is active:

```bash
SCENARIO=dense_crowd bash scripts/run_dwal_cafe.sh check dynamic_dwal off 1
```

Repeat the OFF run/check with `SCENARIO=open_area`. The check command now passes
the selected scenario instead of being artificially restricted to dense crowd.

Focused live diagnostics:

```bash
docker exec iwalk_dwal /bin/bash /dwal_entrypoint.sh \
  ros2 topic echo /dynamic_dwal/diagnostics
docker exec iwalk_dwal /bin/bash /dwal_entrypoint.sh \
  ros2 topic echo /dynamic_dwal/markers
docker exec iwalk_dwal /bin/bash /dwal_entrypoint.sh \
  ros2 run tf2_ros tf2_echo sim_base laser_frame
```

Then repeat with prediction:

```bash
SCENARIO=dense_crowd REFERENCE_MODE=teleop \
  HEADLESS=true GUI=false RVIZ=true \
  bash scripts/run_dwal_cafe.sh run dynamic_dwal on 1
bash scripts/run_dwal_cafe.sh teleop
bash scripts/run_dwal_cafe.sh check dynamic_dwal on 1
```

Fixed-baseline non-regression:

```bash
SCENARIO=dense_crowd HEADLESS=true \
  bash scripts/run_dwal_cafe.sh run fixed_dwal off 1
bash scripts/run_dwal_cafe.sh check fixed_dwal off 1
```

Core tests inside the built image:

```bash
docker run --rm iwalk_dwal:latest /bin/bash -lc \
  'cd /opt/dwal_ws && colcon test --packages-select crowd_aware_simulation \
   --event-handlers console_direct+ && colcon test-result --verbose'
```

## 11. Validation evidence

Evidence collected from the rebuilt image on 2026-10-07:

- Core geometry/sampling/clustering/braking: 12 GTests passed; the colcon result
  reported 13 tests, zero errors/failures/skips. The set includes exact
  full-family symmetry and numerical tangent continuity. Five Python policy and
  teleop-adapter tests also passed.
- `open_area/OFF`, teleop `0.15 m/s`: 81 symmetric paths over `[-4,+4]`,
  `required_stop_length=0.1325 m`, 13 admissible contiguous members at indices
  34–46, selected straight member 40, exact stored transitions, all failed
  poses excluded, and all out-of-window members excluded from clusters. The old
  3–10 cm self-filter failures disappeared; eligible paths reached 3–4 m and
  ended only at the configured cap, real costmap boundary, or lethal cell.
- `dense_crowd/OFF`: 39/39 observed path cycles were `ok`; test speed `0.15`,
  81 paths, 13 admissible contiguous members, and 0.74 m measured displacement.
- `dense_crowd/ON`: a real `/tracked_agents` publisher and dynamic-node
  subscriber were present, tracks stayed fresh, 40/40 observed path cycles had
  tracks, nonzero straight/turn commands passed final recheck, and odometry
  moved 0.64 m. Four cycles correctly reported no safe selected motion rather
  than bypassing prediction/safety.
- Zero command produced controlled-stop footprint markers. Pure rotation
  produced `safe_in_place_rotation` at `0.25 rad/s`, 14 braking poses, a rotation
  swept-footprint marker, and no translational candidate path. Cached family
  markers were renamed `stale_*`; measured remaining lifetimes decayed from
  0.65 s to 0.05 s.
- Live marker messages contained all 81 candidate lines at `0.005 m`, the
  selected line at `0.010 m`, transition/failure markers, and braking/stop
  footprint namespaces.
- The final installed acceptance checker passed every printed gate in
  `open_area/OFF` and `dense_crowd/ON`. The offline scene/configuration audit
  also passed all scenarios, TF roots, URDF-derived footprint, sensor/odometry
  plugins, costmap frames, assets, goals, and deterministic seed materialization.

Statically verified:

- launch-to-executable and topic mapping;
- old policy files and build/launch/message references removed;
- fixed upstream revisions remain pinned and the dynamic node does not replace fixed executables;
- OFF has no structured-track subscription;
- ON selects TORSO explicitly rather than using the first segment;
- one `/cmd_vel` publisher by construction;
- clean compilation of all five image packages.

The interactive graphical RViz window was not visually inspected by a human
during this headless validation. Marker arrays, topics, namespaces, types,
widths, lifetimes, point counts, transitions, and selected member were verified
from live ROS messages. `open_area/ON` was not run separately; ON was exercised
with actual tracks in `dense_crowd`. Fixed-DWAL regression was not rerun for this
change because its executable/configuration were not modified.
