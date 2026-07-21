---
project: Crowd-Aware Front-Following for Assistive Mobile Robots
document_role: Single source of truth for the weekly presentation
language: English
slide_generation_rule: Every level-2 section maps to exactly one slide. When a topic spans multiple slides, every title uses the same base title followed by i/N.
visual_style: Minimal, concise, technically precise, and consistent with the HERON Beamer theme.
assets:
  problem_source: project_sources/01-Screenshot-2026-07-15-134123.png
  robot_type_taxonomy: images/taxonomies_robot_type.png
  planning_taxonomy: images/taxonomies_planning_decision_making.png
  situation_awareness_taxonomy: images/taxonomies_situation_awareness_and_assessments.png
  evaluation_taxonomy: images/taxonomies_tools_evaluation_methods.png
---

# Weekly Presentation Decisions

## Weekly Internship Meeting

- Subtitle: Crowd-Aware Front-Following for Assistive Mobile Robots
- Presenter: Marina Fragkouli
- Supervisor: Antreas Vatistas
- Affiliation: HERON — Hellenic Robotics Center of Excellence
- Meeting label: Week 1 Update

## Problem Description

- Front-following requires the robot to remain ahead of its intended user while guiding or accompanying them in the user's intended direction.
- Crowded, dynamic spaces introduce target confusion, occlusion, rapidly changing free space, and socially sensitive interactions.
- The robot must preserve the user–robot formation without blocking, surprising, or endangering nearby pedestrians.
- **Contribution:** a Nav2-compatible, crowd-aware local planner/controller that uses semantic context to follow crowd-aware social protocols while keeping an assistive robot safely in front of its intended user.
- Scope: local motion generation and control; perception and tracking are upstream inputs.

## Robot Navigation Pipeline 1/7

- Show the complete navigation pipeline and all major data dependencies.
- Main route: global costmap → global planner → raw global path → optional path smoother → smoothed global path → local planner/controller → raw `cmd_vel` → optional velocity smoother → collision monitor → safe `cmd_vel` → motor controller.
- Localization route: robot sensors → robot EKF/localization → robot pose and TF.
- Perception route: camera/LiDAR detections → pedestrian tracker/KF → individual pedestrian tracks.
- Mapping route: static map + robot pose/TF → global costmap; sensor obstacles + pedestrian-aware local information → local costmap.
- The local planner/controller consumes the smoothed global path, local costmap, robot state, and pedestrian tracks.

## Robot Navigation Pipeline 2/7

- Diagram focus: localization.
- Continuously estimate the robot pose.
- Localization publishes `map → odom`.
- Odometry/EKF publishes `odom → base_link`.
- A charging station may provide a known initial or reference pose.
- Takeaway: every planning input must be expressed in a consistent frame and timestamp.

## Robot Navigation Pipeline 3/7

- Diagram focus: costmaps.
- Global costmap: large, mostly static representation used for global planning.
- Local costmap: rolling, frequently updated representation used for local control.
- Add social costs locally, but retain pedestrian IDs, velocities, predictions, and confidence as separate structured data.
- Takeaway: a costmap alone cannot preserve agent identity or motion state.

## Robot Navigation Pipeline 4/7

- Diagram focus: global planner.
- Compute a global path to the current desired front-following pose.
- The planner is usually graph-search based, commonly A* or a related method.
- Treat the global planner as an existing module, not the main project contribution.
- Replan when the desired front pose moves sufficiently or the global route becomes invalid.

## Robot Navigation Pipeline 5/7

- Diagram focus: local planner/controller.
- This is the main project focus.
- Inputs: global path, local costmap, robot state, intended-user state, and individual pedestrian tracks.
- Generate or optimize feasible short-horizon trajectories, evaluate their costs, and produce `cmd_vel`.
- The controller must combine formation keeping, crowd awareness, feasibility, progress, and smoothness.

## Robot Navigation Pipeline 6/7

- Diagram focus: TF and frames.
- Verify the TF tree: `map → odom → base_footprint/base_link → sensors`.
- Transform detections and goals into the local costmap frame before planning.
- Use `base_link` for robot-relative longitudinal and lateral front-following calculations.
- A frame is a coordinate system; `base_footprint` is the ground-projected robot frame; the footprint is the 2D collision polygon.
- Reject or stop on stale, unavailable, or inconsistent transforms.

## Robot Navigation Pipeline 7/7

- Diagram focus: postprocessing and final safety.
- Nav2 exposes three distinct optional stages: Smoother Server after global planning, Velocity Smoother after local control, and Collision Monitor before actuation.
- Path smoothing modifies geometry; velocity smoothing modifies command dynamics; collision monitoring enforces a final safety layer.
- Example control-smoothness cost:

  $$
  J_{\mathrm{control}}=\sum_{k=1}^{N}\left[\lambda_v(v_k-v_{k-1})^2+\lambda_\omega(\omega_k-\omega_{k-1})^2\right].
  $$

- Safety monitoring remains independent of the learned or rule-based planner logic.

## Our Extension

- Extend the Nav2 local planner/controller rather than replacing the complete navigation stack.
- For each cycle, generate or optimize candidate trajectories and select the valid trajectory with the minimum total cost:

  $$
  \tau^*=\arg\min_{\tau\in\mathcal{T}_{\mathrm{valid}}}J(\tau).
  $$

- Add crowd-aware and formation-aware scoring while reusing localization, maps, global planning, command postprocessing, and final collision monitoring.
- Inputs: user state, desired front pose, bystander tracks, robot state, local costmap, and global path.
- Output: a safe, smooth, socially aware velocity command.

## System Taxonomies & Evaluation Framework

- Preserve the original four-panel taxonomy overview.
- Panels and captions:
  - Taxonomy based on robot type — leaves are mostly mutually exclusive.
  - Taxonomy based on planning and decision-making — facets are not mutually exclusive.
  - Taxonomy based on situation awareness and assessment.
  - Taxonomy for tools and evaluation methods.

## Taxonomy Based on Robot Type

- Show the corresponding robot-type taxonomy image.
- The project concerns a 2D assistive mobile ground robot moving on approximately planar ground.
- Navigation uses 2D costmaps, a 2D collision footprint, and planar pose:

  $$
  \mathbf{x}=[x,\,y,\,\theta]^T.
  $$

- The controller outputs linear and angular velocity:

  $$
  \mathbf{u}=[v,\,\omega]^T.
  $$

- This scope excludes aerial, underwater, legged, and full 3D motion planning.

## Taxonomy Based on Planning and Decision-Making 1/6

- Show the corresponding planning and decision-making taxonomy image.
- Communication depends on the use case:
  - implicit: trajectory, speed, posture, and gaze;
  - explicit: speech, gestures, displays, or lights;
  - hybrid: predictable motion supported by an explicit signal.
- Project decision: prioritize implicit communication through smooth, legible motion; explicit projection or display is a later extension.
- Social nature of the navigation task:
  - independent navigation among people: baseline/background;
  - assistive navigation: core;
  - collaborative navigation: limited core behavior, not a full collaboration model.
- The task becomes minimally collaborative when the robot yields, waits, or adapts formation so that the user and pedestrians can pass safely.

## Taxonomy Based on Planning and Decision-Making 2/6

- Negotiation in scope: implicit, rule-based negotiation through yielding, slowdown, path adjustment, waiting, and stopping.
- Out of scope: speech-based negotiation and complex learned negotiation models.
- Priority order:
  1. Avoid collisions and satisfy hard safety constraints.
  2. Stop safely when intended-user identity or state is unreliable.
  3. Respect pedestrian flow, personal space, and detected groups.
  4. Preserve the front-following formation.
  5. Keep motion smooth and predictable.
  6. Minimize delay and path length.
- The priority order governs both hard constraints and soft cost tuning.

## Taxonomy Based on Planning and Decision-Making 3/6

- **Local motion generation is the project focus.**
- DWA samples dynamically reachable velocity commands over a short horizon.
- DWB is the Nav2 critic-based evolution of DWA: trajectory generators propose motions and configurable critic plugins score them.
- TEB represents a trajectory as timed poses and optimizes it online against timing, obstacle, and kinodynamic constraints:

  $$
  \mathcal{B}=\{p_0,\Delta t_0,p_1,\Delta t_1,\ldots,p_N\}.
  $$

- The poses are optimization points, not mandatory stops.
- HATEB extends TEB with human-aware, time-dependent robot–human interaction constraints and can jointly account for predicted human motion.

## Taxonomy Based on Planning and Decision-Making 4/6

- DWB samples a set of trajectories $\mathcal{T}$ and evaluates enabled critic plugins:

  $$
  J_{\mathrm{DWB}}(\tau)=\sum_{i=1}^{N_c}s_iC_i(\tau),\qquad
  \tau^*=\arg\min_{\tau\in\mathcal{T}_{\mathrm{valid}}}J_{\mathrm{DWB}}(\tau).
  $$

- $C_i$ is a critic score and $s_i$ is its configured scale.
- There is no single fixed DWB objective because the critic set is configurable.
- Typical critics include obstacle, path alignment, path distance, goal alignment, goal distance, rotation-to-goal, oscillation, and forward preference.
- Project option: add social and formation critics while retaining the DWB architecture.

## Taxonomy Based on Planning and Decision-Making 5/6

- HATEB uses graph optimization over robot poses and time intervals and may also model predicted human trajectories.
- General weighted nonlinear least-squares form:

  $$
  J_{\mathrm{HATEB}}(\mathbf{z})=\sum_k e_k(\mathbf{z})^T\Omega_k e_k(\mathbf{z}).
  $$

- $\mathbf{z}$ contains optimized poses and time differences; $e_k$ is a constraint residual; $\Omega_k$ is its information matrix or weight.
- Practical terms can include velocity, acceleration, kinematics, execution time, path length, static and dynamic obstacles, via points, robot–human safety, relative velocity, and visibility.
- Project direction: extend HATEB for crowd-aware front-following, building on the CoHAN/HATEB architecture.

## Taxonomy Based on Planning and Decision-Making 6/6

- Planner data flow:
  `user state + desired front pose + bystander tracks + local costmap`
  → `safety and context adaptation`
  → `feasible trajectory generation`
  → `crowd-aware cost evaluation`
  → `velocity command`.
- Hard constraints:
  - no collision with static obstacles or pedestrians;
  - robot velocity and acceleration limits;
  - controlled stop when inputs are stale, confidence is too low, or no safe trajectory exists.
- Initial implementation choice: a crowd-aware HATEB/CoHAN extension; a DWB implementation with new social critics remains the principal baseline or fallback.
- Out of scope: force-based local motion generation, reinforcement learning, and global motion generation.

## Context-Dependent Use Cases

| Context | Desired behavior |
|---|---|
| Open area | Maintain the nominal forward distance. |
| Narrow corridor | Align with the corridor and reduce lateral offset. |
| Doorway | Restrict overtaking and wait when passage is unsafe. |
| Junction | Increase emphasis on short-horizon user-intention prediction. |
| Dense crowd | Reduce speed and shorten formation distance within safe bounds. |
| Occlusion zone | Increase caution and rely on short-term target prediction. |

- Decision: the desired front-following pose is context dependent rather than fixed.

## An Example of Local Motion Generation 1/2

- Candidate trajectories are scored with a modular objective:

  $$
  J=w_sJ_{\mathrm{social}}+w_oJ_{\mathrm{static}}+w_cJ_{\mathrm{control}}+w_gJ_{\mathrm{goal}}+w_fJ_{\mathrm{formation}}.
  $$

- $J_{\mathrm{social}}$: penalize intrusion into anisotropic, speed-dependent pedestrian and group space.
- $J_{\mathrm{static}}=\sum_k C_{\mathrm{costmap}}(p_k)$: penalize walls, furniture, and other mapped obstacles.
- $J_{\mathrm{control}}$: penalize acceleration, braking, and rapid steering changes.
- $J_{\mathrm{goal}}$: reward progress along the route and discourage unnecessary stalling or reversal.
- $J_{\mathrm{formation}}$: maintain the context-dependent pose in front of the intended user.
- Hard safety constraints are enforced before comparing soft costs.

## An Example of Local Motion Generation 2/2

- Flow: planner inputs → candidate trajectories → hard-constraint rejection → component-cost evaluation → weighted total → minimum-cost valid trajectory → `cmd_vel`.
- Planner inputs: robot state, global path, local costmap, intended-user state, pedestrian tracks, and semantic context.
- Candidate trajectories that collide, violate dynamic limits, or rely on stale critical inputs are rejected.
- Component costs remain separately logged so that behavior can be explained and tuned.
- If no valid trajectory exists, produce a controlled stop rather than the least-bad unsafe motion.

## Taxonomy Based on Situation Awareness and Assessment 1/4

- Show the corresponding situation-awareness taxonomy image.
- Environment representation retained for this project:
  - static obstacles and local free space;
  - local crowd density;
  - available passage width or constrained-space indication;
  - occlusion and stale-input indication.
- Not required: full scene semantics, human–object interaction recognition, or semantic classification of every room, door, and object.
- Initial decision: derive context from geometry and tracked agents rather than introducing a new semantic-perception model.

## Taxonomy Based on Situation Awareness and Assessment 2/4

| Intended user | Other pedestrians |
|---|---|
| Persistent ID | Individual tracking ID |
| Pose and heading | Pose and heading |
| Velocity | Velocity |
| Short-term predicted state | Short-term predicted state |
| Tracking confidence | Optional group label |

- A collective pedestrian mask is insufficient for prediction and social trajectory scoring; bystanders require individual tracks.
- Prediction baseline: constant velocity/heading, optionally filtered with a Kalman filter.
- Out of scope: RNN, LSTM, GNN, or other learned social-prediction models.

## Taxonomy Based on Situation Awareness and Assessment 3/4

- Temporary target-loss policy:
  1. Retain the intended user's last ID and short-term predicted state.
  2. Never replace the user merely because another pedestrian is nearer.
  3. Reduce speed as tracking confidence falls.
  4. Accept re-identification from the tracking module when the user reappears.
  5. Stop when confidence, prediction age, or collision risk crosses a threshold.
- Interaction and group rules:
  - do not plan through a detected group or cut a human–human interaction;
  - do not treat a pedestrian passing between robot and user as a new target;
  - avoid separating the intended user from a known companion when group data exist.
- First version: group labels supplied by simulation/tracking or inferred using proximity and similar velocity; learned group recognition is out of scope.

## Taxonomy Based on Situation Awareness and Assessment 4/4

- Required short-horizon intent cues: continued straight motion, left/right turn, slowing/stopping, likely path crossing, and temporary target loss.
- Infer approaching and obstacle-deviation behavior from predicted trajectories rather than separate learned classes.
- Proxemic model decisions:

| Model | Decision |
|---|---|
| Circular fixed-radius personal space | Baseline |
| Anisotropic, heading-dependent field | Core |
| Speed-dependent field size | Core |
| Larger joint field for detected groups | Optional/core when group labels exist |

- Social cost is larger in front of a moving pedestrian, smaller behind or laterally, and increases with pedestrian speed.
- The intended user is governed primarily by formation cost while retaining a hard minimum distance.

## Taxonomy for Tools and Evaluation Methods 1/5

- Show the corresponding tools-and-evaluation taxonomy image.
- Evaluation modes: qualitative evaluation, quantitative evaluation, user studies, simulators, datasets, metrics, and benchmarks.
- Intended-user tracking metrics:

| Metric | Measures |
|---|---|
| Target tracking success rate | Fraction of time the correct target is maintained. |
| ID switch count | Incorrect changes to another pedestrian. |
| Target loss rate | Frequency of intended-user loss. |
| Reacquisition time | Time required to restore the correct track. |
| Track continuity | Uninterrupted duration of correct tracking. |
| Target-position error | Intended-user position estimation error. |
| Target-heading error | Intended-user heading estimation error. |

## Taxonomy for Tools and Evaluation Methods 2/5

- Front-following quality metrics:

| Metric | Measures |
|---|---|
| Longitudinal formation error | Error from the desired forward distance. |
| Lateral formation error | Sideways error from the desired formation. |
| Robot–user distance variance | Stability of separation over time. |
| User visibility rate | Fraction of time with a clear view or valid track. |
| Formation recovery time | Recovery time after a disturbance. |
| User stop count | Times the user must stop because of robot behavior. |

- These metrics separate “reached the goal” from “guided the user well.”

## Taxonomy for Tools and Evaluation Methods 3/5

- Safety and crowd-awareness metrics:

| Metric | Measures |
|---|---|
| Collision rate | Collisions per run or unit time. |
| Minimum pedestrian distance | Closest robot–pedestrian separation. |
| Time-to-collision | Dynamic collision risk. |
| Social-space intrusions | Number of personal-space violations. |
| Time inside social spaces | Duration of proxemic violations. |
| Emergency stops | Abrupt safety-layer interventions. |
| Near-miss count | Unsafe encounters without contact. |
| Group-splitting events | Robot passages through social groups. |

## Taxonomy for Tools and Evaluation Methods 4/5

- Motion-quality metrics:

| Metric | Measures |
|---|---|
| Path length | Route efficiency. |
| Completion time | Task duration. |
| Linear/angular acceleration | Command smoothness. |
| Jerk | Abrupt changes in acceleration. |
| Cumulative heading change | Zigzagging and unnecessary turning. |
| Freeze duration/count | Robot-freezing behavior. |
| Replanning frequency | Planner stability. |

- Report safety and formation metrics together; efficiency alone is not sufficient.

## Taxonomy for Tools and Evaluation Methods 5/5

- For a small user study, measure:
  - perceived safety;
  - comfort;
  - trust;
  - predictability;
  - confidence that the robot knows where it is going;
  - confusion;
  - effort required to follow;
  - preference between planners.
- Combine subjective ratings with objective logs from the same scenario.
- Use within-subject comparisons where practical to reduce inter-person variability.
- Human-centered evaluation follows controlled simulation and technical safety validation.

## Immediate Experiment Plan 1/2

- Use visible ID badges to provide intended-user identity ground truth.
- Controlled scenario suite:
  1. One user, no pedestrians.
  2. One pedestrian crossing the route.
  3. Several similarly dressed pedestrians near the user.
  4. Temporary full occlusion.
  5. A pedestrian passing between robot and user.
  6. Narrow corridor.
  7. Junction where the user turns.
  8. Doorway.
  9. Group blocking part of the route.
  10. Rapid change in user speed or direction.

## Immediate Experiment Plan 2/2

- Compare four baselines:
  1. Front-following without crowd awareness.
  2. Front-following with conventional obstacle avoidance.
  3. Front-following with proxemic costs.
  4. Proposed method with tracking confidence, short-horizon prediction, and crowd-aware behavior.
- Keep the same scenario geometry, pedestrian scripts, initial conditions, and safety limits across planners.
- Log pedestrian IDs, target state, chosen trajectory, cost components, commands, and safety events.
- Evaluate target retention, formation error, safety, motion quality, completion, and failure mode.

## Requirements Summary 1/4

- Inputs from the existing system:
  - robot pose, velocity, footprint, local costmap, and global path;
  - intended-user ID, pose, heading, velocity, short-term prediction, and tracking confidence;
  - individual bystander tracks: ID, pose, heading, velocity, and optional group label;
  - desired front-following pose or the data needed to update it.
- Implemented in this project:
  - crowd-aware local trajectory generation;
  - social, formation, obstacle, progress, and smoothness costs;
  - context-dependent speed and formation adaptation;
  - safe behavior under low confidence, occlusion, stale data, or lack of a feasible trajectory;
  - controlled comparison with non-social baselines.

## Requirements Summary 2/4

- **FR-1:** Drive toward a configurable pose in front of the intended user.
- **FR-2:** Continuously generate dynamically feasible, collision-free local motion.
- **FR-3:** Score static obstacles, pedestrian social space, formation error, progress, and motion smoothness.
- **FR-4:** Use anisotropic, heading- and speed-dependent pedestrian costs; retain a circular cost as a baseline.
- **FR-5:** Adapt speed and formation within configured bounds in crossings, dense crowds, constrained passages, and low-confidence states.

## Requirements Summary 3/4

- **FR-6:** Avoid passing through detected groups when group information is available.
- **FR-7:** Retain the provided intended-user ID; never substitute a bystander using nearest-person proximity alone.
- **FR-8:** Reduce speed on stale or uncertain user input and stop when confidence, collision risk, or trajectory feasibility crosses a configured threshold.
- **FR-9:** Avoid unnecessary oscillation, abrupt turns, and repeated stopping.
- Functional behavior must remain compatible with the surrounding Nav2 pipeline and safety layers.

## Requirements Summary 4/4

- **Real-time performance:** update perception, tracking, planning, and control frequently enough for safe operation; do not issue commands based on materially outdated state.
- **Safety:** fail safely when perception, tracking, TF, or planning becomes unreliable; enforce configured velocity and acceleration limits.
- **Modularity:** keep perception, user tracking, prediction, planning, and control as replaceable modules.
- **Configurability:** expose cost weights, social-space parameters, formation distance, context thresholds, and safety limits.
- **Logging:** record planner inputs, candidate/selected trajectories, component costs, commands, tracking state, and safety events for reproducible evaluation.
- **Design principle:** communicate intent first through predictable motion; explicit projection or display may later clarify ambiguous maneuvers.
