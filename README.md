# Crowd-Aware Front-Following for Assistive Mobile Robots

A ROS 2 research workspace for robust front-following in crowded and dynamic environments. The project combines shared situation awareness and front-following objectives with two local-navigation approaches: DWB extended through social critics and a fork of HATEB extended with social constraints.

## Documentation workflow

The project uses [`docs/decisions.md`](docs/decisions.md) as its current **single source of truth**. It contains the accepted requirements, assumptions, architectural decisions, and implementation choices.

Each weekly presentation summarizes:

- the main takeaways from the material studied that week;
- the changes made to `decisions.md`;
- the code, simulations, or experiments run that week and their results.

When a decision changes, `decisions.md` is updated first. Weekly presentations report the change but do not act as separate project documentation.

## Repository structure

```text
crowd_aware_ws/
├── docs/
│   └── decisions.md                 # Current single source of truth
│
├── src/
│   ├── crowd_aware_interfaces/
│   │   └── Shared message, service, and action definitions
│   │
│   ├── crowd_aware_context/
│   │   └── Situation awareness and shared context
│   │
│   ├── front_following_manager/
│   │   └── User selection and front-following objectives
│   │
│   ├── crowd_aware_dwb_critics/
│   │   ├── Formation critic
│   │   ├── Personal-space critic
│   │   ├── Pedestrian-prediction critic
│   │   └── Motion-smoothness critic
│   │
│   ├── crowd_aware_hateb_extension/
│   │   ├── Formation constraints
│   │   ├── Crowd and social constraints
│   │   └── Context-dependent weights
│   │
│   ├── hateb_fork/
│   │   └── Minimally modified HATEB/CoHAN source
│   │
│   ├── crowd_aware_bringup/
│   │   ├── launch/
│   │   └── config/
│   │       ├── dwb.yaml
│   │       └── hateb.yaml
│   │
│   ├── crowd_aware_simulation/
│   └── crowd_aware_evaluation/
│
├── dependencies.repos
├── README.md
└── .gitignore
```

## Dependency strategy

- **DWB** remains an upstream Nav2 dependency. Project-specific behavior is implemented as external plugins in `crowd_aware_dwb_critics`.
- **HATEB/CoHAN** is maintained as a separate fork and imported into `src/hateb_fork` through `dependencies.repos`. Changes to the upstream source should remain minimal.


---
## Running the local café experiments

The verified environment is the pinned ROS 2 Humble / Gazebo Classic container.
Build it once:

```bash
cd ~/crowd_aware_ws
bash scripts/run_dwal_cafe.sh build
```

Select one controller and one semantic mode:

```bash
bash scripts/run_dwal_cafe.sh run fixed_dwal off 1
bash scripts/run_dwal_cafe.sh run fixed_dwal on 1
bash scripts/run_dwal_cafe.sh run dwb off 1
bash scripts/run_dwal_cafe.sh run dwb on 1
bash scripts/run_dwal_cafe.sh run hateb off 1
bash scripts/run_dwal_cafe.sh run hateb on 1
bash scripts/run_dwal_cafe.sh run dynamic_dwal off 1
bash scripts/run_dwal_cafe.sh run dynamic_dwal on 1
```

Run the matching live gate from a second terminal while the task is moving:

```bash
bash scripts/run_dwal_cafe.sh check dynamic_dwal on 1
```

For the separate interactive DWAL demonstration, launch with
`REFERENCE_MODE=teleop`, then attach the keyboard:

```bash
SCENARIO=dense_crowd REFERENCE_MODE=teleop RVIZ=true bash scripts/run_dwal_cafe.sh run dynamic_dwal on 1
bash scripts/run_dwal_cafe.sh teleop
```

`dynamic_dwal` is the single C++ executable
`crowd_aware_simulation/dynamic_dwal_node`; Docker builds it from this checkout
and the launch file loads the generated installed `dynamic_dwal.yaml`. It now
publishes the full symmetric strict arc-line family on
`/dynamic_dwal/markers`, while dynamic-window-ineligible members remain visible
but cannot enter clusters or commands. Candidate/selected widths default to
`0.005/0.010 m`; labels and widths are parameters in `experiment.yaml`.

Run the two required environments explicitly:

```bash
SCENARIO=open_area REFERENCE_MODE=teleop RVIZ=true \
  bash scripts/run_dwal_cafe.sh run dynamic_dwal off 1
bash scripts/run_dwal_cafe.sh teleop dynamic_dwal off 1
SCENARIO=open_area bash scripts/run_dwal_cafe.sh check dynamic_dwal off 1

SCENARIO=dense_crowd REFERENCE_MODE=teleop RVIZ=true \
  bash scripts/run_dwal_cafe.sh run dynamic_dwal on 1
bash scripts/run_dwal_cafe.sh teleop dynamic_dwal on 1
SCENARIO=dense_crowd bash scripts/run_dwal_cafe.sh check dynamic_dwal on 1
```

The precise failure taxonomy and first failing point/cell/ray are published in
`/dynamic_dwal/diagnostics`. See [clearer_dwal_report1.md](clearer_dwal_report1.md)
for the proven self-filter root cause, safety semantics, marker namespaces, and
validation evidence.

---
## Run diff controllers different usecases
```bash
 SCENARIO=<scenario> REFERENCE_MODE=teleop GUI=true HEADLESS=false RVIZ=true bash scripts/run_dwal_cafe.sh run <controller> <on|off> <seed>
```
example
```bash
SCENARIO=dense_crowd REFERENCE_MODE=teleop GUI=false HEADLESS=true RVIZ=true bash scripts/run_dwal_cafe.sh run dynamic_dwal on 1
```

```bash
bash scripts/run_dwal_cafe.sh teleop
```

`<scenario>`
junction
narrow_corridor
group_blocking
dense_crowd
crossing
cafe
target_confusion **
doorway
occlusion **
open_area

`<controller>`

fixed_dwal
dynamic_dwal
dwb
hateb

`<seed>`

Θετικός ακέραιος:

1, 2, 3, ..., 10
