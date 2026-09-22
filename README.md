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
REFERENCE_MODE=teleop bash scripts/run_dwal_cafe.sh run fixed_dwal on 1
bash scripts/run_dwal_cafe.sh teleop
```

See [report.md](report.md) for the exact condition semantics, graph inventory,
validation evidence, evaluation commands, and known limitations.
