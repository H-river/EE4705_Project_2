# EE4705 Project 2 — Manipulator Collision Avoidance with STOMP (Group 9)

MATLAB implementation of STOMP (Kalakrishnan et al., ICRA 2011) for collision-free path
planning, extended with a second manipulator, Product-of-Exponentials forward kinematics,
custom multi-obstacle scenarios and an end-effector orientation constraint.

## Requirements
- MATLAB R2021a or newer (uses `vecnorm`, `'all'`, implicit expansion, `exportgraphics`) — tested with R2023b
- Robotics System Toolbox (`loadrobot`, `getTransform`, `inverseKinematics`, `checkCollision`)
- Statistics and Machine Learning Toolbox (`mvnrnd`)
- Image Processing Toolbox (`bwdist`, used for the signed distance transform)

## How to run
Each task has one entry script. Open it and press **Run** — it adds `common/` to the path itself.

| Task | Entry script | What it shows | FK |
|---|---|---|---|
| 1 | `Task1_Kinova/main_task1.m` | Kinova Gen3, original scenario, animation | `getTransform` |
| 2 | `Task2_NewRobot/main_task2.m` | Franka Emika Panda (`common/config/robotConfig.m`) in a Task-1-like scene; collision-sphere coverage plot | `getTransform` |
| 3 | `Task3_PoE/main_task3.m` | Screw-axis table, PoE accuracy/speed vs `getTransform` (`compareFK.m`), same-seed planning with both | PoE vs `getTransform` |
| 4 | `Task4_Scenarios/main_task4.m` | Three custom multi-obstacle scenarios + 10-seed robustness study | PoE |
| 5 | `Task5_EEConstraint/main_task5.m` | EE y-axis kept upright: weight sweep, then with vs. without constraint | PoE |

Outputs (`.mat`, figures, `.csv` tables, optional videos) go to each task's `results/` folder,
which is created on the first run. All runs are seeded (`rng`), so they are reproducible.
Videos: set `cfg.recordResultVideo = true` (Task 4: `makeVideos = true`).

## Repository layout
```
common/
  runPipeline.m            IK -> voxel world -> STOMP -> collision check -> video -> save
  config/                  defaultConfig.m (all parameters), robotConfig.m (robot for Tasks 2-5)
  robot/                   buildRobotModel.m, solveInitFinal.m
  kinematics/              updateJointsWorldPosition.m (getTransform | PoE), computeScrewAxes.m, fkPoE.m
  world/                   buildVoxelWorld.m (multi-obstacle voxelisation + sEDT), sEDT_3d.m
  scenes/                  one function per scenario, returns a `scene` struct
  stomp/                   runSTOMP.m and the stomp* building blocks
  viz/                     visualiser, animation/video, shared report plots
Task1_Kinova/ … Task5_EEConstraint/   entry scripts + task-specific helpers
original_code/             the provided sample code, unmodified (reference only)
```

## Shared interfaces (agree before changing)
- **`model = buildRobotModel(robotName, eeName, fkMode)`** — `fkMode` is `'getTransform'` or `'PoE'`.
  Only movable joints are kept (`model.jointBodyNames`); the end-effector is always appended as the
  last frame (`model.fkBodyNames`). PoE screw axes are computed once here.
- **`[X, T] = updateJointsWorldPosition(model, theta)`** — `X` is `nFK x 4`, `T` is a `1 x nFK` cell;
  `T{end}` is the end-effector pose. Same output for both FK modes.
- **`scene` struct** — `name, taskInit, taskFinal (or q0, qT), obstacles(type/center/dims), envSize,
  voxelSize, axisLimits, viewAngle` (see `common/scenes/scene_kinova_default.m`).
- **`cfg` struct** — every tunable number lives in `common/config/defaultConfig.m`.
- **`result` struct** — returned by `runPipeline`; see its header for fields.

## Changes to the provided code
Interface changes (so one code base serves all five tasks):
- `updateJointsWorldPosition` / `stompTrajCost` take `model` (+ `cfg`) instead of `robot_struct`.
- Scripts `helperSTOMP`, `helperCreateObstaclesKINOVA`, `helperInitialVisualizerKINOVA` became the
  functions `runSTOMP`, `buildVoxelWorld`, `initVisualizer`.

Bug fixes:
- Obstacle cost returned 0 for samples outside the voxel world (`try/catch` swallowed the index error).
- Control cost summed cross-joint terms (`sum(theta*R*theta','all')` instead of the trace).
- `qc_cost = 0` inside the loop overwrote the constraint-cost vector with a scalar.
- The safety margin was hard-coded (0.05) while the main script's `safetyDistance` was never used.
- The training-video block created an empty `.avi` when disabled and indexed past the end of
  `theta_animation` when the iteration count was a multiple of 5.
- The zero-gradient stop test used `sum(...) == 0` (positive and negative entries can cancel).
- Kinova: `EndEffector_Link` was not among the frames, so the last link had no collision spheres.
- Prismatic joints were wrapped to [-pi, pi] (`solveInitFinal` now checks the joint type).
- An obstacle partly outside the voxel world made `sub2ind` error (`buildVoxelWorld` clamps).
- Coincident frames (Panda link1/link2, link5/link6) have length 0 with `getTransform` but
  ~1e-17 with PoE; `ceil` then changed the number of collision spheres between waypoints and
  `stompTrajCost` crashed (`stompRobotSphere` now uses a small tolerance).

Additional features:
- **PoE forward kinematics** (`computeScrewAxes`, `fkPoE`): screw axes computed once at the
  zero configuration; one running product with closed-form Rodrigues exponentials gives all
  frames. Same result as `getTransform` to ~1e-15, FK ~35x faster, STOMP iteration ~13x faster;
  screw axes cross-checked with the matrix logarithm.
- **Multi-obstacle voxelisation**: boxes (optionally rotated), spheres and upright cylinders.
- **Ground-truth collision check** with the robot meshes (`checkTrajCollision`), independent of
  the sphere model STOMP optimises.
- **Robustness studies**: Task 4 runs each scenario with 10 seeds; Task 5 sweeps the
  constraint weight over 5 seeds per weight and picks it automatically.
- **Convergence patience** (`cfg.convergencePatience`): stop only after several consecutive
  small cost changes instead of the first one.

## Team
| Member | Tasks |
|---|---|
| Joshua Lim | Task 1, Task 2, framework, README, clean-environment reproduction check |
| Hu Jiamo | Task 3 (PoE), Task 4 (scenarios, multi-obstacle voxelisation) |
| T5 owner | Task 5, shared plotting functions, report and slides integration |

## Reference
M. Kalakrishnan, S. Chitta, E. Theodorou, P. Pastor and S. Schaal, "STOMP: Stochastic trajectory
optimization for motion planning," ICRA 2011, pp. 4569–4574.
