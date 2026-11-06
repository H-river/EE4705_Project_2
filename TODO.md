# TODO 索引（组内用，提交前删除或移出 zip）

**截止**：11 月 10 日 14:00 交报告和代码（`Group9_Report_Code.zip`）；同日 16:00 做展示（SDE-LT422，8 分钟讲 + 4 分钟答辩，每人讲自己的部分）。

**查剩余 TODO**：在 MATLAB 里用 `Find Files`，或在终端 `grep -rn "TODO\[" common Task*`。每个 TODO 都有 `[编号] (Owner: …)` 标签，做完就删掉标签，再把下表的状态改成 ✅。

Owner 代号：**T12** = Task 1+2 负责人，**Jiamo** = Task 3+4，**T5** = Task 5 负责人。

## 依赖（谁会卡住谁）
```
T2-1 选机器人 ──► T3-1 screw axes(新机器人)  ──► T4 场景用 PoE 跑
               └► T5-2a 末端姿态的起止位姿
T1-1..5 STOMP 能跑 ──► 所有 Task 的规划结果
T5-3 画图函数 ──► 所有 Task 的报告图
```
- **不会被卡**：Task 3 先在 Kinova 上原型（`main_task3.m` 里 `useKinova = true`）；Task 5 的 `stompConstraintCost` 和画图函数可以先写。
- **T2-1 尽早定**：定了就在 `robotConfig.m` 里改 `isDecided = true`，并在群里通知。

## Task 1+2（Joshua Lim，约 11 h）
| 编号 | 文件 | 内容 | 状态 |
|---|---|---|---|
| T1-1a–e | `common/stomp/runSTOMP.m` | 主循环：采样 → 局部代价 → 概率 → dθ 并更新 → 新代价 | ✅ |
| T1-2 | `common/kinematics/updateJointsWorldPosition.m` | getTransform 分支（直接用 column 格式，比原版的 struct 快） | ✅ |
| T1-3 | `common/stomp/stompDTheta.m` | 按概率加权求 δθ | ✅ |
| T1-4 | `common/stomp/stompSamples.m` | 每个关节用 mvnrnd 采样，首尾补 0 | ✅ |
| T1-5 | `common/stomp/stompObstacleCost.m` | Eq.(13) | ✅ |
| T1-6a | `common/stomp/stompObstacleCost.m` | **Bug**：越界采样的代价被算成 0 | ✅ |
| T1-6b | `runSTOMP.m` 和 `stompTrajCost.m` | **Bug**：控制代价多加了跨关节项，应改成 trace | ✅ |
| T1-7 | `Task1_Kinova/main_task1.m` | Task 1 的报告图 | ✅ |
| T2-1 | `common/config/robotConfig.m` | **选机器人**（外部 URDF 要处理 mesh 路径，见 T2-1b） | ✅ |
| T2-2 | `robotConfig.m`、`stompRobotSphere.m` | 碰撞球半径（可以每段 link 单独设）、检查覆盖 | 🟡 覆盖图已出（`task2_spheres.png`）：5 cm 球大多在连杆网格内侧，靠 5 cm safetyMargin 补偿，网格碰撞检测全部通过；半径没按连杆调 |
| T2-3 | `common/robot/solveInitFinal.m` | 有移动关节时别做 wrapToPi（已按关节类型处理，需要验证） | ✅ |
| T2-4 | `common/scenes/scene_newrobot_base.m` | 新机器人的起止位姿、障碍物、envSize | ✅ |
| T2-5/6 | `Task2_NewRobot/main_task2.m` | 关节和碰撞球的检查图、报告图 | ✅ |
| FW | 根目录 | README 定稿；用全新 MATLAB 环境跑一遍复现检查；打包 zip | 🟡 README 已定稿；打包用 `tools/make_submission.m`（排除 TODO.md 和 tools/，自动加报告 PDF）；还差报告 PDF |

## Task 3+4（Jiamo，约 11 h）
| 编号 | 文件 | 内容 | 状态 |
|---|---|---|---|
| T3-1 | `common/kinematics/computeScrewAxes.m` | 方法 1：在 home 位姿用 getTransform 取 ω、q，v = −ω×q | ✅ |
| T3-1b | 同上 | （可选）方法 2：用矩阵对数交叉验证 | ✅ |
| T3-1c | 同上 | 报告用的 screw axis 表，对应 URDF 的 origin 和 axis | ✅ |
| T3-2 / 2b | `common/kinematics/fkPoE.m` | 累乘一次算出所有关节帧，Rodrigues 闭式解 | ✅ |
| T3-3a/b | `Task3_PoE/compareFK.m` | 和 getTransform 比精度、比速度 | ✅ |
| T3-4 | `Task3_PoE/main_task3.m` | 同一随机种子下规划结果一致性、每次迭代的耗时 | ✅ |
| T4-1a | `common/world/buildVoxelWorld.m` | 越界 clamp | ✅ |
| T4-1b/c | 同上 | 球体、圆柱体素化 | ✅ |
| T4-1d | 同上 | （可选）旋转的 box | ✅ |
| T4-2 | `common/scenes/scene_t4_{1,2,3}.m` | 2–3 个场景，直线初始化必须会碰撞 | ✅ |
| T4-3 | `Task4_Scenarios/main_task4.m` | 结果表（可选：多随机种子的成功率）、图、视频 | ✅ |

## Task 5（T5，末端约束 + 全组画图 + 报告/slides）
| 编号 | 文件 | 内容 | 状态 |
|---|---|---|---|
| T5-1 | `common/stomp/stompConstraintCost.m` | 末端 y 轴竖直，用 ℓ1 范数 | ✅ |
| T5-2a | `common/scenes/scene_t5.m` | 起止位姿本身就是竖直的 | ✅ |
| T5-2b | `Task5_EEConstraint/main_task5.m` | 调 w_con，做权重扫描 | ✅ |
| T5-2c/d | `main_task5.m`、`plotEEAxisDeviation.m` | 加约束前后对比（每个 waypoint 的偏差角） | ✅ |
| T5-3a | `common/viz/plotConfigOverlay.m` | 构型叠加图（全组共用） | ✅ |
| T5-3b | `common/viz/plotCostHistory.m` | 代价曲线（全组共用） | ✅ |
| T5-4 | — | 整合报告（每人写自己的部分 + 贡献说明）、整合 slides | ⬜ |

## 加分项（谁有空谁做，报告里明确写出来）
- OPT-1 `initTrajectory.m`：连续关节按最短角度插值。
- `runSTOMP` 里对每个采样的代价计算改用 `parfor`。
- 多随机种子的成功率统计（T4-3）。

## 框架里已经修好的问题（报告"改进"一节可以直接引用）
- 视频段：关闭录制时仍会建空 avi；`k+1` 索引越界。
- 零梯度停止条件：原来用 `sum == 0`，改为 `all == 0`。
- safetyDistance 没被用到：现在统一由 `cfg.safetyMargin` 控制。
- Kinova 末端 link 没有碰撞球：`fkBodyNames` 末尾补上 EE。
- `qc_cost` 被标量覆盖：改为按 waypoint 下标赋值。
- 移动关节被 wrapToPi：现在按关节类型判断。
