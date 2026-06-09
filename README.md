# perceptive_legged_control

## 2026-06-09 14:44:58 UTC

修复感知 reference manager 在 `lie_down` 模式下错误覆盖 base 参考高度的问题。

### 问题原因

P1 两份 reference 配置分别为：

```text
reference.info           comHeight = 0.58
reference_lie_down.info  comHeight = 0.20
                         locomotionComHeight = 0.58
```

手柄切换 `lie_down` 时会发布目标 base 高度 `0.20`。此前 `PerceptiveSwitchedModelReferenceManager::modifyReferences()` 无条件使用 `locomotionComHeight_` 修正目标：

```text
target z = terrain z + locomotionComHeight / cos(pitch)
```

因此 `lie_down` 发布的 `0.20` 会被重新覆盖成约 `0.58`。

### 修复

地形适配现在保留每条目标轨迹自身给出的相对地面高度：

```text
requestedRelativeHeight = target state base z
target z = terrain z + requestedRelativeHeight / cos(pitch)
```

只有目标高度无效或小于等于零时，才回退到 `locomotionComHeight_`。

结果：

```text
lie_down -> terrain z + 0.20 / cos(pitch)
stand    -> terrain z + 0.58 / cos(pitch)
locomotion cmd_vel -> 仍由 target publisher 提升/保持 locomotionComHeight
```

修改文件：

```text
src/interface/PerceptiveSwitchedModelReferenceManager.cpp
```

### 验证

```bash
colcon build --packages-select perceptive_legged_control --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
ros2 launch perceptive_legged_control perceptive_stack.launch.py --show-args
```

两项均已通过。尚未进行 Gazebo 中的完整 `stand -> lie_down -> stand` 动态验证。


## 2026-06-09 15:06:54 UTC - 楼梯接触失稳约束对齐

分析最新日志 `/workspace/mpc_logs/session_20260609_145856`：机器人在楼梯上先出现预期落地脚未建立接触、实际 mode 从正常 trot 的 `9/6` 扩散为多种非规则接触组合，随后横滚从约 `0.4 rad` 快速增长到 `1.57 rad` 并触发 SafetyChecker。`cmd_vel.linear_y` 与 `angular_z` 均为零，侧翻并非手柄横向指令直接造成。

当前感知 P1 配置只启用了普通 FootPlacement 软约束，而 NACL 的配置加载默认启用 FootPlacement CBF、SwingFootPlacement CBF 和 FootCollision。为与 NACL 的实际 OCP 接线保持一致，本次仅修改感知专用配置：

```text
footPlacement.enableCBF = true
footPlacement.enableSwingCBF = true
CollisionParam.enablefootCollision = true
CollisionParam.enableSphereSdfConstraint = false
```

Sphere-SDF 继续保持关闭，与 NACL 默认配置一致。未修改原始 `legged_control`、步态、世界模型或手柄逻辑。

验证：

```bash
colcon build --base-paths /workspace/src --packages-select perceptive_legged_control --symlink-install
```

构建通过。仍需在 Gazebo 重跑同一路径，确认实际接触 mode 能稳定保持 trot 的 `9/6`，并检查是否出现 MPC infeasible 或足端约束告警。


## 2026-06-09 15:13:50 UTC - 修正爬楼速度目标的 RPY 垂直耦合

分析最新日志 `/workspace/mpc_logs/session_20260609_150816`：失稳前实际接触 mode 出现多种非规则组合，base 竖直速度明显振荡，随后 roll 超过 SafetyChecker 阈值。该会话同时存在较大的前进、横移和转向指令，因此机器人也偏离了窄楼梯通道；日志中未发现 MPC infeasible 或约束求解告警。

进一步对比代码发现，当前 `PerceptiveTargetTrajectoriesPublisher` 沿用了通用 legged 目标发布逻辑：使用机身完整 roll/pitch/yaw 旋转手柄线速度。爬楼时机身 pitch/roll 较大，原本的平面前进/横移命令会被投影出非零世界系竖直速度，并写入 MPC 目标状态，可能持续干扰摆腿和接触稳定性。

NACL 的感知专用轨迹生成使用 heading/lateral/yaw-rate 平面命令，并由地形参考生成器单独适配 base 高度和姿态；其旧的完整 RPY 速度旋转路径已被停用。为对齐该语义，本次仅修改感知专用文件：

```text
src/PerceptiveTargetTrajectoriesPublisher.cpp
```

修改内容：

- `cmd_vel.linear.x/y` 仅按当前 yaw 旋转到世界平面。
- 世界系目标 `v_z` 固定为零，不再由机身 pitch/roll 产生。
- 保留前后、横移和 yaw 手柄控制；地形高度与 pitch 继续由感知 reference manager 处理。
- 未修改原始 `legged_control`、步态、世界模型或手柄节点。

验证：

```bash
colcon build --base-paths /workspace/src --packages-select perceptive_legged_control --symlink-install
```

构建通过。仍需先用仅前进、低速命令复测直楼梯，再逐步加入横移和转向，区分控制稳定性与主动离开楼梯通道两类问题。


## 2026-06-09 15:22:11 UTC - 对齐感知 MPC 的 locomotion reference

分析最新会话 `/workspace/mpc_logs/session_20260609_151416` 和 `/workspace/mpc_logs/session_20260609_151502` 后，确认此前速度投影修复已生效，但机器人仍在步态切换后出现不规则接触并侧翻。

### 根因

NACL 的感知控制器启动文件明确给 MPC controller 加载普通站立/行走 `reference.info`。当前 P1 仿真启动文件默认给 controller 加载 `reference_lie_down.info`，而感知目标发布器使用站立 `reference.info`。这造成 MPC 内部默认关节态与行走目标不一致：

```text
controller default: HFE=1.35, KFE=-2.55, base z=0.20
locomotion target:  HFE=0.635, KFE=-1.188, base z=0.58
```

最新目标日志也记录了这两组姿态在同一会话内切换。感知约束虽然已创建，但凸区域选择、运动学和代价是在不一致的 reference 基础上运行。

另外确认当前 `joy_control.py` 的 stand/lie_down 仍通过 `legged_robot_mpc_target` 发布，并未绕过 MPC。因此高度适配必须保留姿态目标自身的相对高度。

### 修改

全部修改限制在 `perceptive_legged_control`：

- 新增 `config/p1/reference.info`，作为感知 MPC 专用站立/行走 reference。
- controller 配置与感知 target publisher 统一读取该文件。
- 新增 `launch/perceptive_p1_sim.launch.py`，向原 P1 仿真显式传入感知 task、感知 reference 和 `PerceptiveLeggedController`。
- base 高度从 `terrainZ + requestedHeight / cos(pitch)` 改为 `terrainZ + requestedHeight`。
- 行走时得到 NACL 对应的 `terrainZ + 0.58`；lie_down 仍保持 `terrainZ + 0.20`。
- 未修改原始 `legged_control` 文件。

后续仿真必须使用：

```bash
source /workspace/install/setup.bash
ros2 launch perceptive_legged_control perceptive_p1_sim.launch.py
```

不要再直接用默认参数启动 `p1_sim.launch.py`，否则 controller 会再次读取 `reference_lie_down.info`。

### 验证

```bash
colcon build --base-paths /workspace/src --packages-select perceptive_legged_control --symlink-install
ros2 launch perceptive_legged_control perceptive_p1_sim.launch.py --show-args
```

构建与启动描述解析通过。Gazebo 动态复测待执行。日志中的 `target_plan_expired` 仍需作为下一项单独对齐，避免停止手柄后参考轨迹不再持续更新。


## 2026-06-09 15:27:02 UTC - 最新楼梯日志复核

本次仅分析日志，未修改代码。最新有效会话为 `/workspace/mpc_logs/session_20260609_152517`。

结论：reference 与高度修复后，机器人在 `mpc_time≈45-63s` 的主要爬楼阶段保持稳定，roll 基本接近零，说明感知高度适配并未先发散。失稳与横向命令时间严格重合：`15:26:22` 起 `cmd_vel.linear_y` 从约 `0.21m/s` 增长到接近 `0.50m/s`，同一秒 roll 首次超过 `0.35rad`，约 `0.2s` 后超过 `0.70rad`，随后实际接触 mode 从正常 `9/6` 扩散为 `0/1/2/4/10/11/12` 等组合并翻倒。

直楼梯宽度为 `1.20m`，中心位于 `y=0`，边界为约 `[-0.60, 0.60]`。失稳开始时右侧足端已达到约 `y=-0.66m`，落到台阶边界之外。当前 FootPlacement 约束只限制单脚选择到的局部凸平面，不约束 base/全身必须留在楼梯通道内，因此较大的横移命令仍可使机身和足端越过窄台阶边缘。

对照会话 `/workspace/mpc_logs/session_20260609_152419`：机器人走到 `x=4.405m、base_z=1.748m` 时仍保持 mode `9`、roll 约 `-0.003rad`，说明当前 reference 对齐后存在稳定爬楼区间。

日志中的 `target_plan_expired` 发生在多个较早时刻，但失稳前目标发布已恢复，目标高度保持 `0.58m` 相对高度，因此它不是本次侧翻的直接触发因素。

后续正确方向是补充 NACL 上层未提供的楼梯通道/机身可行域保护，或在地形边缘依据足端凸区域降低横移速度；不应继续修改 `terrain + requestedHeight` 高度公式。为遵守“不做牺牲性适配”，本轮没有直接限幅或禁用横移功能。


## 2026-06-09 15:33:19 UTC - 恢复 NACL 落脚区域边缘余量

根据用户对“落脚点先出错、随后腿部接触紊乱”的判断，继续对照当前实现与 NACL 源码。

确认 `PerceptiveLeggedPrecomputation::getPolygonConstraint()` 在移植时漏掉了 NACL 原有的多边形边缘内缩项：

```text
polytopeB = edge_expression - 0.04
```

此前当前工程只使用 `edge_expression`，允许优化足端贴近凸平面/台阶边缘。最新日志中失稳前右侧足端达到约 `y=-0.66m`，而直楼梯边界约为 `y=-0.60m`，与缺少落脚安全余量的风险一致。

本次仅修改感知包：

- 在足端凸多边形约束中恢复 NACL 的 `-0.04` 边缘内缩。
- 将支撑相 FootPlacement CBF 参数对齐 NACL 配置：`cbfLambda=1.0`、`cbfPenaltyDeltaParam=1e-4`。
- 保持 SwingFootPlacementConstraintCBF 与 NACL 当前实际实现一致，仍使用带相位放宽的足端位置约束；未启用 NACL 源码中被注释的速度 CBF 返回式。
- 未修改原始 `legged_control`、手柄横移功能、世界模型或 base 高度公式。

曾检查 `PerceptiveLeggedPrecomputation` 的 RequestSet 条件。当前 SQP 普通节点会同时请求 `Constraint + SoftConstraint`，因此它不是本次直接根因，未作修改。

验证：

```bash
colcon build --base-paths /workspace/src --packages-select perceptive_legged_control --symlink-install
```

构建通过。下一次复测应重点观察 RViz 足端凸区域与实际落脚点，并比较失稳前足端是否仍越过台阶边界；若仍发生，应继续检查 `ConvexRegionSelector` 是否在边缘选择了相邻地面区域。


## 2026-06-09 15:39:43 UTC - 修正台阶落脚区域过度收缩

最新有效日志 `session_20260609_153617` 显示机器人仅接受前进命令，并在 `x≈1.16m` 首次踩到台阶后迅速侧翻。根因是上一轮直接照搬 NACL 的 `polytopeB -= 0.04`，但当前多边形约束的边法向量没有归一化，该常数并不代表固定 4 cm 距离。

对于深度 0.30m 的台阶踏面，原始写法会让对应方向每侧实际收缩约 0.133m，只留下约 0.033m 可行区域。这解释了足端一接触第一阶便失去稳定落脚点的现象。

本次将安全边距改为：

```cpp
polytopeB(i) -= 0.04 * polytopeA.row(i).norm();
```

约束先统一朝向，再按照法向量长度缩放。这样所有边界的实际内缩距离均为 4 cm，0.30m 踏面仍保留约 0.22m 可落脚宽度。FootPlacement CBF 参数继续保持与 NACL 对齐，其他控制逻辑未修改。

验证：

```bash
colcon build --base-paths /workspace/src --packages-select perceptive_legged_control --symlink-install
```

构建通过。动态效果仍需重新启动 `perceptive_p1_sim.launch.py` 后复测第一阶落脚。


## 2026-06-09 15:52:56 UTC - 撤回导致首阶接触紊乱的约束增强

最新日志 `session_20260609_154610` 表明，机器人并非机身到达台阶后才倾倒。`mpc_time≈22.10s` 时 base 仍在 `x≈0.66m`，前足已摆到 `x≈1.11m`；此时观测接触模式从正常 trot 的 `6` 突然变为 `14→0→1`，约 0.12 秒后姿态越过安全阈值。因此直接起因是首个跨阶落脚期间接触模式崩溃。

该回归与上一轮同时加入的多边形内缩和更强支撑相 CBF 参数高度相关。本次只撤回这三处：

- 移除额外的凸多边形内缩。
- `cbfLambda` 从 `1.0` 恢复为 `0.5`。
- `cbfPenaltyDeltaParam` 从 `1e-4` 恢复为 `0.005`。

FootPlacement CBF、SwingFootPlacement CBF 和 FootCollision 仍保持启用，没有通过关闭感知约束换取稳定性。感知包已重新构建通过。下一轮应完整重启仿真，并检查第一阶前的接触模式是否稳定保持 `9/6`。


## 2026-06-09 16:09:26 UTC - 单独恢复落脚多边形边距

按要求重新加入归一化的 4 cm 落脚多边形内缩：

```cpp
polytopeB(i) -= 0.04 * polytopeA.row(i).norm();
```

本次没有同时增强 CBF。`cbfLambda=0.5`、`cbfPenaltyDeltaParam=0.005` 保持不变，因此下一轮复测可以单独判断几何内缩是否影响第一阶落脚。感知包构建通过。


## 2026-06-09 16:13:26 UTC - 撤回固定 4 cm 落脚内缩

单变量复测确认：在 CBF 参数保持 `lambda=0.5、delta=0.005` 时，仅加入固定 4 cm 几何内缩也会使楼梯表现变差。因此已再次移除该内缩，恢复无额外内缩的稳定版本。全部感知约束仍保持启用，感知包构建通过。后续边缘余量应采用与凸区域尺寸相关的自适应策略。


## 2026-06-09 16:20:17 UTC - 对齐坡面 locomotion 高度补偿

感知 reference manager 已恢复 `legged_perceptive` 原版的坡面法向高度保持逻辑，并与 lie_down 分流：

- 站立/行走：`terrainZ + requestedHeight / max(cos(pitch), 0.5)`。
- lie_down：保持 `terrainZ + requestedHeight`。

模式依据目标轨迹最终相对高度判断，避免单个插值节点导致补偿反复切换。30 度时 locomotion 的 0.58m 高度会转换为约 0.670m 竖直高度。感知包构建通过，需动态复测 30 度楼梯和坡面 lie_down。
