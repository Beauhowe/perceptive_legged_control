# perceptive_legged_control

## 2026-06-07 10:14:20 UTC

本次新增一个独立的感知扩展包，所有感知相关代码都放在：

```text
src/perceptive_legged_control/perceptive_legged_control
```

当前实现遵循“只新增，不修改原有 legged_control/elev_mpc 代码”的原则。

### 已添加内容

- 新增 `perceptive_target_trajectories_publisher`。
- 新增 `perceptive_stack.launch.py`。
- 新增 `perceptive_target.yaml`。
- 复用 `convex_plane_decomposition_ros_world_box_terrain`，不再使用图片贴图高程图。

### 当前感知链路

```text
Gazebo world box collision
  -> convex_plane_decomposition_ros_world_box_terrain
  -> /convex_plane_decomposition_ros/planar_terrain
  -> perceptive_target_trajectories_publisher
  -> /legged_robot_mpc_target
```

`world_box_terrain` 负责从 Gazebo world 中的 box collision 预构建高程图，按机器人 `base` 附近截取局部 submap，并发布凸平面分割后的 `PlanarTerrain`。

`perceptive_target_trajectories_publisher` 订阅 `PlanarTerrain`，使用 `smooth_planar` 层优先、`elevation` 层兜底，为 `/cmd_vel` 和 `/move_base_simple/goal` 生成带地形高度适配的 OCS2 target trajectories。

### 验证

```bash
colcon build --packages-select perceptive_legged_control --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
ros2 launch perceptive_legged_control perceptive_stack.launch.py --show-args
```

两项均已通过。

### 当前边界

目前完成的是 nacl 链路中的：

```text
PlanarTerrain -> terrain-adapted TargetTrajectories
```

还没有把 terrain model 直接注入现有 `LeggedController` 的 MPC solver 内部。原因是当前 ROS 2 `legged_control` 使用的是 `ocs2_legged_robot` 接口，而 nacl 中的 `TerrainReceiverSynchronizedModule` 基于 perceptive anymal / switched_model 接口，不能直接无缝复用。

后续如果要继续补齐 nacl 逻辑，需要新增兼容当前 ROS 2 `legged_interface` 的 perceptive controller 或 terrain synchronized module。

## 2026-06-07 10:27:30 UTC

继续移植 nacl 的感知 MPC 逻辑，新增一个不修改原 `legged_control` 的 controller 插件：

```text
perceptive_legged_control/PerceptiveLeggedController
```

### 已添加内容

- 新增 `PerceptiveLeggedController`，继承原 `legged::LeggedController`，只替换 `setupLeggedInterface()`。
- 新增 `PerceptiveLeggedInterface`，继承原 `legged::LeggedInterface`，只替换 `setupReferenceManager()`。
- 新增 `PerceptiveSwitchedModelReferenceManager`，继承原 `ocs2::legged_robot::SwitchedModelReferenceManager`。
- 新增 `PerceptiveTerrainBuffer`，订阅 `/convex_plane_decomposition_ros/planar_terrain` 并缓存最新地形。
- 新增 `perceptive_legged_control_plugins.xml`，导出 controller plugin。
- 新增 `config/perceptive_p1_controllers.yaml`，作为不改原 controller 配置的感知 controller 样例。

### 当前 MPC 感知链路

```text
convex_plane_decomposition_ros_world_box_terrain
  -> /convex_plane_decomposition_ros/planar_terrain
  -> PerceptiveTerrainBuffer
  -> PerceptiveSwitchedModelReferenceManager::modifyReferences()
  -> SwingTrajectoryPlanner::update(modeSchedule, terrainHeight)
  -> OCS2 MPC swing foot z constraint
```

原 ROS 2 `SwitchedModelReferenceManager` 每次使用 `terrainHeight = 0.0` 更新摆腿轨迹。本次新增的感知版 reference manager 保留原来的 gait schedule 时间窗口逻辑，只把该高度替换为机器人当前位置在 `PlanarTerrain.gridMap` 上的高度。

高度层读取规则：优先 `smooth_planar`，不存在时回退到 `elevation`；如果还没有收到 terrain、位置在地图外、或高度无效，则回退到当前 base z。

### 使用方式

在 controller manager 配置中把 controller type 切到：

```yaml
perceptive_legged_controller:
  type: perceptive_legged_control/PerceptiveLeggedController
```

并保留原来的 `urdfFile`、`taskFile`、`referenceFile` 参数。可参考：

```text
src/perceptive_legged_control/perceptive_legged_control/config/perceptive_p1_controllers.yaml
```

### 验证

```bash
colcon build --packages-select perceptive_legged_control --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
ros2 launch perceptive_legged_control perceptive_stack.launch.py --show-args
```

构建已通过，`perceptive_stack.launch.py --show-args` 已通过。安装目录中已确认存在：

```text
install/perceptive_legged_control/lib/libperceptive_legged_controller.so
install/perceptive_legged_control/share/perceptive_legged_control/perceptive_legged_control_plugins.xml
```

### 当前边界

这一步已经把感知地形高度接入当前 ROS 2 `ocs2_legged_robot` 的 swing trajectory planner，逻辑上对应 nacl 中把局部平面地形送入 MPC 的高度约束部分。当前还没有移植 nacl/perceptive anymal 的完整 terrain plane normal、区域投影和 foothold 级别的 `TerrainModel` 逻辑，因为当前 ROS 2 `legged_interface` 没有同名 terrain model 接口；后续可以继续在新包中扩展，不改原控制器。

## 2026-06-07 14:03:53 UTC

根据“要和 nacl 对齐，不要做牺牲性的适配”的要求，本次把上一版只注入单一地形高度的实现替换为 nacl 结构对齐版本。

### 本次对齐 nacl 的模块

新增/改造为以下对应关系：

```text
nacl legged_perceptive_controllers/synchronized_module/PlanarTerrainReceiver
  -> perceptive_legged_control/PlanarTerrainReceiver

nacl legged_perceptive_interface/ConvexRegionSelector
  -> perceptive_legged_control/ConvexRegionSelector

nacl legged_perceptive_interface/PerceptiveLeggedReferenceManager
  -> perceptive_legged_control/PerceptiveSwitchedModelReferenceManager

nacl legged_perceptive_interface/PerceptiveLeggedInterface
  -> perceptive_legged_control/PerceptiveLeggedInterface

nacl legged_perceptive_controllers/PerceptiveController::setupMpc()
  -> perceptive_legged_control/PerceptiveLeggedController::setupMpc()
```

### 当前对齐后的 MPC 链路

```text
convex_plane_decomposition_ros_world_box_terrain
  -> /convex_plane_decomposition_ros/planar_terrain
  -> PlanarTerrainReceiver::planarTerrainCallback()
  -> PlanarTerrainReceiver::preSolverRun()
  -> shared PlanarTerrain
  -> PerceptiveSwitchedModelReferenceManager::modifyReferences()
  -> ConvexRegionSelector::update()
  -> SwingTrajectoryPlanner::update(liftOff, touchDown, maxHeight)
```

现在不再只是把 `terrainHeight` 从 `0.0` 改成局部高度，而是按 nacl 的逻辑执行：

- 用 `PlanarTerrainReceiver` 在 solver 同步点更新共享地形。
- 用 `modifyReferences()` 按 `smooth_planar` 调整 base z 和 base pitch。
- 用 `ConvexRegionSelector` 为每条腿、每个支撑相计算 nominal foothold。
- 把 nominal foothold 投影到最佳 planar region。
- 在 projected planar region 内做 convex polygon growing。
- 用投影结果生成 lift-off height、touch-down height、swing max height 三组序列。
- 用三组序列更新当前 ROS 2 `legged_interface` 里的 `SwingTrajectoryPlanner`。

### 保留的 nacl 语义

- 初始 fallback terrain 是 5m x 5m 平面，包含 `elevation` 和 `smooth_planar` 层。
- `numVertices` 默认 16，可通过 controller 参数 `perceptive_num_vertices` 修改。
- `perceptive_terrain_topic` 默认 `/convex_plane_decomposition_ros/planar_terrain`。
- `perceptive_sdf_elevation_layer` 默认 `elevation`，用于 receiver 对 NaN 高程做 inpainting。
- 摆腿最高点沿 lift-off 到 touch-down 的地图连线取最高高程，并乘以 `1.05`，与 nacl 中 yxy 增加的逻辑保持一致。

### 当前仍未完成的完整 nacl 约束

nacl 中还包含 `FootPlacementConstraint`、`FootPlacementConstraintCBF`、`SwingFootPlacementConstraintCBF`、`FootCollisionConstraint`、`SphereSdfConstraint`、`PerceptiveLeggedPrecomputation`、`FootPlacementVisualization`、`SphereVisualization` 等模块。

当前 ROS 2 工作区没有 nacl 使用的 `grid_map_sdf`、`ocs2_sphere_approximation` 以及这些 legged_perceptive constraint/precomputation 类。因此这次没有把它们降级为近似实现，而是先补齐能在当前 ROS 2 `legged_interface` 中真实接入的 nacl 核心链路。后续要继续完全对齐，应在新包中继续仿照 nacl 创建这些 constraint/precomputation/visualization 类，而不是改原始 `legged_control`。

### 验证

```bash
colcon build --packages-select perceptive_legged_control --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
ros2 launch perceptive_legged_control perceptive_stack.launch.py --show-args
```

两项均已通过。插件共享库和 plugin XML 已确认安装：

```text
install/perceptive_legged_control/lib/libperceptive_legged_controller.so
install/perceptive_legged_control/share/perceptive_legged_control/perceptive_legged_control_plugins.xml
```

## 2026-06-07 14:15:03 UTC

补充参考 `/workspace/legged_perceptive` 项目的逻辑和结构，用于后续继续重构/迁移 nacl 感知 MPC 约束。

### legged_perceptive 的结构结论

`legged_perceptive` 相比 nacl 中的大 workspace 更像一个整理后的最小感知控制项目，主要拆成：

```text
legged_perceptive_interface
  -> ConvexRegionSelector
  -> PerceptiveLeggedReferenceManager
  -> PerceptiveLeggedPrecomputation
  -> PerceptiveLeggedInterface
  -> constraint/FootPlacementConstraint
  -> constraint/FootCollisionConstraint
  -> constraint/SphereSdfConstraint

legged_perceptive_controllers
  -> PerceptiveController
  -> synchronized_module/PlanarTerrainReceiver
  -> visualization/FootPlacementVisualization
  -> visualization/SphereVisualization
```

### 和 nacl 的差异

`legged_perceptive` 这一版没有接入 nacl 后续新增的 `FootPlacementConstraintCBF` 和 `SwingFootPlacementConstraintCBF`，也没有把 swing max height 沿路径最高点的逻辑加进 reference manager；这些更接近 nacl 中较新的实现。

但它的包结构更清楚，后续 ROS 2 新包可以参考这种结构，在当前 `perceptive_legged_control` 包内部按以下目录继续整理：

```text
include/perceptive_legged_control/interface/
include/perceptive_legged_control/constraint/
include/perceptive_legged_control/synchronized_module/
include/perceptive_legged_control/visualization/
src/interface/
src/constraint/
src/synchronized_module/
src/visualization/
```

### 后续迁移建议

后续继续迁 `PerceptiveLeggedPrecomputation`、`FootPlacementConstraint`、`FootCollisionConstraint` 时，优先参考 `legged_perceptive` 的清晰包结构；迁 `FootPlacementConstraintCBF`、`SwingFootPlacementConstraintCBF` 和 swing max-height 逻辑时，继续以 nacl 中较新的实现为准。

本次只是补充结构参考和迁移判断，没有修改功能代码。

## 2026-06-07 14:39:17 UTC

按前一轮计划继续迁移，完成第一阶段结构重构，并迁入 nacl/legged_perceptive 中实际使用的 `PerceptiveLeggedPrecomputation` 和 `FootPlacementConstraint`。

### 目录重构

把感知 controller 相关代码从平铺结构整理为类似 `legged_perceptive` 的分层结构：

```text
include/perceptive_legged_control/controller/
include/perceptive_legged_control/interface/
include/perceptive_legged_control/constraint/
include/perceptive_legged_control/synchronized_module/

src/controller/
src/interface/
src/constraint/
src/synchronized_module/
```

当前主要文件位置：

```text
controller/PerceptiveLeggedController
interface/PerceptiveLeggedInterface
interface/PerceptiveSwitchedModelReferenceManager
interface/ConvexRegionSelector
interface/PerceptiveLeggedPrecomputation
constraint/FootPlacementConstraint
synchronized_module/PlanarTerrainReceiver
```

### 新增迁移内容

新增 `PerceptiveLeggedPrecomputation`，参考 `legged_perceptive/legged_perceptive_interface`：

- 继承当前 ROS 2 `ocs2::legged_robot::LeggedRobotPreComputation`。
- 在 `request()` 中根据 `ConvexRegionSelector` 的凸多边形生成 `A * foot_position + b` 约束参数。
- 向 `FootPlacementConstraint` 提供每条腿的多边形半空间参数。

新增 `FootPlacementConstraint`：

- 继承 `ocs2::StateConstraint`。
- 激活条件来自 `PerceptiveSwitchedModelReferenceManager::getFootPlacementFlags()`。
- 约束值使用 `PerceptiveLeggedPrecomputation` 缓存的 convex polygon half-space。

### 接入点

`PerceptiveLeggedInterface::setupPreComputation()` 现在用：

```text
PerceptiveLeggedPrecomputation
```

替换原来的 `LeggedRobotPreComputation`。

`PerceptiveLeggedInterface::setupOptimalControlProblem()` 现在会为每个 3DoF foot 添加：

```text
<foot>_footPlacement
```

作为 `StateSoftConstraint`。

### 当前状态

这一步完成的是默认实际使用链路中的：

```text
PerceptiveLeggedPrecomputation + FootPlacementConstraint
```

暂时还未迁入：

```text
FootPlacementConstraintCBF
FootCollisionConstraint
SwingFootPlacementConstraintCBF
SphereSdfConstraint
```

下一步建议优先迁 `FootPlacementConstraintCBF` 和 `FootCollisionConstraint`，因为 nacl/aliengo 默认启用这两个相关功能。

### 验证

```bash
colcon build --packages-select perceptive_legged_control --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
ros2 launch perceptive_legged_control perceptive_stack.launch.py --show-args
```

两项均已通过。插件库和 plugin XML 仍正常安装：

```text
install/perceptive_legged_control/lib/libperceptive_legged_controller.so
install/perceptive_legged_control/share/perceptive_legged_control/perceptive_legged_control_plugins.xml
```

## 2026-06-07 14:47:29 UTC

继续迁移 nacl/aliengo 默认启用的感知约束，完成 `FootPlacementConstraintCBF` 和 `FootCollisionConstraint` 接入。

### 新增内容

新增 `constraint/FootPlacementConstraintCBF`：

- 参考 nacl 的 `FootPlacementConstraintCBF`。
- 继承 `ocs2::StateInputConstraint`。
- 使用 `PerceptiveLeggedPrecomputation` 提供的 convex polygon half-space 参数。
- 约束形式保持 nacl 逻辑：

```text
dot(h) + lambda * h
```

其中 `h = A * foot_position + b`，`dot(h) = A * foot_velocity`。

新增 `constraint/FootCollisionConstraint`：

- 参考 `legged_perceptive` / nacl 的 `FootCollisionConstraint`。
- 继承 `ocs2::StateConstraint`。
- 只在摆动中段激活，和 nacl 的 `time +/- offset` 判断一致。
- 约束值为：

```text
signed_distance(foot_position) - clearance
```

新增 `interface/PlanarSignedDistanceField`：

- 当前 ROS 2 工作区没有 nacl 使用的 `grid_map_sdf` 包，因此在新包内仿照创建最小可用 SDF。
- 使用 `GridMap` 的高程层构造近似 signed distance：`z - terrain_height(x, y)`。
- 使用有限差分估计 terrain gradient，给 `FootCollisionConstraint` 提供线性化方向。
- `PlanarTerrainReceiver::preSolverRun()` 在同步更新 `PlanarTerrain` 时同步更新 SDF。

### 接入点

`PerceptiveLeggedInterface::setupOptimalControlProblem()` 现在每个 foot 都会添加：

```text
<foot>_footPlacement
<foot>_footPlacementCBF
<foot>_footCollision
```

当前 penalty/lambda 先采用 nacl/aliengo 默认值：

```text
FootPlacementConstraint      mu=1e-2, delta=1e-4
FootPlacementConstraintCBF   lambda=0.5, mu=1e-2, delta=0.005
FootCollisionConstraint      clearance=0.03, mu=1e-2, delta=0.005
```

后续还需要把这些参数改成从 `task.info` 读取，以完全对齐 nacl 的配置开关。

### 验证

```bash
colcon build --packages-select perceptive_legged_control --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
ros2 launch perceptive_legged_control perceptive_stack.launch.py --show-args
```

两项均已通过。插件库和 plugin XML 仍正常安装。

### 后续

下一步建议迁移 `SwingFootPlacementConstraintCBF`，但保持默认关闭；再评估 `SphereSdfConstraint`，它还需要仿照 nacl 的 sphere approximation/kinematics 依赖。

## 2026-06-07 14:50:06 UTC

补充对照 nacl controller 层文件：

```text
/workspace/nacl/perceptive_legged_control/OCS2_ws/src/legged_perceptive/legged_perceptive_controllers/src/PerceptiveController.cpp
```

该文件确认 nacl 的 controller 层包含三部分：

```text
setupLeggedInterface()
  -> 创建 PerceptiveLeggedInterface
  -> setupOptimalControlProblem()
  -> setupVisualization()

setupMpc()
  -> 先调用 LeggedController::setupMpc()
  -> 创建 PlanarTerrainReceiver
  -> addSynchronizedModule(planarTerrainReceiver)

update()
  -> 先调用 LeggedController::update()
  -> footPlacementVisualizationPtr_->update(currentObservation_)
  -> sphereVisualizationPtr_->update(currentObservation_)
```

当前 ROS 2 新包已经对齐了：

```text
setupLeggedInterface()
setupMpc() + PlanarTerrainReceiver synchronized module
```

尚未对齐：

```text
setupVisualization()
FootPlacementVisualization
SphereVisualization
PerceptiveLeggedController::update() 中的 visualization update
```

后续如果继续对齐 controller 层，应优先迁 `FootPlacementVisualization`；`SphereVisualization` 依赖 sphere approximation，可等 `SphereSdfConstraint` 一起处理。

## 2026-06-07 15:00:02 UTC

继续对齐 nacl/legged_perceptive 的 controller 层，迁入 `FootPlacementVisualization`。

### 新增内容

新增 ROS 2 版：

```text
include/perceptive_legged_control/visualization/FootPlacementVisualization.h
src/visualization/FootPlacementVisualization.cpp
```

该类参考 `legged_perceptive_controllers/visualization/FootPlacementVisualization`，发布：

```text
/foot_placement  (visualization_msgs/msg/MarkerArray)
```

可视化内容包括：

- `Projections`：落足点在平面上的投影方向箭头。
- `Convex Regions`：每条腿未来支撑相对应的凸落足区域。
- `Nominal Footholds`：nominal foothold marker。

### Controller 接入

`PerceptiveLeggedController` 现在新增：

```text
setupVisualization()
update()
```

对齐 nacl 中：

```text
setupVisualization()
footPlacementVisualizationPtr_->update(currentObservation_)
```

当前 ROS 2 代码现在会在 `setupLeggedInterface()` 后创建 `FootPlacementVisualization`，并在 controller `update()` 成功后刷新 marker。

因此 `PerceptiveLeggedController.cpp` 现在直接 include：

```text
perceptive_legged_control/interface/PerceptiveSwitchedModelReferenceManager.h
```

用途和 nacl 的 `PerceptiveLeggedReferenceManager.h` 一致：从 reference manager 中取 `ConvexRegionSelector`，传给 foot placement visualization。

### 暂未迁移

`SphereVisualization` 暂未迁移。原因是它依赖 nacl/legged_perceptive 的 `PinocchioSphereInterface` / `ocs2_sphere_approximation`。当前 ROS 2 新包里还没有仿照创建 sphere approximation 层；后续应和 `SphereSdfConstraint` 一起处理。

### 验证

```bash
colcon build --packages-select perceptive_legged_control --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
ros2 launch perceptive_legged_control perceptive_stack.launch.py --show-args
```

两项均已通过。插件库和 plugin XML 仍正常安装。

## 2026-06-07 15:07:04 UTC

继续对齐 nacl 的 swing foot placement CBF 逻辑，迁入默认未启用的 `SwingFootPlacementConstraintCBF`。

### 新增内容

新增 ROS 2 版：

```text
include/perceptive_legged_control/constraint/SwingFootPlacementConstraintCBF.h
src/constraint/SwingFootPlacementConstraintCBF.cpp
```

该类参考 nacl/`legged_perceptive_interface/constraint/SwingFootPlacementConstraintCBF`，在摆动相中使用当前 foot placement convex polygon 参数，并按 swing phase penalty 扩展边界约束。

### Reference Manager 对齐

`PerceptiveSwitchedModelReferenceManager` 新增和 nacl 同名的接口：

```text
getSwingFootPlacementFlags(time)
ContactPhasePerLeg(time)
SwingPhasePerLeg(time)
```

其中 `ContactPhasePerLeg` 和 `SwingPhasePerLeg` 直接使用当前工作区已有的 `ocs2_legged_robot/gait/LegLogic.h` 工具函数，避免修改原始 `legged_interface`。

### 接入状态

当前 `SwingFootPlacementConstraintCBF` 已加入 `perceptive_legged_controller` 编译，但没有添加到 `problemPtr_` 的 soft constraints 中，因此不会改变现有控制行为。

后续如果要完全按 nacl 打开它，应在 `PerceptiveLeggedInterface::setupOptimalControlProblem()` 中增加配置开关读取，再按 nacl 的 `enableSwingFootPlacementConstraintCBF` 逻辑接入；不建议无开关直接启用。

### 验证

```bash
colcon build --packages-select perceptive_legged_control --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
ros2 launch perceptive_legged_control perceptive_stack.launch.py --show-args
```

两项均已通过。

## 2026-06-07 15:16:13 UTC

继续对齐 nacl/`legged_perceptive` 的 sphere collision 链路，新增 `SphereSdfConstraint` 和 `SphereVisualization` 的 ROS 2 版本。

### 新增内容

新增文件：

```text
include/perceptive_legged_control/constraint/SphereSdfConstraint.h
src/constraint/SphereSdfConstraint.cpp
include/perceptive_legged_control/visualization/SphereVisualization.h
src/visualization/SphereVisualization.cpp
```

`SphereSdfConstraint` 参考 nacl 的 `legged_perceptive_interface/constraint/SphereSdfConstraint`，逻辑为：

```text
sphere center world position -> signed distance field value - sphere radius
```

`SphereVisualization` 参考 nacl 的 `legged_perceptive_controllers/visualization/SphereVisualization`，发布：

```text
/sphere_markers  (visualization_msgs/msg/MarkerArray)
```

### Interface / Controller 对齐

`PerceptiveLeggedInterface` 已补入 nacl 同类接口：

```text
getPinocchioSphereInterfacePtr()
```

并按 nacl 的默认逻辑创建 collision sphere interface：

```text
LF_calf, RF_calf, LH_calf, RH_calf
maxExcess = 0.02
shrinkRatio = 0.6
```

这些值在 ROS 2 新包里通过 controller 参数可覆盖：

```text
perceptive_collision_links
perceptive_collision_max_excesses
perceptive_sphere_shrink_ratio
perceptive_enable_sphere_sdf_constraint
```

`PerceptiveLeggedController` 中也按 nacl 的 controller 层逻辑接入了 sphere visualization 的创建和 `update()` 刷新。

### 当前构建策略

当前工作区中存在源码包：

```text
src/ocs2/ocs2_pinocchio/ocs2_sphere_approximation
```

但该包在当前环境里不能直接构建，原因是它的 `PinocchioSphereInterface.cpp` 把 `urdf::exportURDF()` 返回的 `TiXmlDocument*` 当成 `tinyxml2::XMLDocument*` 使用，和当前系统的 urdfdom/tinyxml 版本不匹配。

为了遵守“不修改原本代码”的要求，本次没有改 `src/ocs2`。新包中 sphere 相关代码保留，并通过 CMake 做成可选编译：只有当 `ocs2_sphere_approximation` 已经可被 `find_package()` 找到时，才会编译并启用：

```text
SphereSdfConstraint
SphereVisualization
PERCEPTIVE_HAS_SPHERE_APPROXIMATION
```

当前环境下该依赖不可用，因此新包会跳过 sphere 编译路径，但文件和接入点已经就位。

### 验证

```bash
colcon build --packages-select perceptive_legged_control --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
ros2 launch perceptive_legged_control perceptive_stack.launch.py --show-args
```

两项均已通过。

备注：由于尝试构建 `ocs2_sphere_approximation` 失败，当前 `source install/setup.bash` 会提示一次缺失：

```text
/workspace/install/ocs2_sphere_approximation/share/ocs2_sphere_approximation/local_setup.bash
```

这是失败构建留下的 install 环境提示，不影响 `perceptive_legged_control` 当前构建和 launch 参数解析。后续若要真正启用 sphere 约束，应先修复或重建 `ocs2_sphere_approximation`，再打开 `perceptive_enable_sphere_sdf_constraint`。

## 2026-06-07 15:25:41 UTC

继续对齐 nacl 的 `PerceptiveLeggedInterface::setupOptimalControlProblem()`，把感知约束从硬编码添加改为 task.info 风格配置开关控制。

### 新增配置读取

`PerceptiveLeggedInterface` 新增三个 nacl 同名逻辑的配置读取函数：

```text
loadFootPlacementSettings(taskFile, verbose)
loadSwingFootPlacementSettings(taskFile, verbose)
loadCollisionSettings(taskFile, verbose)
```

读取字段与 nacl 对齐：

```text
footPlacement.enable
footPlacement.enableCBF
footPlacement.cbfLambda
footPlacement.cbfPenaltyMuParam
footPlacement.cbfPenaltyDeltaParam
footPlacement.statePenaltyMuParam
footPlacement.statePenaltyDeltaParam
footPlacement.enableSwingCBF
footPlacement.swingcbfLambda
footPlacement.swingcbfPenaltyMuParam
footPlacement.swingcbfPenaltyDeltaParam
footPlacement.swingPenaltyMuParam
footPlacement.swingPenaltyDeltaParam
CollisionParam.enablefootCollision
CollisionParam.enableSphereSdfConstraint
CollisionParam.footCollisionPenaltyMu
CollisionParam.footCollisionPenaltyDelta
CollisionParam.SphereSdfConstraintPenaltyMu
CollisionParam.SphereSdfConstraintPenaltyDelta
```

### 约束接入变化

现在以下约束按配置开关添加：

```text
FootPlacementConstraint              -> footPlacement.enable
FootPlacementConstraintCBF           -> footPlacement.enableCBF
SwingFootPlacementConstraintCBF      -> footPlacement.enableSwingCBF
FootCollisionConstraint              -> CollisionParam.enablefootCollision
SphereSdfConstraint                  -> CollisionParam.enableSphereSdfConstraint + ocs2_sphere_approximation 可用
```

其中 `SwingFootPlacementConstraintCBF` 这次正式接入 `problemPtr_->softConstraintPtr`，但受配置开关控制，默认值按 nacl 思路为启用。

### 默认值

如果 task file 里没有这些字段，新包会使用保守默认值：

```text
footPlacement.enable                 true
footPlacement.enableCBF              true
footPlacement.enableSwingCBF         true
CollisionParam.enablefootCollision   true
CollisionParam.enableSphereSdfConstraint false
```

sphere SDF 仍保持默认关闭，这与 nacl/legged_perceptive 的实际配置一致。

### 验证

```bash
colcon build --packages-select perceptive_legged_control --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
ros2 launch perceptive_legged_control perceptive_stack.launch.py --show-args
```

两项均已通过。

## 2026-06-07 15:30:55 UTC

按要求将感知专用的 P1 task 配置放到 `perceptive_legged_control` 包内，避免修改原始 `legged_control` 的 P1 配置。

### 新增配置文件

新增：

```text
config/p1/task.info
```

该文件以当前 P1 基础 task 为底本，并在末尾追加感知 MPC 相关配置：

```text
footPlacement
CollisionParam
```

其中 `footPlacement` 控制：

```text
FootPlacementConstraint
FootPlacementConstraintCBF
SwingFootPlacementConstraintCBF
```

`CollisionParam` 控制：

```text
FootCollisionConstraint
SphereSdfConstraint
```

`enableSphereSdfConstraint` 仍保持 `false`，与 nacl/legged_perceptive 的默认实际使用状态一致。

### 配置读取路径

已把感知相关配置改为读取新建文件：

```text
/workspace/src/perceptive_legged_control/perceptive_legged_control/config/p1/task.info
```

修改位置：

```text
config/perceptive_target.yaml
config/perceptive_p1_controllers.yaml
```

原始文件未修改：

```text
src/legged_control/legged_controllers/config/p1/task.info
```

### 验证

```bash
colcon build --packages-select perceptive_legged_control --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
ros2 launch perceptive_legged_control perceptive_stack.launch.py --show-args
```

两项均已通过。

## 2026-06-08 02:19:26 UTC

切换 `PlanarSignedDistanceField` 的底层实现，从包内高度差近似改为已安装的 ROS Humble `grid_map_sdf`。

### 修改内容

- `PlanarSignedDistanceField` 保留原有外部接口，内部新增 `grid_map::SignedDistanceField`。
- `update()` 中继续保留 elevation layer fallback 和 NaN inpainting，再用正值 `heightClearance` 调用 `calculateSignedDistanceField()` 重建 SDF。
- `value()` 改为转发 `grid_map_sdf::SignedDistanceField::getDistanceAt()`。
- `derivative()` 改为转发 `grid_map_sdf::SignedDistanceField::getDistanceGradientAt()`。
- `CMakeLists.txt` 和 `package.xml` 新增 `grid_map_sdf` 依赖。
- `CMakeLists.txt` 补充 PCL include/link 配置，用于满足 `grid_map_sdf` 头文件中的 PCL 依赖。

### 影响

这不是牺牲性适配：约束类仍通过原 `PlanarSignedDistanceField` 接口调用，但底层距离和梯度改为真实 grid map SDF，而不是 `z - elevation(x, y)` 的竖直高度差近似。

需要注意的是 ROS Humble 的 `grid_map_sdf` API 与 nacl 原版 API 不完全一致，因此这里做的是版本兼容包装，不改变外部约束接入方式。

### 验证

已通过：

```bash
colcon build --packages-select perceptive_legged_control --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
```


## 2026-06-08 02:37:25 UTC

补充 P1 仿真验证入口，按当前工程实际使用的预构建高程图链路接入感知 controller。

### 修改内容

- `legged_gazebo/launch/p1_sim.launch.py` 新增 `legged_controller_type` launch 参数，默认仍为 `legged/LeggedController`，不改变原 P1 仿真启动行为。
- 新增 `p1_dds_joy_tools/launch/p1_perceptive_gait_bridge_sim_test.launch.py`，复用原 P1 DDS/手柄 gait bridge 链路。
- 新 launch 启动 `convex_plane_decomposition_ros/launch/world_box_terrain.launch.py`，使用预构建 world box 高程图发布 `/convex_plane_decomposition_ros/planar_terrain`。
- 新 launch 将 `legged_controller` 的 type 切为 `perceptive_legged_control/PerceptiveLeggedController`，controller 名称仍保持 `legged_controller`，避免影响现有 DDS bridge topic/controller 假设。
- 新 launch 使用 `perceptive_target_trajectories_publisher` 替代原 `legged_target_trajectories_publisher`，避免同一 `/legged_robot_mpc_target` 上出现两个 target publisher。

### 验证命令

已通过：

```bash
colcon build --packages-select legged_gazebo p1_dds_joy_tools perceptive_legged_control --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
ros2 launch p1_dds_joy_tools p1_perceptive_gait_bridge_sim_test.launch.py --show-args
```

### 运行验证入口

后续可用下面命令启动完整 P1 感知仿真验证：

```bash
ros2 launch p1_dds_joy_tools p1_perceptive_gait_bridge_sim_test.launch.py
```

重点检查 `/convex_plane_decomposition_ros/planar_terrain` 是否发布、`PlanarTerrainReceiver` 是否订阅成功、MPC 是否持续求解，以及切换到 `grid_map_sdf` 后足端 clearance 是否稳定。


## 2026-06-08 02:44:40 UTC

调整 P1 感知仿真入口的参考高度，默认使用站立参考而不是趴下参考。

### 修改内容

- `p1_perceptive_gait_bridge_sim_test.launch.py` 中新增内部 `controller_reference_file` 选择逻辑。
- 默认情况下，感知 controller / MPC 的 `reference_file` 使用 `stand_reference_file`，即 `legged_controllers/config/p1/reference.info`。
- `perceptive_target_trajectories_publisher` 已保持使用 `stand_reference_file`。
- `lie_down_reference_file` 仍保留给 `lie_down_gait_id` 触发时使用，不影响趴下指令。
- 若后续需要临时覆盖 controller 参考文件，仍可通过 launch 参数 `reference_file:=...` 显式指定。

### 验证

已通过：

```bash
colcon build --packages-select p1_dds_joy_tools --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
ros2 launch p1_dds_joy_tools p1_perceptive_gait_bridge_sim_test.launch.py --show-args
```


## 2026-06-08 02:56:00 UTC

修正 P1 感知仿真入口的默认参考流程，恢复“先趴下启动，再通过 gait bridge 进入站立和其他步态”的行为。

### 背景

上一版把 `p1_perceptive_gait_bridge_sim_test.launch.py` 中空的 `reference_file` 默认解析为 `stand_reference_file`。这样会让 controller/MPC 一启动就使用站立参考高度，不再默认进入趴下姿态，和 P1 现有 DDS gait bridge 流程不一致。

### 修改内容

- `controller_reference_file` 空值默认改回 `lie_down_reference_file`。
- `p1_sim.launch.py` 中的感知 controller 启动参考仍默认使用 `reference_lie_down.info`。
- `p1_gait_dds_bridge` 的 `mpc_reference_file` 同样默认使用趴下参考，保持和 controller 初始状态一致。
- `stand_reference_file` 仍用于 `stand_gait_id` 触发站立目标。
- `perceptive_target_trajectories_publisher` 继续使用站立参考生成目标轨迹；感知 terrain/SDF 从启动时在线，切换到站立和后续步态时生效。
- 如需临时强制 controller 直接使用其他参考，仍可显式传 `reference_file:=...` 覆盖默认行为。

### 验证

已通过：

```bash
colcon build --packages-select p1_dds_joy_tools --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
ros2 launch p1_dds_joy_tools p1_perceptive_gait_bridge_sim_test.launch.py --show-args
```


## 2026-06-08 03:11:58 UTC

修正切换站立时高度不变的问题。

### 问题原因

`PerceptiveSwitchedModelReferenceManager::modifyReferences()` 之前会把所有 target trajectory 的 base z 强制改成：

```text
terrainZ + comHeight_
```

其中 `comHeight_` 来自 controller 启动时的 `reference_file`。P1 感知仿真默认用 `reference_lie_down.info` 启动，因此这里读到的是趴下低高度。结果是 DDS bridge 发布的 stand target 虽然使用了 `reference.info` 的站立高度，随后仍会被感知 reference manager 覆盖回低高度。

### 修改内容

- `modifyReferences()` 不再用启动 reference 的 `comHeight_` 覆盖目标高度。
- 现在保留 target trajectory 自身给出的 base z，并叠加当前 terrain 高度：

```text
adapted_base_z = terrainZ + target_base_z
```

这样 `lie_down` 仍保持低参考高度，`stand` 使用 `reference.info` 的站立高度，后续其他步态也保留各自目标高度，只额外接入 terrain 高度修正。

### 验证

已通过：

```bash
colcon build --packages-select perceptive_legged_control --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
```

### 运行时检查

再次切站立时，重点看：

```bash
ros2 topic echo /legged_robot_mpc_target --once
```

以及日志中的：

```text
Published stand posture target: height=...
```

若 target topic 中 stand target 已经是高 z，但机器人仍不升高，则下一步应检查 controller 是否收到并跟踪该 target。


## 2026-06-08 03:26:19 UTC

修正上楼梯时 `/legged_robot/optimizedStateTrajectory` 中 base 轨迹异常的问题。

### 问题原因

上一版修正站立高度时，让 `PerceptiveSwitchedModelReferenceManager` 在 MPC 内统一执行：

```text
adapted_base_z = terrainZ + target_base_z
```

但 `perceptive_target_trajectories_publisher` 在 `/cmd_vel` 和 `/move_base_simple/goal` 生成 target 时，已经提前把 `target_base_z` 写成 `terrainHeight + comHeight`。因此上楼梯时 terrain height 会被重复叠加；平地上 `terrainZ = 0`，所以这个问题不明显。

### 修改内容

- `perceptive_target_trajectories_publisher` 输出的 target base z 改为相对地形高度。
- 当前 state 写入 target trajectory 前，先减去当前位置 terrain height。
- `/cmd_vel` 目标保持当前相对地形高度，不再提前加目标点 terrain height。
- `/move_base_simple/goal` 目标使用 `goal.pose.position.z + comHeight` 作为相对高度。
- terrain height 叠加统一由 `PerceptiveSwitchedModelReferenceManager` 在 MPC 内完成。

### 预期效果

- 平地行为不变。
- 楼梯/台阶上不再重复叠加 terrain height。
- `lie_down`、`stand` 和后续步态仍保持各自 reference/target 给出的相对 base 高度。

### 验证

已通过：

```bash
colcon build --packages-select perceptive_legged_control --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
```


## 2026-06-08 07:15:00 UTC

修正 P1 感知仿真中 Gazebo 与 RViz 机器人/楼梯位置不一致的问题。

### 问题原因

`p1_perceptive_gait_bridge_sim_test.launch.py` 默认使用 `PerceptiveLeggedController`，其状态估计为 `KalmanFilterEstimate`（假设足端高度为平地 z=0）。仿真中 Gazebo 通过 `/ground_truth/state` 发布真实位姿，两者在接近楼梯时会产生 xy/z 偏差。

`convex_plane_decomposition_ros_world_box_terrain` 的 submap 中心来自 TF `odom -> base`（与 MPC 观测一致），而 Gazebo 显示的是物理真值。因此 RViz 中机器人、地形图、MPC 轨迹与 Gazebo 中的机器人和楼梯会出现空间错位；同时 MPC 会在尚未到达楼梯的 xy 位置提前采样台阶高度，表现为「狗还在地面，轨迹已在楼梯上」。

### 修改内容

- 新增 `PerceptiveLeggedCheaterController`，仿真时订阅 `/ground_truth/state` 作为状态估计（与 `legged/LeggedCheaterController` 一致）。
- 在 `perceptive_legged_control_plugins.xml` 注册新插件。
- `p1_perceptive_gait_bridge_sim_test.launch.py` 默认 `legged_controller_type` 改为 `perceptive_legged_control/PerceptiveLeggedCheaterController`。
- 新增 launch 参数 `legged_controller_type`，实机部署可改回 `PerceptiveLeggedController`。

### 验证

已通过：

```bash
colcon build --packages-select perceptive_legged_control p1_dds_joy_tools --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
```

### 使用说明

重新 source 后启动：

```bash
ros2 launch p1_dds_joy_tools p1_perceptive_gait_bridge_sim_test.launch.py
```

RViz Fixed Frame 请保持 `odom`。若仍看到 MPC 轨迹略超前于机体（cmd_vel 1 s 前瞻），属正常前瞻行为；若整体空间仍错位，检查 `/ground_truth/state` 是否在发布。


## 2026-06-08 08:05:00 UTC

修正爬楼梯 5~6 节后 MPC 规划过期、机体发散摔倒的问题。

### 问题原因

终端日志显示：

```text
The requested currentTime is greater than the received plan: 75.57>59.17
[SafetyChecker] Orientation safety check failed!
```

根因有两层：

1. `ocs2::MPC_BASE::run()` 在 `currentTime >= solver.getFinalTime()` 时直接 `return false`，MPC 线程不再更新 policy，MRT 持续用过期轨迹（可滞后十余秒），最终姿态失控。
2. `/cmd_vel` 目标轨迹默认只有约 1 s 有效窗口，长距离爬楼梯时若 DDS 速度命令有间隙，MPC 参考会过期。

### 修改内容

- `ocs2_mpc/MPC_BASE.cpp`：规划过期时设置 `initRun_=true` 并继续求解，而不是永久停止 MPC。
- `perceptive_target_trajectories_publisher`：新增 20 Hz 定时重发活动 `cmd_vel` 目标；默认 `command_horizon_scale=2.5`（约 2.5 s 前瞻）。
- `p1_perceptive_gait_bridge_sim_test.launch.py`：新增 `command_horizon_scale`、`target_republish_rate` launch 参数。
- `config/p1/task.info`：`swingHeight` 从 0.15 提高到 0.18，改善台阶落差处的摆腿 clearance。

### 验证

已通过：

```bash
colcon build --packages-select ocs2_mpc perceptive_legged_control p1_dds_joy_tools --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
```

### 使用建议

爬楼梯时保持 trot 步态并持续给前进速度；若仍偶发摔倒，可尝试放慢 `velocity_x` 或把 `command_horizon_scale` 调到 `3.0`。


## 2026-06-09 13:45:15 UTC

读取最新 `/workspace/mpc_logs/session_20260609_134002` 日志，分析爬楼梯不稳定和摔倒原因；本次未修改控制代码。

### 诊断摘要

- `observation.csv` 显示摔倒前机身高度与目标高度明显脱节：观测 `base_z` 在 `mpc_time=52.38s` 达到约 `2.19m`，但 `mpc_target_trajectory.csv` 目标 `base_z` 最大只有约 `0.66m`。
- `mpc_time=56.75s` 时姿态已严重失控：roll 约 `89.8deg`，pitch 约 `73.9deg`，`base_z` 掉到约 `0.46m`，随后 `rosout.log` 反复出现 `[Legged Controller] Safety check failed`。
- `cmd_vel.log` 中最大前进速度约 `0.43m/s`、最大角速度约 `1.02rad/s`，不像是单纯速度过大导致，而是地形/目标高度和实际楼梯高度没有对齐。
- `events.log` 仍出现 `target_plan_expired`，说明目标轨迹仍有过期窗口，需要继续关注目标重发和 MPC 参考有效期。

### 初步判断

爬楼梯不稳定的主要原因是感知地形/目标轨迹没有正确把楼梯高度反映到 MPC 目标中，控制器在实际身体已经爬高时仍按接近平地的 `base_z` 和零 pitch/roll 规划，导致大高度误差、俯仰持续增大，最终触发安全检查并摔倒。


## 2026-06-09 13:47:44 UTC

回答“是否代码层面问题”的判断；本次只读检查目标发布器和感知参考管理代码，未修改控制代码。

### 判断

这更像代码/集成层面的问题，而不是单纯速度或参数问题。`PerceptiveTargetTrajectoriesPublisher` 发布的 cmd_vel 目标高度是相对地形高度，后续需要 `PerceptiveSwitchedModelReferenceManager` 根据 `smooth_planar`/`elevation` 地形图再补成绝对高度和坡面 pitch。如果控制器端没有收到有效地形、地形 frame/layer 不匹配、或地形高度没有覆盖楼梯区域，MPC 就会继续使用接近平地的目标高度，导致实际楼梯高度和目标高度脱节。

需要优先排查目标发布器、地形 topic/layer/frame、控制器内 reference adaptation 是否都在同一坐标系和同一高度语义下工作。


## 2026-06-09 13:52:36 UTC

根据“应该怎么修改可以修复爬楼梯不稳定”的问题，实施一个代码层面的最小修复。

### 修改内容

- `src/PerceptiveTargetTrajectoriesPublisher.cpp`：`cmd_vel` 目标生成时，线速度只按 yaw 从机体系旋到世界系，不再使用完整 roll/pitch/yaw 姿态旋转。
- 移除不再使用的 `ocs2_robotic_tools/common/RotationTransforms.h` include。

### 原因

`cmd_vel` 是平面导航命令。旧实现使用完整姿态旋转 `linear.x/y/z`，当机器人在楼梯上有明显 pitch/roll 时，前进速度会被错误投影出垂直速度分量，导致 MPC 目标速度包含非预期的 `v_z`，容易在台阶上放大俯仰和高度振荡。

### 验证

已通过：

```bash
colcon build --packages-select perceptive_legged_control --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
```

### 后续建议

重新跑楼梯场景后，对比最新 `observation.csv` 中 `vcom_z`、pitch、roll 是否明显收敛；若仍不稳，再继续排查控制器内部地形适配后的最终参考高度是否与实际台阶高度一致。


## 2026-06-09 13:57:15 UTC

读取最新 `/workspace/mpc_logs/session_20260609_135322`，分析 yaw-only `cmd_vel` 修复后仍摔倒的原因；本次未修改控制代码。

### 诊断摘要

- 新会话在 `mpc_time=26.466s` 结束，末尾 roll 约 `89.85deg`，触发 `[Legged Controller] Safety check failed`。
- 与上一轮不同，这次 `base_z` 最大约 `1.263m`，没有再冲到 `2.19m`，说明上一轮速度投影修复减轻了垂直发散。
- 主要异常变成侧向偏移：机器人从 `y≈0.02` 被带到 `y≈-1.52` 后才开始向前走，摔倒时达到 `y≈-2.51`。
- `cmd_vel.log` 显示早期存在较大横向命令 `linear_y=-0.5`，后续前进时仍有 `linear_y≈-0.08`。
- `empty_world.world` 中楼梯每级尺寸为 `0.30 x 1.20 x ...`，中心 `y=0`，有效台阶宽度约 `[-0.6, 0.6]`；最新轨迹已经远离台阶区域。

### 初步判断

这次摔倒主要不是高度目标问题，而是侧向命令/目标规划没有楼梯通道约束，机器人先横移出楼梯宽度，再在台阶边缘或台阶外侧继续前进，导致大 roll 翻倒。

### 建议

短期先禁用或强限幅 `cmd_vel.linear_y`，爬楼梯时保持 `y` 接近 0；中期在目标发布器或 gait bridge 中增加楼梯模式下的横向通道约束，例如限制目标 `base_y` 到台阶可通行范围内，并对 `linear_y` 加死区/限幅。


## 2026-06-09 13:59:51 UTC

根据最新日志和用户反馈，重新判断 `PerceptiveTargetTrajectoriesPublisher.cpp` 中 yaw-only `cmd_vel` 速度旋转修改的作用；本次未修改代码。

### 判断更新

最新 `session_20260609_135322` 显示摔倒主因不再指向速度投影本身。yaw-only 修改可以避免 pitch/roll 将平面 `cmd_vel` 投影出非预期垂直速度，但它不是本轮摔倒的主要原因。

当前更主要的问题是横向目标/命令把机器人带出楼梯通道：楼梯宽度约 `y in [-0.6, 0.6]`，而日志中机器人在爬楼梯前已偏到 `y≈-1.52`，摔倒时约 `y≈-2.51`。后续修复重点应放在楼梯模式下限制 `linear_y` 或约束目标 `base_y`，以及验证地形/落脚区域约束是否覆盖台阶边界。

## 2026-06-09 14:04:37 UTC
- 分析最新日志 `mpc_logs/session_20260609_135322`：速度 yaw-only 修改后仍摔倒，`base_z` 峰值约 1.263 m，roll 接近 90 deg，最终 y 约 -2.507，说明主因不像是线速度被 pitch/roll 投影到竖直方向。
- 对比世界模型楼梯宽度：楼梯 y 方向有效范围约 [-0.6, 0.6]，但最新轨迹在摔倒前已到 y≈-1.52，最终 y≈-2.51，目标轨迹也跟着向楼梯外侧走，属于目标/约束层面允许机器人离开可落脚区域。
- 参考 `/workspace/legged_perceptive`：参考工程的 `PerceptiveLeggedReferenceManager` 会按 terrain normal 设置 pitch，并用 `terrain + comHeight / cos(pitch)` 设置 base z；当前工程 `PerceptiveSwitchedModelReferenceManager` 使用 `terrainZ + pos.z()`，依赖上游 target z 为相对高度，语义更脆弱，可能在台阶/坡面上给出偏低 body height。
- 当前 `config/p1/task.info` 中 `footPlacement.enableCBF=false`、`enableSwingCBF=false`、`CollisionParam.enablefootCollision=false`，硬约束/CBF/碰撞保护关闭；建议优先限制楼梯模式 lateral target/cmd_vel，启用并调试 foot placement CBF/swing CBF，再考虑把高度参考逻辑对齐参考工程。

## 2026-06-09 14:18:51 UTC
- 用户已撤销 `PerceptiveTargetTrajectoriesPublisher.cpp` 中关于 cmd_vel 速度旋转的修改。
- 重新确认当前仓库状态：`src/PerceptiveTargetTrajectoriesPublisher.cpp` 无 diff，当前仅 `AGEBT.md` 处于修改状态。
- 当前代码仍使用 `getRotationMatrixFromZyxEulerAngles(zyx)` 对 cmd_vel 线速度按完整 RPY 旋转。
- 重新确认 `config/p1/task.info`：`footPlacement.enableCBF=false`、`enableSwingCBF=false`、`CollisionParam.enablefootCollision=false`、`enableSphereSdfConstraint=false`，后续排查重点应转向楼梯横向目标越界与足端/碰撞约束保护不足。

## 2026-06-09 14:20:30 UTC
- 解释 `terrain + comHeight / cos(pitch)`：`terrain` 是目标位置处地形高度，`comHeight` 是期望质心/机身相对地面的标称高度，`pitch` 是根据地形法向估计出的机身俯仰角，`cos(pitch)` 用于把沿机身/地形法向的高度换算成世界系 z 方向高度。
- 说明该公式的目的：在坡面或台阶边缘使 base z 随 pitch 做投影补偿，避免机器人倾斜时世界系 z 参考过低。

## 2026-06-09 14:23:13 UTC
- 解释 `terrainZ + pos.z()`：`terrainZ` 是目标 xy 位置采样到的地形高度，`pos.z()` 在当前工程语境下应表示相对地形的 base/COM 高度，因此两者相加得到世界系 base z。
- 强调该公式依赖 `pos.z()` 的语义：若 `pos.z()` 已经是世界系高度，会导致重复加地形；若 `pos.z()` 只是 raw target 的 0 或过低值，则会导致机身高度参考过低。与参考工程的 `terrain + comHeight / cos(pitch)` 相比，它不会自动根据 pitch 做高度投影补偿。

## 2026-06-09 14:25:11 UTC
- 澄清 `terrain` 和 `terrainZ` 在当前讨论中语义相同，都是目标 xy 位置处采样得到的地形世界系 z 高度；两者差异主要是变量命名/代码上下文。
- 因此两个公式的核心差异不在 terrain 项，而在第二项：`comHeight / cos(pitch)` 是固定标称高度加 pitch 投影补偿，`pos.z()` 则依赖 target 中 z 分量已经被正确表达为相对地形高度。

## 2026-06-09 14:26:20 UTC
- 追踪 `terrainZ + pos.z()` 中 `pos.z()` 的来源：在 `PerceptiveSwitchedModelReferenceManager::modifyReferences()` 中，`pos` 来自 `centroidal_model::getBasePose(state, info_).head(3)`，而 `state` 来自 `relativeTargetTrajectories->getDesiredState(time)`。
- 因此 `pos.z()` 实际由上游目标轨迹赋值：`PerceptiveTargetTrajectoriesPublisher::poseToTargetTrajectories()` 将 `targetPose` 写入 `stateTrajectory[1]`，cmd_vel 模式下 `targetPose(2)=max(currentPose(2)-terrainHeight, comHeight_)`，goal 模式下 `targetPose(2)=pose.pose.position.z + comHeight_`。
- 说明当前工程中 `pos.z()` 被 reference manager 当成“相对地形的 base/COM 高度”使用，并在 reference manager 内部再加一次 `terrainZ` 转成世界系高度。

## 2026-06-09 14:32:52 UTC
- 用户说明控制架构存在两个 reference/default 状态：默认 `liedown` 用于位控趴下，另有站立状态；在 `liedown` 和 `stand` 位控状态下不应有 MPC 控制接入，只有切换到 trot 或其他步态时 MPC 才接入。
- 分析结论：如果感知 MPC 的高度适配改为 `terrain + comHeight / cos(pitch)`，其中 `comHeight` 应绑定 MPC-active 的站立/行走标称 body height，而不是默认趴下高度。
- 当前配置检查：`perceptive_target.yaml` 的 `referenceFile` 指向 `legged_controllers/config/p1/reference.info`，其中 `comHeight=0.58`；`PerceptiveLeggedInterface::setupReferenceManager()` 也从同一个 `referenceFile` 读取 `comHeight` 注入 `PerceptiveSwitchedModelReferenceManager`。
- 后续设计注意：若默认 reference 切换为 `liedown` 文件，不能直接用该文件的 `comHeight` 做 terrain height compensation，应单独读取站立/MPC reference height 或增加明确参数，例如 `mpcComHeight`/`standingComHeight`。

## 2026-06-09 14:34:09 UTC
- 评估用户提出的修改方向：在感知 MPC 接入阶段使用站立/行走参考高度作为 `comHeight`，并用于 `terrain + comHeight / cos(pitch)` 的高度适配是合理的。
- 建议实现方式：不要复用默认 `liedown` reference 的高度；应新增或明确使用 `mpcComHeight`/`standingComHeight`/`locomotionComHeight`，让 MPC-active 的 terrain compensation 独立绑定站立/步态高度。
- 注意条件：该高度适配只应在 trot/walk/stair 等 MPC 接入状态生效；`liedown` 和纯 stand 位控状态不应由 MPC reference manager 接管控制。

## 2026-06-09 14:37:14 UTC
- 按用户确认实施修改：感知 MPC 高度适配从 `terrainZ + pos.z()` 改为 `terrainZ + locomotionComHeight / cos(pitch)`，避免依赖 target 中 `pos.z()` 的相对高度语义。
- 在 `PerceptiveSwitchedModelReferenceManager` 中将成员语义改为 `locomotionComHeight_`，并用地形法向计算出的 `pitch` 同时设置 base pitch 和 base z 投影补偿。
- 在 `PerceptiveLeggedInterface::setupReferenceManager()` 中新增 `locomotionComHeight` 读取逻辑：若 reference 文件未配置该项，则回退使用原 `comHeight`，保持向后兼容。
- 在 `/workspace/src/legged_control/legged_controllers/config/p1/reference.info` 和运行时安装路径 `/workspace/install/legged_controllers/share/legged_controllers/config/p1/reference.info` 中加入 `locomotionComHeight 0.58`，明确 MPC-active 站立/行走高度，不与默认 `liedown` 高度混用。
- 验证：执行 `colcon build --packages-select perceptive_legged_control --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo`，构建通过。

## 2026-06-09 14:39:31 UTC
- 进一步补全专用 `locomotionComHeight`：`PerceptiveTargetTrajectoriesPublisher` 现在也从 reference 文件读取 `locomotionComHeight`，若未配置则回退 `comHeight`。
- raw target 生成中的 goal/cmd_vel 高度也改为使用 `locomotionComHeight_`：goal 模式为 `pose.pose.position.z + locomotionComHeight_`，cmd_vel 模式为 `max(currentRelativeHeight, locomotionComHeight_)`。
- 这样 target publisher 与 `PerceptiveSwitchedModelReferenceManager` 的高度语义一致，都使用 MPC-active 的站立/行走高度，而不是默认 `liedown` 高度。
- 验证：再次执行 `colcon build --packages-select perceptive_legged_control --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo`，构建通过。
- 注意：`/workspace/src/legged_control/legged_controllers/config/p1/reference_lie_down.info` 当前也处于修改状态，但本次未修改该文件。


## 2026-06-09 15:06:54 UTC
- 分析最新日志 `/workspace/mpc_logs/session_20260609_145856`：楼梯失稳前实际接触 mode 从 trot 的 `9/6` 逐渐变成 `13/14/2/1/5/12/0` 等组合，说明预期落地脚未稳定接触；随后 roll 达到 `1.57 rad`，触发 SafetyChecker 的 `±pi/2` 阈值。
- `cmd_vel` 在失稳前保持 `linear_y=0`、`angular_z=0`，因此本轮不是手柄横向输入直接造成；用户观察到的四足踏地/摆动越来越不规律与日志一致。
- 对比 NACL：其配置加载默认启用 FootPlacement CBF、SwingFootPlacement CBF 和 FootCollision；当前 P1 感知配置此前只启用普通 FootPlacement 软约束。
- 仅修改感知专用 `config/p1/task.info`：设置 `footPlacement.enableCBF=true`、`footPlacement.enableSwingCBF=true`、`CollisionParam.enablefootCollision=true`；`enableSphereSdfConstraint=false` 保持不变。
- 未修改原始 `legged_control`、Gazebo world、gait 或手柄控制代码。
- 验证：`colcon build --base-paths /workspace/src --packages-select perceptive_legged_control --symlink-install` 构建通过；Gazebo 同路径动态复测尚待执行。


## 2026-06-09 15:13:50 UTC
- 分析最新日志 `/workspace/mpc_logs/session_20260609_150816`：失稳前接触 mode 逐渐不规则，base 竖直速度振荡并最终侧翻；该会话还包含较大的前进、横移和转向输入，机器人明显偏离窄楼梯通道，日志未出现 MPC infeasible 或约束求解告警。
- 定位到 `PerceptiveTargetTrajectoriesPublisher` 使用完整 roll/pitch/yaw 旋转 `cmd_vel` 线速度；楼梯上的 pitch/roll 会把平面手柄输入投影成非预期目标 `v_z`，可能加剧摆腿与接触紊乱。
- 对比 NACL 感知专用轨迹生成：其使用 heading/lateral/yaw-rate 平面命令，由地形生成器单独适配高度和姿态，旧的完整 RPY 速度旋转逻辑未启用。
- 仅修改感知专用 `src/PerceptiveTargetTrajectoriesPublisher.cpp`：线速度改为仅按 yaw 转到世界平面，目标 `v_z` 保持为零；前进、横移和转向控制均保留。
- 未修改原始 `legged_control`、步态、世界模型或手柄代码。
- 验证：`colcon build --base-paths /workspace/src --packages-select perceptive_legged_control --symlink-install` 构建通过；Gazebo 楼梯动态复测待执行。


## 2026-06-09 15:22:11 UTC
- 分析最新 `/workspace/mpc_logs/session_20260609_151416` 与 `session_20260609_151502`：yaw-only 速度投影已生效，但步态切换后仍出现不规则接触和侧翻。
- 找到主要未对齐项：NACL 感知 controller 固定加载站立/行走 `reference.info`；当前 P1 仿真默认给感知 controller 加载 `reference_lie_down.info`，而感知 target publisher 使用站立 reference，导致 MPC 内部默认关节态 `1.35/-2.55` 与 locomotion 目标 `0.635/-1.188` 不一致。
- 确认当前 `joy_control.py` 的 stand/lie_down 仍通过 MPC target 发布，并非独立位置控制器，因此感知 reference manager 必须保留目标自身的 0.20/0.58 相对高度。
- 仅在感知包新增 `config/p1/reference.info`，并让 controller 配置和 target publisher 统一读取它。
- 新增 `launch/perceptive_p1_sim.launch.py`，显式向原 P1 仿真传入感知 task、感知站立 reference 和 `perceptive_legged_control/PerceptiveLeggedController`；未修改原始 `legged_control`。
- 高度适配由 `terrainZ + requestedHeight / cos(pitch)` 改为 `terrainZ + requestedHeight`：locomotion 对齐 NACL 的 `terrainZ + 0.58`，lie_down 保持 `terrainZ + 0.20`。
- 验证：感知包构建通过；新增 launch Python 语法和 `ros2 launch ... --show-args` 解析通过。后续必须使用 `ros2 launch perceptive_legged_control perceptive_p1_sim.launch.py` 复测。
- 尚存差异：零 `cmd_vel` 当前停止目标重发，日志会出现 `target_plan_expired`，后续需单独修复持续驻停参考。


## 2026-06-09 15:27:02 UTC
- 本轮仅分析最新 `/workspace/mpc_logs/session_20260609_152517`，未修改代码。
- `mpc_time≈45-63s` 的爬楼阶段 roll 基本接近零；失稳从 `15:26:22` 的持续横向命令开始，`linear_y` 从约 `0.21m/s` 增长到接近 `0.50m/s`，同一秒 roll 超过 `0.35rad`，约 `0.2s` 后超过 `0.70rad`。
- 失稳后实际接触 mode 从 `9/6` 扩散为多种异常组合。直楼梯宽度 `1.20m`、边界约 `y=±0.60m`，失稳开始时右侧足端已到约 `y=-0.66m`，超出台阶边界。
- 对照 `session_20260609_152419`：机器人到达 `x=4.405m、base_z=1.748m` 时仍保持 mode `9`、roll≈-0.003rad，证明当前 reference/高度对齐后能够稳定爬楼一段距离。
- `target_plan_expired` 不是本轮直接触发因素；失稳窗口内目标已持续更新且相对高度保持 `0.58m`。
- 当前缺失的是楼梯通道或全身可行域保护：FootPlacement 只约束各足局部凸区域，不能阻止较大横移命令把机身/足端带出窄楼梯。为避免牺牲横移能力，本轮未直接限幅或禁用 `linear_y`。


## 2026-06-09 15:33:19 UTC
- 根据“落脚点先错、腿部随后紊乱”的判断，继续逐行对比 NACL。
- 确认移植遗漏：NACL `PerceptiveLeggedPrecomputation::getPolygonConstraint()` 的 `polytopeB` 包含 `-0.04` 边缘内缩，当前实现此前没有，允许足端优化结果贴近凸平面/台阶边缘。
- 仅修改感知包 `src/interface/PerceptiveLeggedPrecomputation.cpp`，恢复 NACL 的 `-0.04` 落脚安全余量。
- 将感知 `config/p1/task.info` 中支撑相 FootPlacement CBF 对齐 NACL：`cbfLambda=1.0`、`cbfPenaltyDeltaParam=1e-4`。
- SwingFootPlacementConstraintCBF 保持 NACL 当前实际位置约束公式，未启用参考源码中被注释的速度 CBF 形式。
- 检查了当前 OCS2 SQP RequestSet：普通节点同时请求 `Constraint + SoftConstraint`，预计算刷新条件不是本轮直接根因，因此未修改。
- 未修改原始 `legged_control`、手柄横移、世界模型或 base 高度公式。
- 验证：`colcon build --base-paths /workspace/src --packages-select perceptive_legged_control --symlink-install` 构建通过；Gazebo/RViz 动态落脚复测待执行。


## 2026-06-09 15:39:43 UTC
- 分析最新 `/workspace/mpc_logs/session_20260609_153532` 与 `session_20260609_153617`。前者启动时已处于明显倾倒状态，不作为楼梯回归依据；后者仅有前进命令，在 `x≈1.16m` 首次接触台阶后 roll 快速超过 `0.35rad`，随后翻倒。
- 确认直接回归来自上一轮照搬的原始 `polytopeB -= 0.04`：多边形半空间法向量未归一化，因此该值并不等于 4 cm 几何距离。
- 对 0.30m 深的踏面，原始写法会在对应方向每侧内缩约 0.133m，使可落脚宽度仅剩约 0.033m，足端在第一阶几乎无可行区域。
- 将感知包的落脚区域安全余量改为 `polytopeB -= 0.04 * ||A||`，并在统一半空间朝向后执行；现在不同边长的边界均对应真实 4 cm 内缩，0.30m 踏面保留约 0.22m 可落脚宽度。
- 保留上一轮已与 NACL 对齐的 FootPlacement CBF 参数，未修改原始 `legged_control`、台阶世界、手柄或高度参考。
- 验证：`colcon build --base-paths /workspace/src --packages-select perceptive_legged_control --symlink-install` 构建通过；需重新启动仿真后动态复测第一阶落脚。


## 2026-06-09 15:52:56 UTC
- 分析最新有效日志 `/workspace/mpc_logs/session_20260609_154610`：首个明显异常发生在 `mpc_time≈22.10s`，base 仅到 `x≈0.66m`，但前足已摆到 `x≈1.11m`，说明失稳由首个跨阶落脚过程触发，而不是机身撞上楼梯。
- 失稳前计划接触模式为 trot 的 `9/6`，实际观测模式却从 `6` 突然扩散为 `14→0→1`；约 0.12s 后 roll/pitch 超过安全阈值，随后四足接触紊乱。
- 对比此前能继续前进的日志，异常是在上一轮同时加入多边形内缩并将支撑相 CBF 从 `lambda=0.5, delta=0.005` 加强为 `lambda=1.0, delta=1e-4` 后出现。
- 精确撤回上述三处回归：移除额外多边形内缩，恢复 `cbfLambda=0.5` 和 `cbfPenaltyDeltaParam=0.005`。
- 保留 `enableCBF=true`、`enableSwingCBF=true`、`enablefootCollision=true`，没有关闭或牺牲感知 MPC 功能，也未修改原始 `legged_control`。
- 验证：`colcon build --base-paths /workspace/src --packages-select perceptive_legged_control --symlink-install` 构建通过；需要完整重启仿真后复测首个跨阶接触模式是否保持 `9/6`。


## 2026-06-09 16:04:18 UTC
- 用户复测反馈：撤回额外多边形内缩，并恢复支撑相 CBF `lambda=0.5、delta=0.005` 后，楼梯表现“好很多”。
- 该结果支持此前判断：首阶直接失稳主要由过强的新增落脚约束触发。
- 本轮不继续修改参数，保持当前感知约束启用状态；后续依据新日志定位更高台阶处最早出现的足端、接触模式或姿态偏差。


## 2026-06-09 16:09:26 UTC
- 按用户要求重新加入落脚多边形额外内缩，但不恢复上一轮过强的 CBF 参数，以便单独验证几何边距。
- 在半空间方向统一后使用 `polytopeB(i) -= 0.04 * polytopeA.row(i).norm()`，表示与边长无关的真实 4 cm 内缩；没有使用尺度相关的原始常数减法。
- 支撑相 CBF 保持已改善稳定性的 `cbfLambda=0.5`、`cbfPenaltyDeltaParam=0.005`；Swing CBF、FootCollision 及其他感知功能均保持启用。
- 验证：`colcon build --base-paths /workspace/src --packages-select perceptive_legged_control --symlink-install` 构建通过。下一轮复测用于隔离判断 4 cm 内缩本身是否影响首阶接触模式。


## 2026-06-09 16:13:26 UTC
- 用户单变量复测反馈：保持 `cbfLambda=0.5、cbfPenaltyDeltaParam=0.005` 不变，仅重新加入归一化 4 cm 多边形内缩后，楼梯表现再次变差。
- A/B 结果确认固定 4 cm 内缩本身不适合当前 P1 的 0.30m 台阶踏面、凸区域分割与步幅组合。
- 已再次移除 `polytopeB(i) -= 0.04 * polytopeA.row(i).norm()`，恢复此前表现“好很多”的无额外内缩版本。
- FootPlacement CBF、Swing CBF、FootCollision 均继续启用，CBF 参数保持 `0.5/0.005`，未修改原始 `legged_control`。
- 感知包重新构建通过。后续如需边缘余量，应基于区域尺寸设计自适应内缩并单独验证，不再使用固定 4 cm。


## 2026-06-09 16:18:25 UTC
- 分析最新 30 度楼梯日志 `/workspace/mpc_logs/session_20260609_161402`。机器人已爬到 `x≈3.26m、base_z≈1.88m`，并非接触第一阶即停止。
- 停止前实际 pitch 约 `-0.524rad`，符合 30 度地形；`SafetyChecker` 只检查 roll 是否超过 ±90 度，因此不是 30 度姿态被安全阈值误杀，而是翻倒后才触发停止。
- 最早失稳表现为计划模式仍为 trot `9/6`，实际接触模式在 `mpc_time≈40.36s` 从 `9` 突变为 `1→5→7`，随后 pitch/roll 和垂向速度发散。
- 当前机身参考使用 `terrainZ + requestedRelativeHeight`。机身跟随 30 度 pitch 后，0.58m 竖直高度只对应约 `0.58*cos(30°)=0.502m` 法向离地距离，比平地少约 0.078m，导致腿部压缩、工作空间和接触裕量下降。
- 对照 `legged_perceptive` 原版，其使用 `terrain + comHeight / cos(pitch)`；NACL 修改版将 `/cos(pitch)` 注释掉，当前实现与后者一致。30 度离散台阶比低坡度更容易暴露这一差异。
- 本轮仅完成原因诊断，未修改代码。建议下一步在感知 reference manager 中恢复带保护上限的坡度高度补偿，并单独复测 lie_down 与楼梯。


## 2026-06-09 16:19:29 UTC
- 用户询问 30 度楼梯高度公式是否需要对齐。建议对齐 `legged_perceptive` 原版的坡面法向高度保持逻辑，即 locomotion 使用 `terrainZ + comHeight / cos(pitch)`。
- 不建议无条件应用到所有目标：`lie_down` 的 0.20m 相对高度应继续使用 `terrainZ + requestedHeight`，避免坡面趴下姿态被额外抬高。
- 该选择基于最新日志中 30 度时法向离地高度损失约 7.8cm、随后接触模式崩溃的证据；本轮仅确定修改方向，未改代码。


## 2026-06-09 16:20:17 UTC
- 按确认方案对齐 `legged_perceptive` 原版的坡面法向高度保持逻辑，但仅应用于站立/行走目标。
- `PerceptiveSwitchedModelReferenceManager` 依据整条目标轨迹的最终相对高度区分模式：最终高度接近 `locomotionComHeight=0.58m` 时启用坡度补偿；最终高度为 lie_down 的 `0.20m` 时不补偿。
- locomotion 高度改为 `terrainZ + requestedRelativeHeight / max(cos(pitch), 0.5)`。30 度时 0.58m 对应约 0.670m 竖直高度，从而保持约 0.58m 的坡面法向间距。
- lie_down 继续使用 `terrainZ + requestedRelativeHeight`，避免坡面趴下目标被额外抬高。
- 未修改原始 `legged_control`、步态、手柄、落脚约束或碰撞约束。
- 验证：`colcon build --base-paths /workspace/src --packages-select perceptive_legged_control --symlink-install` 构建通过；动态复测需覆盖 30 度连续爬升和坡面 lie_down。
