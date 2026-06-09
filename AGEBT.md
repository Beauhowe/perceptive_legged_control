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

## 2026-06-09 19:05:00 CST

### 问题：站在楼梯上会抖

`session_20260609_094134` 分析：hold 以 20 Hz 用实时 `currentRelativeHeight` 更新目标 z，机身轻微弹跳会被目标追逐放大（`mpc_target` start_z 波动大），表现为站立抖动。

### 修改

`PerceptiveTargetTrajectoriesPublisher.cpp`：

- 松杆进入 hold 时调用 `latchHoldRelativeHeight()` 锁定相对高度。
- hold 重发期间 `targetPose(2)` 与 `poseToTargetTrajectories` 起点 z 均使用 `holdRelativeHeight_`，不再每帧读观测高度。
- 非零 `cmd_vel` 时清除锁定。

### 验证与使用

```bash
colcon build --packages-select perceptive_legged_control
```

坡上停稳：先减速 → 切 **stance (gait_id=0)** → 再松杆；trot 零速仍会周期摆腿，不如 stance 稳。

## 2026-06-09 19:20:00 CST

### 日志：`session_20260609_094732`

- 场景：20° 楼梯（`base_y≈1.8`），爬至 `z≈1.83m`，`t≈39.6s` 摔倒（`roll≈0.79`，`vz≈-1.69`）。
- 抖动：高位零速段 `t=36–39s`，`vcom_z` std≈0.34，最大约 0.87；低位 `t=24–27s`（`z≈0.61`）vz_std≈0.02。
- `cmd_vel`：站立段约一半为零速，但仍有 `linear_y` 尖峰（最大约 0.5），导致 `y` 从 1.75 漂到 1.29。
- `mpc_target`：`start_z` 在 t=36.0–36.9s 维持 0.644464，t=38.36–38.81s 维持 0.642008，但 37–39s 仍有阶跃（0.66→0.58→0.36），需确认 latch 补丁是否已编译部署。
- `events.log`：仅启动后 2 次 `target_plan_expired`，hold 续发基本正常。

## 2026-06-09 19:35:00 CST

### 日志：`session_20260609_095217`（latch 复测）

- 场景：26° 灰楼梯（`y≈0`），爬至 `z≈2.23m`，时长 64s。
- **高度 latch 生效**：`mpc_target` `start_z=0.564726` 在 t=48–53s 连续 24 次不变；整体 `start_z` std=0.049（对比 `094732` 的 0.49）。
- **高位站立改善**：零速段 t=48–56s，`vcom_z` std≈0.09，`|roll|max≈0.036`，`pitch≈-0.44` 稳定。
- **摔倒原因**：t≥56s 出现后退 `linear_x<0`（该段零速仅 33%），机器人从楼梯顶端向后滑落，t≈60.4s `roll≈-0.93`；非站立抖动直接导致。
- 建议：楼梯顶端停稳后避免后退指令；`linear_y` 全程为 0，表现良好。

## 2026-06-09 19:50:00 CST

### 用户反馈：越来越差

跨 session 对比：`084930`（30° 成功 z=2.87m）、`095626`（30° z=2.72m 站 25s roll<0.02 后因 `angular_z` 摔）、`095744`（摇杆横向/后退/转向摔）。爬升与站立能力并未单调变差；摔倒多由**微小摇杆输入打断 hold** 引起。

### 修改

- `PerceptiveTargetTrajectoriesPublisher`：hold 死区（`hold_linear_deadzone=0.08`、`hold_angular_deadzone=0.2`）；进入 hold 锁定 x/y/yaw/相对高度；死区内保持 hold 不响应转向/微移。
- `task.info`：`swingHeight` 0.17→0.18，`touchDownVelocity` -0.05→-0.1（恢复更稳摆腿参数）。
- launch 新增 `hold_linear_deadzone`、`hold_angular_deadzone` 参数。


2026-06-09 14:32:25 CST - 用户说明代码运行在容器里；本轮未进行代码修改。
2026-06-09 14:33:36 CST - 用户说明 `src/perceptive_legged_control` 是感知 MPC 代码目录；本轮未进行源码修改。
2026-06-09 14:39:38 CST - 对比 `src/perceptive_legged_control` 与 `legged_perceptive`、`nacl` 的感知 MPC 逻辑；未修改源码。结论：当前包已移植 terrain receiver、perceptive reference manager、convex region selector、foot placement、foot collision、CBF/SDF 相关结构；按当前 `config/p1/task.info` 实际启用 foot placement 和 foot collision，FootPlacement CBF、SwingFootPlacement CBF、SphereSDF 当前为关闭。
## 2026-06-09 14:41:40 CST 
   - 补充发现：`PerceptiveTerrainBuffer` 仍在 `CMakeLists.txt` 中编译，但当前 controller 主链路使用 `PlanarTerrainReceiver`；`perceptive_p1_controllers.yaml` 中的 `perceptive_height_layer` 和 `perceptive_height_offset` 对当前主链路没有直接作用。
   当前主链路
   PerceptiveLeggedController 替换成 PerceptiveLeggedInterface，并在 MPC solver
   里加入 PlanarTerrainReceiver 同步模块：src/perceptive_legged_control/src/
   controller/PerceptiveLeggedController.cpp:11。
   PlanarTerrainReceiver 订阅 /convex_plane_decomposition_ros/planar_terrain，在
   preSolverRun() 把新地形写进 MPC 共享 terrain，并更新 SDF：src/
   perceptive_legged_control/src/synchronized_module/
   PlanarTerrainReceiver.cpp:24。
   PerceptiveLeggedInterface 初始化平地 fallback，然后添加 foot placement、CBF、
   collision、可选 SphereSDF 约束：src/perceptive_legged_control/src/interface/
   PerceptiveLeggedInterface.cpp:40。
   PerceptiveSwitchedModelReferenceManager 会按地形修正 base pitch/z、更新凸区域
   foothold，并给 swing planner 传 lift-off/touch-down/swing max height：src/
   perceptive_legged_control/src/interface/
   PerceptiveSwitchedModelReferenceManager.cpp:29。

   和 legged_perceptive 比
   结构基本来自它：controller、terrain receiver、perceptive interface、reference
   manager、convex region selector、precomputation 都是一一对应。
   但 legged_perceptive 原版主要默认接了 FootPlacementConstraint 和
   FootCollisionConstraint，SphereSDF 创建了但没加进 problem：legged_perceptive/
   legged_perceptive_interface/src/PerceptiveLeggedInterface.cpp:51。它也没有
   nacl 后续的 FootPlacement CBF、SwingFootPlacement CBF、沿地形连线取 swing max
   height 这些逻辑。

   和 nacl 比
   当前代码更接近 nacl：已经移植了 FootPlacement CBF、SwingFootPlacement CBF、配
   置读取、foot collision、可选 SphereSDF，以及 swing 最高点沿 elevation 连线取最
   大值再乘 1.05 的逻辑。对应 nacl 在这里：nacl/perceptive_legged_control/
   OCS2_ws/src/legged_perceptive/legged_perceptive_interface/src/
   PerceptiveLeggedInterface.cpp:24、nacl/perceptive_legged_control/OCS2_ws/src/
   legged_perceptive/legged_perceptive_interface/src/
   PerceptiveLeggedReferenceManager.cpp:75。

   关键差异有三点：

   1. 当前配置实际没有启用 CBF：enableCBF false、enableSwingCBF false，只启用
      footPlacement 和 footCollision：src/perceptive_legged_control/config/p1/
      task.info:323。nacl 的 aliengo 配置里 enableCBF true，enableSwingCBF
      false：nacl/perceptive_legged_control/OCS2_ws/src/legged_perceptive/
      legged_perceptive_controllers/config/aliengo/task.info:309。

   2. 当前 SwingFootPlacement CBF 激活条件比 nacl 保守。nacl 约束类里基本是摆动相
      就激活；当前通过 getSwingFootPlacementFlags() 还要求过了初始支撑结束时间：
      src/perceptive_legged_control/src/constraint/
      SwingFootPlacementConstraintCBF.cpp:29。

   3. 当前 ROS 2 包多了 perceptive_target_trajectories_publisher，它会根据地形高
      度生成 /cmd_vel 和 /move_base_simple/goal 的 target trajectories：src/
      perceptive_legged_control/src/
      PerceptiveTargetTrajectoriesPublisher.cpp:35。这是当前工程自己的上层目标适
      配链路，不是 nacl/legged_perceptive 的 solver 内部逻辑。

   另外有个小风险点：perceptive_p1_controllers.yaml 里的 perceptive_height_layer
   和 perceptive_height_offset 当前主链路不读取；它们对应的是仍在编译但主链路未使
   用的 PerceptiveTerrainBuffer。当前 controller 实际读取的是
   perceptive_terrain_topic 和 perceptive_sdf_elevation_layer

## 2026-06-09 15:10:00 CST - 
   在 `legged_gazebo/worlds/empty_world.world` 新增 30° 坡楼梯模型 `stairs_30deg`：17 级、踏步深 0.30 m、级高 0.173 m（tan30°）、宽度 2.0 m（原楼梯 1.2 m），位置 y=-1.80（与 y=0 灰楼梯、y=1.80 蓝 20° 楼梯平行）。已 `colcon build --packages-select legged_gazebo`；需重启仿真 launch 生效。

## 2026-06-09 15:35:00 CST 
   - 新增感知仿真会话日志：`perceptive_legged_control` 包内 `perceptive_session_logger` 节点，启动后在 `log_dir/session_YYYYMMDD_HHMMSS/` 写入 `events.log`、`observation.csv`、`cmd_vel.log`、`mode_schedule.log`、`rosout.log`；`events.log` 会在 MPC target 过期时记录 `target_plan_expired`。`p1_perceptive_gait_bridge_sim_test.launch.py` 默认 `enable_session_logger:=true`、`log_dir:=/workspace/logs`。

## 2026-06-09 16:20:00 CST 
   - 修复并扩展 `perceptive_session_logger`：`mpc_target.log` 现记录每次 target 的起止 base 位姿与 12 关节角；新增 `mpc_target_trajectory.csv` 记录 target 各节点轨迹；`observation.csv` 扩展 12 关节角与 4 足端在 `odom` 下的位置（TF：`LF_FOOT`/`LH_FOOT`/`RF_FOOT`/`RH_FOOT`）。需 `colcon build --packages-select perceptive_legged_control` 后重启 launch。

## 2026-06-09 16:45:00 CST 
   - Logger 二次修复：修正 `mpc_target.log` 关节列 CSV 错位；`mpc_target`/`mpc_target_trajectory` 内容去重 + 默认 5Hz 上限；`cmd_vel` 默认 20Hz 采样；`rosout` 默认仅 WARN 及以上（`rosout_log_rate_hz:=0`）；`target_plan_expired` 按 `target_final_time` 去重。Launch 新增 `mpc_target_log_rate_hz`、`cmd_vel_log_rate_hz`、`rosout_log_rate_hz` 参数。

## 2026-06-09 17:10:00 CST 
   - 实现零速 hold target（P0）：`PerceptiveTargetTrajectoriesPublisher` 在收到零 `cmd_vel` 后进入 `holdTargetActive_` 模式，按当前位姿 + 地形高度发布零速 target，并由 `target_republish_rate` 定时器持续刷新，避免松杆后 MPC 规划过期。新增参数 `hold_target_on_zero_vel`（默认 true）；`p1_perceptive_gait_bridge_sim_test.launch.py` 已透传。需 `colcon build --packages-select perceptive_legged_control` 后重启 launch。

## 2026-06-09 17:25:00 CST 
   - 日志验证 hold target：`session_20260609_084930`（修后）约 81s、爬至 z=2.87m（30° 楼梯），`events.log` 无 `target_plan_expired`，无侧翻（|roll|max=0.33），t≈45s 在 z≈2.84m 近零速站稳；`mpc_target` 持续刷新至 t=80s。对比 `session_20260609_083654`（修前）：3 次 plan 过期、roll→1.57 侧翻。`session_20260609_084759` 为短测仍有 2 次早期过期（可能未重编译或起步站立）。

## 2026-06-09 17:40:00 CST 
   - 坡上高度补偿：`PerceptiveSwitchedModelReferenceManager` 将 `terrainZ + pos.z()` 改为 `terrainZ + pos.z()/cos(pitch)`；publisher 改为平地名义高度编码。

## 2026-06-09 17:55:00 CST 
   - 坡上高度补偿已回退：`session_20260609_090355` 验证失败（z 仅至 1.2m 即侧翻，|pitch|≈1.07，|vz|≈1.9）。根因：地形 pitch 在平地/台阶棱处有噪声，`/cos` 放大目标高度；hold 与运动混用 `currentRelative*cos` 在动态 pitch 下不稳定。已恢复 `terrainZ + pos.z()` 与 `max(currentRelative, comHeight)`，零速 hold target 保留。后续建议：`|pitch|>0.35rad` 才补偿；hold 发 `comHeight` 名义值；统一 controller/publisher 的 comHeight。

## 2026-06-09 18:10:00 CST 
   - 减摆腿冲击：`maxHeightAlongLine×1.05` 改为可配置 `swingHeightAlongLineScale`（默认 1.0）；`task.info` 中 `swingHeight` 0.18→0.15、`touchDownVelocity` -0.1→-0.05。需 `colcon build --packages-select perceptive_legged_control` 后重启 launch 再测 30° 楼梯。

## 2026-06-09 18:30:00 CST 
   - 日志分析 `session_20260609_092929`（30° 楼梯摔倒）：仅爬至 z≈1.9m（成功会话 `084930` 达 2.87m）。t≈40s 仍稳（y≈-1.66，pitch≈-0.51）；t≈44s 横向漂移至 y≈-1.35 且 `vcom_x≈0.83`；t≈45s roll→-0.59、vz→-1.12；t≈46s 侧翻 roll≈-1.52。`cmd_vel` 存在持续 `linear_y`（最大约 -0.44），为偏离楼梯中心 y≈-1.8 的主因。操作：30° 楼梯保持 `linear_y=0`、放慢 `linear.x`。

## 2026-06-09 18:45:00 CST 
   - 修复坡上 trot 零速站立：hold 模式改用当前地形相对高度 `currentRelativeHeight`（不再 `max(..., comHeight)` 把机身往平地高度拉）；`swingHeight` 0.15→0.17 改善 trot 落脚 clearance。需 `colcon build --packages-select perceptive_legged_control` 后重启 launch。

## 2026-06-09 19:05:00 CST 
   - 修复楼梯站立抖动：`PerceptiveTargetTrajectoriesPublisher` 在松杆进入 hold 时锁定 `holdRelativeHeight_`，20 Hz 重发不再每帧追踪带噪声的 `currentRelativeHeight`（避免目标高度随弹跳正反馈振荡）；轨迹起点 z 与目标 z 使用同一锁定值。操作：坡上停稳建议切 **stance** 再松杆；trot 零速仍会交替摆腿。需 `colcon build --packages-select perceptive_legged_control` 后重启 launch。

## 2026-06-09 19:20:00 CST 
   - 日志分析 `session_20260609_094732`（20° 楼梯 y≈1.8，约 40s 后摔倒）：t=24–27s 低位（z≈0.61）vz_std≈0.02 较稳；t=36–39s 高位（z≈1.76）零速站立 vz_std≈0.34、|vz|≈0.87，机身上下抖。`cmd_vel` 在“站立”段并非全零（约 50% 零速，仍有 `linear_y` 最大约 0.5），y 从 1.75 漂到 1.29 偏离楼梯中心；t≈39.6s roll→0.79、vz→-1.69 侧翻。`mpc_target` start_z 在 t=36–38s 有 0.644/0.642 平台但仍阶跃变化，需确认是否已编译高度锁定补丁。建议：20° 楼梯保持 `linear_y=0`、高位先切 stance 再松杆、编译 latch 后复测。

## 2026-06-09 19:35:00 CST 
   - 日志分析 `session_20260609_095217`（26° 灰楼梯 y≈0，64s）：高度 latch 已生效——`start_z=0.564726` 连续锁定 24+ 次发布；高位零速站 `t=48–56s`（z≈2.23m）vz_std≈0.09、|roll|max≈0.036，抖动较 `094732` 明显改善。摔倒发生在 `t≈56s` 后：用户给了后退 `linear_x<0`（零速仅 33%），x 从 4.37 退回 3.3、y 漂至 0.7，从楼梯顶端滑落，`t≈60.4s` roll→-0.93。操作：楼梯顶端停稳后勿给后退速度；`linear_y=0` 保持较好。

## 2026-06-09 19:50:00 CST 
   - 用户反馈“越来越差”；对比 `084930`/`095217`/`095626`/`095744`：`095626` 在 30° 楼梯 z=2.63m 零速站稳 25s（roll<0.02），`t=64.7s` 因 `angular_z≈0.37` 退出 hold 摔倒；`095744` 因 `linear_y`、后退 `linear_x`、`angular_z` 偏离楼梯。根因：微小摇杆输入会打断 hold。修复：hold 死区（默认线速度 0.08、角速度 0.2）；hold 锁定 x/y/yaw/z；`swingHeight` 恢复 0.18、`touchDownVelocity` 恢复 -0.1。需 `colcon build --packages-select perceptive_legged_control p1_dds_joy_tools_assets` 后重启 launch。