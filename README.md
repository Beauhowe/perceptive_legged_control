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
