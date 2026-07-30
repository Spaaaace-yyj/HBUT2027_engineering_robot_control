# rm_vision_pick_place 0.2

当前版本只打通以下流程：

```text
FoundationPose reinitialize
→ 稳定位姿并冻结到 planning_frame
→ 加入圆柱 CollisionObject
→ 生成圆柱侧面候选
→ ComputeIK
→ Connect 到预接近状态
→ 直线接近
→ 允许接触
→ 虚拟 attach
```

## 关键修正

1. MTC 节点通过 launch 显式加载：
   - `robot_description`
   - `robot_description_semantic`
   - `robot_description_kinematics`
   - OMPL planning pipeline
   - joint limits
2. IK solver 不存在时立即失败，不再为所有候选重复刷屏。
3. `ik_timeout_sec` 独立可调，默认 0.20 s。
4. 抓取候选和圆柱 CollisionObject 使用同一个 `object_to_cylinder_*` 变换，删除只对碰撞体硬编码旋转 90°的不一致行为。
5. 实车默认低速，且 `auto_execute=false`。

## 编译

```bash
cd ~/Code/HBUT2025_rm_vision/lc_moveit_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select rm_vision_pick_place
source install/setup.bash
```

## 启动

先启动实车的 `robot_state_publisher`、`ros2_control`、`move_group`、ZED 和 FoundationPose Bridge/Worker。

MTC 节点必须用 launch 启动：

```bash
ros2 launch rm_vision_pick_place vision_pick_place.launch.py \
  hardware_type:=real \
  use_sim_time:=false \
  auto_execute:=false
```

不要用裸 `ros2 run` 做规划测试；它不会自动向本节点注入完整 MoveIt 运动学配置。

## 启动前检查

```bash
ros2 param get /vision_pick_place \
  robot_description_kinematics.rm_robot_arm.kinematics_solver
```

应看到：

```text
kdl_kinematics_plugin/KDLKinematicsPlugin
```

开始任务：

```bash
ros2 service call /vision_pick_place/start std_srvs/srv/Trigger "{}"
```

查看状态：

```bash
ros2 topic echo /vision_pick_place/status
```

## 第一次实车测试

保持：

```yaml
auto_execute: false
max_velocity_scaling: 0.05
max_acceleration_scaling: 0.05
side_grasp_azimuth_samples: 4
side_grasp_roll_samples: 1
```

先在 RViz 检查候选姿态和完整 Task 解。确认后再启动：

```bash
ros2 launch rm_vision_pick_place vision_pick_place.launch.py \
  hardware_type:=real \
  auto_execute:=true
```

## 坐标约定

`object_to_cylinder_rpy` 定义 FoundationPose mesh frame 到规范圆柱 frame 的固定变换。规范圆柱 frame 的 `+Z` 是圆柱轴。

当前默认：

```yaml
object_to_cylinder_rpy: [1.57079632679, 0.0, 0.0]
```

这是对原代码中“碰撞圆柱绕 X 轴旋转 90°”行为的统一化。若 RViz 中圆柱轴不正确，应只调整这个参数；抓取候选与碰撞体会同步变化。

当前还假设 `end_effect_link +X` 是接近方向。若真实 TCP 的前向轴不是 `+X`，应在 URDF 中增加固定 `grasp_tcp`，让其 `+X` 指向工具前方，然后把：

```yaml
end_effector_frame: grasp_tcp
```

## MoveGroup 执行能力

实际执行 MTC solution 时，`move_group` 需要加载：

```text
move_group/ExecuteTaskSolutionCapability
```

没有物理夹爪时，attach 只改变 MoveIt PlanningScene；现实物体不会被抓住。
