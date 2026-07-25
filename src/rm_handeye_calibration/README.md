# rm_handeye_calibration

ROS 2 Humble 下的 ChArUco 手眼标定包，适用于：

```text
机械臂 base_link
    └── ...
        └── 相机安装 Link（例如 link3）
            └── ZED 相机
```

相机不必安装在末端。只要相机和某个机械臂 Link 刚性连接，就把该 Link 当作手眼标定中的 `hand/gripper frame`。

本程序的所有操作都集成在 OpenCV `imshow` 窗口中，可用鼠标点击，也支持键盘快捷键。

---

## 1. 标定关系

定义：

```text
B = base_frame，例如 base_link
H = hand_frame，即相机刚性安装的机械臂 Link，例如 link3
C = ZED 校正图像的 optical frame
T = 固定在环境中的 ChArUco 标定板
```

每个样本保存：

```text
B_T_H：base 到相机安装 Link
C_T_T：相机 optical frame 到 ChArUco 标定板
```

OpenCV `calibrateHandEye()` 输出：

```text
H_T_C：相机 optical frame 在 hand_frame 下的位姿
```

如果配置了：

```yaml
camera_root_frame: zed_camera_link
```

程序还会通过 ZED 自己发布的内部 TF：

```text
zed_camera_link → optical frame
```

自动换算出 URDF 固定关节需要的：

```text
hand_frame → zed_camera_link
```

---

## 2. 功能

- 订阅 ZED 校正 RGB 图像和对应 `CameraInfo`；
- 实时检测 ChArUco；
- 显示 marker、ChArUco 角点、坐标轴和重投影误差；
- 鼠标点击采样；
- 自动按图像时间戳查询 `base_frame → hand_frame`；
- 拒绝角点过少、重投影误差过大或与已有样本过于相似的采样；
- 同时运行 TSAI、PARK、HORAUD、ANDREFF、DANIILIDIS 五种手眼标定方法；
- 根据固定标定板在 `base_frame` 下的一致性误差自动选择最佳方法；
- 输出 YAML、原始样本和可直接复制进 URDF 的固定关节片段；
- 图像回调只保存最新消息，不在 ROS 回调中运行检测，避免 ZED 高帧率造成消息积压。

---

## 3. 目录结构

```text
rm_handeye_calibration/
├── CMakeLists.txt
├── package.xml
├── README.md
├── config/
│   └── handeye.yaml
├── launch/
│   └── handeye_calibration.launch.py
└── src/
    ├── handeye_calibration_node.cpp
    └── generate_charuco_board.cpp
```

---

## 4. 安装依赖

```bash
sudo apt update
sudo apt install -y \
  libopencv-dev \
  ros-humble-cv-bridge \
  ros-humble-tf2 \
  ros-humble-tf2-ros \
  ros-humble-sensor-msgs \
  ros-humble-geometry-msgs
```

确认 OpenCV 包含 ArUco：

```bash
pkg-config --modversion opencv4
```

---

## 5. 放入工作空间并编译

将整个包复制到：

```text
~/Code/HBUT2025_rm_vision/lc_moveit_ws/src/
```

编译：

```bash
cd ~/Code/HBUT2025_rm_vision/lc_moveit_ws

source /opt/ros/humble/setup.bash

rosdep install \
  --from-paths src \
  --ignore-src \
  -r -y

colcon build \
  --symlink-install \
  --packages-select rm_handeye_calibration

source install/setup.bash
```

---

## 6. 生成 ChArUco 标定板

包内提供了使用同一套 C++ OpenCV 库生成标定板的工具，避免 Python OpenCV 与系统 OpenCV 版本不同导致图案不一致。

生成默认 7×5 标定板：

```bash
ros2 run rm_handeye_calibration generate_charuco_board \
  /tmp/charuco_7x5.png \
  7 5 30 22 DICT_5X5_100 300 10
```

参数依次为：

```text
输出文件
横向方格数
纵向方格数
方格边长，毫米
Marker 边长，毫米
ArUco 字典
DPI
外边距，毫米
```

打印时：

1. 使用 100% 原始比例，不要“适应页面”；
2. 贴到硬质且平整的板上；
3. 用卡尺重新测量打印后的真实方格边长和 Marker 边长；
4. 将实测值写入 `config/handeye.yaml`，单位换成米。

例如实测方格为 29.92 mm：

```yaml
board:
  square_length_m: 0.02992
```

标定板坐标系原点和方向由 OpenCV ChArUcoBoard 定义，不需要手动添加 TF。

---

## 7. 修改配置

打开：

```text
config/handeye.yaml
```

最重要的参数：

```yaml
handeye_calibration:
  ros__parameters:
    image_topic: /zed/zed_node/rgb/color/rect/image
    camera_info_topic: /zed/zed_node/rgb/color/rect/camera_info

    base_frame: base_link

    # 必须改成 ZED 实际刚性安装的机械臂 Link
    hand_frame: link3

    # 建议留空，程序自动使用图像 header.frame_id
    camera_frame: ""

    # 机械臂 URDF 中连接的 ZED 根 Frame
    camera_root_frame: zed_camera_link

    board:
      squares_x: 7
      squares_y: 5
      square_length_m: 0.030
      marker_length_m: 0.022
      dictionary: DICT_5X5_100
```

确认图像 Frame：

```bash
ros2 topic echo \
  /zed/zed_node/rgb/color/rect/image \
  --once \
  --field header
```

---

## 8. 标定前的 TF 要求

标定前必须存在：

```text
base_link → ... → hand_frame
zed_camera_link → ... → ZED optical frame
```

当前粗略的：

```text
hand_frame → zed_camera_link
```

可以暂时保留，它只用于连接 TF 树，不参与 `calibrateHandEye()` 求解。

ZED 不应发布：

```text
map → odom → zed_camera_link
```

但应保留相机自身内部 TF。

检查：

```bash
ros2 run tf2_ros tf2_echo base_link link3
```

```bash
ros2 run tf2_ros tf2_echo \
  zed_camera_link \
  zed_left_camera_optical_frame
```

---

## 9. 启动

先启动机械臂、`robot_state_publisher` 和 ZED，然后运行：

```bash
source /opt/ros/humble/setup.bash
source ~/Code/HBUT2025_rm_vision/lc_moveit_ws/install/setup.bash

ros2 launch rm_handeye_calibration handeye_calibration.launch.py
```

使用其他配置文件：

```bash
ros2 launch rm_handeye_calibration handeye_calibration.launch.py \
  config:=/绝对路径/handeye.yaml
```

---

## 10. OpenCV 窗口操作

窗口底部有按钮：

```text
CAPTURE  保存当前样本
UNDO     删除最后一个样本
SOLVE    运行五种手眼标定方法
SAVE     保存标定结果
CLEAR    清空全部样本
QUIT     退出
```

键盘快捷键：

```text
A：Capture
U：Undo
S：Solve
W：Save
C：Clear
Q / Esc：Quit
```

窗口会显示：

- ArUco Marker；
- ChArUco 角点；
- 标定板坐标轴；
- 角点数量；
- 重投影误差；
- 当前样本数量；
- 标定结果 RMS；
- 拒绝采样的原因。

---

## 11. 正确的采样流程

ChArUco 板必须相对 `base_link` 完全固定。

每一组：

```text
移动机械臂
→ 等待机械臂和相机停止振动
→ 确认窗口角点稳定、重投影误差较小
→ 点击 CAPTURE
```

相机安装在中间 Link 时，只有该 Link 上游的关节能够改变相机位姿。

例如相机装在 `link3`：

```text
base → joint0 → link1 → joint1 → link2 → joint2 → link3
```

应重点改变：

```text
joint0、joint1、joint2
```

只移动 `link3` 下游关节不会改变相机位姿，对标定没有贡献。

建议采集 15～25 组，覆盖：

- 不同距离；
- 画面左、中、右；
- 画面上、中、下；
- 不同俯仰和偏航；
- 至少两个不平行的旋转轴。

不要只做平移，也不要只绕同一个轴旋转。

---

## 12. 采样质量限制

默认设置：

```yaml
quality:
  min_charuco_corners: 12
  max_reprojection_error_px: 1.5
  min_translation_delta_m: 0.030
  min_rotation_delta_deg: 8.0
  min_samples: 10
```

某一姿态与已有姿态同时满足：

```text
平移变化 < 30 mm
旋转变化 < 8°
```

会被判定为重复样本。

如果机械臂可运动范围较小，可适当降低阈值，但不要连续采集几乎相同的姿态。

---

## 13. 性能设计

为了避免 ZED 30～100 FPS 输入导致 GUI 卡顿：

1. ROS 图像回调只替换最新的消息指针，不执行 ChArUco；
2. 不为每一帧做 `cv::Mat::clone()`；
3. ChArUco 检测在独立 UI 线程中按 `max_detection_hz` 限频；
4. 不建立图像消息队列，不处理已经过时的旧帧；
5. TF 只在点击 `CAPTURE` 时查询；
6. OpenCV 线程数量可限制；
7. 只在本地 `imshow`，默认不发布额外 Debug 图像 Topic。

默认：

```yaml
performance:
  detection_scale: 1.0
  max_detection_hz: 15.0
  ui_hz: 30.0
  opencv_threads: 2
```

如果 RTX 2060 电脑上的 CPU 负载仍然较高：

```yaml
performance:
  detection_scale: 0.75
  max_detection_hz: 10.0
```

`detection_scale=0.75` 会同步缩放相机内参，位姿仍能计算，但最终精度可能略低。正式标定优先使用 `1.0`。

---

## 14. 求解方法和选择标准

点击 `SOLVE` 后，程序会运行：

```text
TSAI
PARK
HORAUD
ANDREFF
DANIILIDIS
```

对于每个候选结果，计算每一组样本对应的：

```text
base_T_target = base_T_hand × hand_T_camera × camera_T_target
```

因为 ChArUco 板固定，所有 `base_T_target` 应接近一致。

程序统计：

```text
平移 RMS
旋转 RMS
平移最大误差
旋转最大误差
```

并自动选择综合误差最小的方法。

RMS 很大通常不是“算法选错”，而是：

- 样本姿态变化不足；
- 标定板移动；
- URDF 运动学误差；
- 图像和 TF 时间不同步；
- 方格尺寸错误；
- 图像模糊；
- 相机支架松动。

---

## 15. 输出文件

默认输出目录：

```text
~/handeye_calibration
```

点击 `SAVE` 后生成：

```text
handeye_result_latest.yaml
handeye_result_时间戳.yaml
handeye_samples_latest.yaml
handeye_samples_时间戳.yaml
camera_mount_snippet.urdf
autosave_samples.yaml
```

`handeye_result_latest.yaml` 包含：

```yaml
hand_to_camera_optical:
  parent_frame: link3
  child_frame: zed_left_camera_optical_frame
  xyz: [...]
  quaternion_xyzw: [...]
  rpy_rad: [...]
  matrix: [...]

hand_to_camera_root:
  parent_frame: link3
  child_frame: zed_camera_link
  xyz: [...]
  quaternion_xyzw: [...]
  rpy_rad: [...]
  matrix: [...]
```

真正写进机械臂 URDF 的是：

```text
hand_to_camera_root
```

也可以直接复制：

```text
camera_mount_snippet.urdf
```

中的固定关节。

---

## 16. 写回 URDF

例如输出：

```xml
<joint name="link3_to_zed_camera_link" type="fixed">
  <parent link="link3"/>
  <child link="zed_camera_link"/>
  <origin xyz="... ... ..." rpy="... ... ..."/>
</joint>
```

替换机械臂 URDF 中原来的粗略外参，然后：

```bash
cd ~/Code/HBUT2025_rm_vision/lc_moveit_ws

colcon build \
  --symlink-install \
  --packages-select rm_engineering_robot_description \
                            rm_engineering_moveit_config

source install/setup.bash
```

重启 `robot_state_publisher`、MoveIt 和 ZED。

不要同时额外发布另一条同名静态 TF。

---

## 17. 最终验证

### 验证固定标定板

机械臂带相机运动，但标定板保持不动。使用标定结果计算标定板在 `base_link` 下的位置，应该基本不变。

### 验证 FoundationPose

固定物体不动，让机械臂带相机运动：

```bash
ros2 run tf2_ros tf2_echo \
  base_link \
  foundationpose_object
```

理想结果：

```text
camera optical 下的物体位姿不断变化
base_link 下的物体位姿基本保持不变
```

第一轮实车验证时保持低速，并保留足够安全距离。

---

## 18. 常见问题

### 窗口没有图像

```bash
ros2 topic hz /zed/zed_node/rgb/color/rect/image
```

并检查配置中的 Topic。

### 一直 Waiting for CameraInfo

```bash
ros2 topic echo \
  /zed/zed_node/rgb/color/rect/camera_info \
  --once
```

### 点击 CAPTURE 提示 TF error

检查：

```bash
ros2 run tf2_ros tf2_echo base_link link3
```

还要保证 `/joint_states` 时间戳与图像处于相同 ROS 时钟体系。

### 重投影误差很大

检查：

- 打印后的实际方格尺寸；
- `CameraInfo` 是否与 rectified RGB 配套；
- 标定板是否平整；
- 图像是否运动模糊；
- 角点数量是否过少。

### 标定结果变化很大

增加姿态数量和旋转多样性，删除明显模糊或角点较少的样本，再求解。
