# foundationpose_shm_bridge 0.2.0

功能：

1. 订阅并同步 ZED RGB、注册深度和 CameraInfo。
2. 将 RGB-D 写入 POSIX 共享内存，供 Python 3.11 FoundationPose Worker 使用。
3. 通过共享内存向 Worker 发送 ROS 服务命令和动态参数。
4. 读取 Worker 返回的位姿、Debug 图、Mask、状态。
5. 发布 ROS Pose、Image、状态 Topic 和 TF。

## ROS 接口

### 服务

```text
/foundationpose/set_enabled      std_srvs/srv/SetBool
/foundationpose/reinitialize     std_srvs/srv/Trigger
/foundationpose/shutdown_worker  std_srvs/srv/Trigger
/foundationpose/get_status       std_srvs/srv/Trigger
```

暂停估计但保留模型和显存：

```bash
ros2 service call /foundationpose/set_enabled std_srvs/srv/SetBool "{data: false}"
```

重新开启；Worker 会自动重新运行 YOLO 和 register：

```bash
ros2 service call /foundationpose/set_enabled std_srvs/srv/SetBool "{data: true}"
```

强制重新初始化：

```bash
ros2 service call /foundationpose/reinitialize std_srvs/srv/Trigger "{}"
```

完全退出 Python Worker并释放显存：

```bash
ros2 service call /foundationpose/shutdown_worker std_srvs/srv/Trigger "{}"
```

### 发布 Topic

```text
/foundationpose/pose         geometry_msgs/msg/PoseStamped
/foundationpose/debug_image  sensor_msgs/msg/Image，bgr8
/foundationpose/mask         sensor_msgs/msg/Image，mono8
/foundationpose/status       std_msgs/msg/String
/foundationpose/state        std_msgs/msg/UInt8
/tf                          camera frame -> foundationpose_object
```

状态值：

```text
0 DISABLED
1 WAITING_FOR_FRAME
2 WAITING_FOR_MASK
3 REGISTERING
4 TRACKING
5 ERROR
6 SHUTDOWN
```

## 动态参数

```bash
ros2 param set /foundationpose_bridge yolo_conf 0.4
ros2 param set /foundationpose_bridge mask_threshold 0.5
ros2 param set /foundationpose_bridge track_refine_iter 3
ros2 param set /foundationpose_bridge publish_debug_image false
ros2 param set /foundationpose_bridge publish_tf true
```

修改分割和注册相关参数时，Bridge 会自动请求重新初始化。修改 `track_refine_iter`、深度范围或发布开关时直接在后续帧生效。

## 编译

```bash
cd ~/Code/HBUT2025_rm_vision/lc_moveit_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-select foundationpose_shm_bridge
source install/setup.bash
```

## 分开启动

终端一：

```bash
ros2 launch foundationpose_shm_bridge foundationpose_bridge.launch.py
```

终端二：

```bash
conda activate foundationpose
cd ~/Code/FoundationPose
python foundationpose_server/foundationpose_worker.py
```

## 一键启动

默认路径已经按当前用户目录填写，可通过 launch 参数覆盖：

```bash
ros2 launch foundationpose_shm_bridge foundationpose_system.launch.py \
  conda_env:=foundationpose \
  worker_script:=/home/spaaaaace/Code/FoundationPose/foundationpose_server/foundationpose_worker.py \
  weights_file:=/home/spaaaaace/Code/FoundationPose/foundationpose_server/weiget/best.pt \
  mesh_file:=/home/spaaaaace/Code/FoundationPose/foundationpose_server/obj_model/real_color_obj.obj \
  mesh_scale:=0.001 \
  show_window:=0
```

## TF

默认父坐标系为空时，Bridge 使用 ZED RGB 消息的 `header.frame_id`。子坐标系默认为：

```text
foundationpose_object
```

验证：

```bash
ros2 run tf2_ros tf2_echo <zed_rgb_frame> foundationpose_object
```
