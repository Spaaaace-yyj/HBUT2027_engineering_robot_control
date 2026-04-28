# HBUT_engineering_robot_control

湖北工业大学力创RM战队工程机器人ros2控制项目

项目基于moveit2框架实现

## 环境要求

OS:Ubuntu22.04

ROS2:Humble

##  编译运行

```bash
#编译
bash build.sh
#运行
source install/setup.bash 
ros2 launch robot_moveit_config demo.launch.py
```

启动servo服务

```bash
ros2 service call /servo_node/start_servo std_srvs/srv/Trigger "{}"
```

