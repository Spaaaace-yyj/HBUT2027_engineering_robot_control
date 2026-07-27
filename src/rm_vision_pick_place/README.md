# rm_vision_pick_place

流程：启用 FoundationPose → 重新初始化 → 稳定位姿 → 冻结 → 加入 PlanningScene → MTC 虚拟抓取/附着/搬运/分离。

```bash
ros2 service call /vision_pick_place/start std_srvs/srv/Trigger "{}"
```

`auto_execute` 默认 false。先在 RViz 中验证完整 Task，再允许实车执行。
