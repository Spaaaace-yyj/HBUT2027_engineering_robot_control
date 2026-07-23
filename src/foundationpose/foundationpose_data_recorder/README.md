# foundationpose_data_recorder 0.2.0

ROS 2 Humble Python node for recording synchronized ZED RGB and registered
Depth images in the directory layout used by the FoundationPose official demo.

## Output layout

```text
<output_dir>/<scene_name>/
├── cam_K.txt
├── rgb/
│   ├── 000000.png
│   ├── 000001.png
│   └── ...
├── depth/
│   ├── 000000.png       # uint16 millimeters
│   ├── 000001.png
│   └── ...
├── masks/
│   ├── 000000.png       # blank mask template
│   ├── 000001.png
│   └── ...
├── mesh/
│   └── ...              # OBJ/MTL/textures, optionally copied
├── depth_vis/            # optional visualization only
└── meta/
    ├── 000000.json
    └── ...
```

FoundationPose uses `masks/000000.png` for the initial `register()` call and
then uses `track_one()` for the following RGB-D frames. Edit the first mask in
Photoshop: target=255, background=0.

## Build

```bash
mkdir -p ~/foundationpose_ros_ws/src
cp -r foundationpose_data_recorder ~/foundationpose_ros_ws/src/

cd ~/foundationpose_ros_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## Start the recorder

The launch file defaults match this ZED wrapper topic layout:

```text
/zed/zed_node/rgb/color/rect/image
/zed/zed_node/depth/depth_registered
/zed/zed_node/depth/depth_registered/camera_info
```

Start without recording immediately:

```bash
#start zedcamera
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=<camera_model>

ros2 launch foundationpose_data_recorder record_zed.launch.py \
  output_dir:=/home/spaaaaace/Code/HBUT2025_rm_vision/lc_moveit_ws/src/foundationpose_data_recorder/output \
  scene_name:=my_object \
  mesh_source_dir:=/home/spaaaaace/Code/HBUT2025_rm_vision/lc_moveit_ws/src/foundationpose_data_recorder/mesh_temp \
  max_depth_m:=3.0
```

## Record continuously using services

Start:

```bash
ros2 service call \
  /foundationpose_data_recorder/start_recording \
  std_srvs/srv/Trigger "{}"
```

Stop:

```bash
ros2 service call \
  /foundationpose_data_recorder/stop_recording \
  std_srvs/srv/Trigger "{}"
```

Check status and dropped frames:

```bash
ros2 service call \
  /foundationpose_data_recorder/status \
  std_srvs/srv/Trigger "{}"
```

Save one frame without starting a recording session:

```bash
ros2 service call \
  /foundationpose_data_recorder/capture \
  std_srvs/srv/Trigger "{}"
```

## Record automatically from node startup

Save every synchronized frame:

```bash
ros2 launch foundationpose_data_recorder record_zed.launch.py \
  output_dir:=$HOME/Code/FoundationPose/demo_data \
  scene_name:=my_object \
  mesh_source_dir:=$HOME/Models/my_object \
  max_depth_m:=3.0 \
  record_on_start:=true \
  save_every_n_frames:=1 \
  max_recorded_frames:=0 \
  save_depth_preview:=false
```

Parameters:

- `record_on_start`: begin continuous recording immediately.
- `save_every_n_frames=1`: save every synchronized pair.
- `save_every_n_frames=3`: for a 30 FPS camera, save about 10 FPS.
- `max_recorded_frames=0`: unlimited until the stop service is called.
- `max_recorded_frames=300`: stop after 300 queued frames.
- `writer_queue_size=60`: maximum pending disk-write frames.
- `save_depth_preview=false`: recommended for continuous recording because
  preview generation and extra PNG writes reduce throughput.

If `status` reports `dropped > 0` or a growing `writer_backlog`, the disk cannot
keep up. Increase `save_every_n_frames`, use a faster SSD, reduce resolution, or
increase `writer_queue_size` for short bursts.

## Run FoundationPose

```bash
conda activate foundationpose
cd ~/Code/FoundationPose

python run_demo.py \
  --mesh_file \
  $HOME/Code/FoundationPose/demo_data/my_object/mesh/my_object.obj \
  --test_scene_dir \
  $HOME/Code/FoundationPose/demo_data/my_object \
  --debug 2
```
