#!/usr/bin/env python3

from __future__ import annotations

import json
import math
import queue
import shutil
import threading
import time
from pathlib import Path
from typing import Optional, Tuple

import cv2
import message_filters
import numpy as np
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_srvs.srv import Trigger


CaptureItem = Tuple[Image, Image, CameraInfo]


class FoundationPoseDataRecorder(Node):
    """Save synchronized RGB/depth frames in FoundationPose demo layout."""

    def __init__(self) -> None:
        super().__init__("foundationpose_data_recorder")

        # ZED topics used by the user's current wrapper version.
        self.declare_parameter(
            "rgb_topic", "/zed/zed_node/rgb/color/rect/image"
        )
        self.declare_parameter(
            "depth_topic", "/zed/zed_node/depth/depth_registered"
        )
        self.declare_parameter(
            "camera_info_topic",
            "/zed/zed_node/depth/depth_registered/camera_info",
        )

        self.declare_parameter(
            "output_dir", str(Path.home() / "foundationpose_capture")
        )
        self.declare_parameter("scene_name", "my_object")
        self.declare_parameter("mesh_source_dir", "")
        self.declare_parameter("sync_queue_size", 10)
        self.declare_parameter("sync_slop_sec", 0.03)
        self.declare_parameter("max_depth_m", 10.0)

        # Recording controls.
        self.declare_parameter("record_on_start", False)
        self.declare_parameter("save_every_n_frames", 1)
        self.declare_parameter("max_recorded_frames", 0)
        self.declare_parameter("writer_queue_size", 60)
        self.declare_parameter("save_depth_preview", False)
        self.declare_parameter("log_every_n_saved", 30)

        self.rgb_topic = str(self.get_parameter("rgb_topic").value)
        self.depth_topic = str(self.get_parameter("depth_topic").value)
        self.camera_info_topic = str(
            self.get_parameter("camera_info_topic").value
        )
        self.output_dir = Path(
            str(self.get_parameter("output_dir").value)
        ).expanduser()
        self.scene_name = str(self.get_parameter("scene_name").value)
        mesh_source_text = str(self.get_parameter("mesh_source_dir").value)
        self.mesh_source_dir: Optional[Path] = (
            Path(mesh_source_text).expanduser()
            if mesh_source_text.strip()
            else None
        )
        self.queue_size = int(self.get_parameter("sync_queue_size").value)
        self.sync_slop_sec = float(
            self.get_parameter("sync_slop_sec").value
        )
        self.max_depth_m = float(self.get_parameter("max_depth_m").value)
        self.record_on_start = bool(
            self.get_parameter("record_on_start").value
        )
        self.save_every_n_frames = int(
            self.get_parameter("save_every_n_frames").value
        )
        self.max_recorded_frames = int(
            self.get_parameter("max_recorded_frames").value
        )
        self.writer_queue_size = int(
            self.get_parameter("writer_queue_size").value
        )
        self.save_depth_preview = bool(
            self.get_parameter("save_depth_preview").value
        )
        self.log_every_n_saved = int(
            self.get_parameter("log_every_n_saved").value
        )

        if not self.scene_name.strip():
            raise ValueError("Parameter 'scene_name' must not be empty.")
        if self.queue_size <= 0:
            raise ValueError("Parameter 'sync_queue_size' must be positive.")
        if self.sync_slop_sec < 0.0:
            raise ValueError("Parameter 'sync_slop_sec' must be non-negative.")
        if self.save_every_n_frames <= 0:
            raise ValueError("Parameter 'save_every_n_frames' must be >= 1.")
        if self.max_recorded_frames < 0:
            raise ValueError("Parameter 'max_recorded_frames' must be >= 0.")
        if self.writer_queue_size <= 0:
            raise ValueError("Parameter 'writer_queue_size' must be positive.")
        if self.log_every_n_saved <= 0:
            raise ValueError("Parameter 'log_every_n_saved' must be >= 1.")

        self.scene_dir = self.output_dir / self.scene_name
        self.rgb_dir = self.scene_dir / "rgb"
        self.depth_dir = self.scene_dir / "depth"
        self.mask_dir = self.scene_dir / "masks"
        self.mesh_dir = self.scene_dir / "mesh"
        self.meta_dir = self.scene_dir / "meta"
        self.depth_preview_dir = self.scene_dir / "depth_vis"

        for directory in (
            self.rgb_dir,
            self.depth_dir,
            self.mask_dir,
            self.mesh_dir,
            self.meta_dir,
        ):
            directory.mkdir(parents=True, exist_ok=True)
        if self.save_depth_preview:
            self.depth_preview_dir.mkdir(parents=True, exist_ok=True)

        self._copy_mesh_assets()
        self.next_index = self._find_next_index()

        self.bridge = CvBridge()
        self.state_lock = threading.Lock()
        self.camera_info: Optional[CameraInfo] = None
        self.pending_single_capture = False
        self.recording = self.record_on_start
        self.synced_frame_counter = 0
        self.session_enqueued_frames = 0
        self.total_enqueued_frames = 0
        self.total_saved_frames = 0
        self.total_dropped_frames = 0
        self.last_drop_warning_time = 0.0

        # Disk writes are performed outside the ROS callback thread. This lowers
        # the probability of blocking image subscription callbacks.
        self.write_queue: queue.Queue[CaptureItem] = queue.Queue(
            maxsize=self.writer_queue_size
        )
        self.writer_stop_event = threading.Event()
        self.writer_thread = threading.Thread(
            target=self._writer_loop,
            name="foundationpose_writer",
            daemon=True,
        )
        self.writer_thread.start()

        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            self.camera_info_topic,
            self._camera_info_callback,
            qos_profile_sensor_data,
        )

        self.rgb_sub = message_filters.Subscriber(
            self,
            Image,
            self.rgb_topic,
            qos_profile=qos_profile_sensor_data,
        )
        self.depth_sub = message_filters.Subscriber(
            self,
            Image,
            self.depth_topic,
            qos_profile=qos_profile_sensor_data,
        )
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.rgb_sub, self.depth_sub],
            queue_size=self.queue_size,
            slop=self.sync_slop_sec,
        )
        self.sync.registerCallback(self._synced_image_callback)

        self.capture_service = self.create_service(
            Trigger,
            "~/capture",
            self._capture_service_callback,
        )
        self.start_service = self.create_service(
            Trigger,
            "~/start_recording",
            self._start_recording_callback,
        )
        self.stop_service = self.create_service(
            Trigger,
            "~/stop_recording",
            self._stop_recording_callback,
        )
        self.status_service = self.create_service(
            Trigger,
            "~/status",
            self._status_callback,
        )

        self.get_logger().info("FoundationPose RGB-D recorder started.")
        self.get_logger().info(f"RGB topic:        {self.rgb_topic}")
        self.get_logger().info(f"Depth topic:      {self.depth_topic}")
        self.get_logger().info(f"CameraInfo topic: {self.camera_info_topic}")
        self.get_logger().info(f"Scene directory:  {self.scene_dir}")
        self.get_logger().info(
            "Services: ~/capture, ~/start_recording, ~/stop_recording, ~/status"
        )
        if self.recording:
            self.get_logger().info(
                "Continuous recording is active from startup."
            )

    def _copy_mesh_assets(self) -> None:
        if self.mesh_source_dir is None:
            return
        if not self.mesh_source_dir.exists():
            self.get_logger().warning(
                f"mesh_source_dir does not exist: {self.mesh_source_dir}"
            )
            return
        if not self.mesh_source_dir.is_dir():
            self.get_logger().warning(
                "mesh_source_dir must be a directory containing OBJ/MTL/textures."
            )
            return

        try:
            shutil.copytree(
                self.mesh_source_dir,
                self.mesh_dir,
                dirs_exist_ok=True,
            )
            self.get_logger().info(
                f"Copied mesh assets from {self.mesh_source_dir} "
                f"to {self.mesh_dir}"
            )
        except OSError as exc:
            self.get_logger().error(f"Failed to copy mesh assets: {exc}")

    def _find_next_index(self) -> int:
        indices = []
        for path in self.rgb_dir.glob("*.png"):
            try:
                indices.append(int(path.stem))
            except ValueError:
                continue
        return max(indices, default=-1) + 1

    def _camera_info_callback(self, msg: CameraInfo) -> None:
        with self.state_lock:
            self.camera_info = msg

    def _capture_service_callback(
        self,
        request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        del request
        with self.state_lock:
            if self.pending_single_capture:
                response.success = False
                response.message = "A one-frame capture is already pending."
                return response
            self.pending_single_capture = True

        response.success = True
        response.message = (
            "Capture armed. The next synchronized RGB/depth pair will be queued."
        )
        return response

    def _start_recording_callback(
        self,
        request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        del request
        with self.state_lock:
            if self.recording:
                response.success = False
                response.message = "Continuous recording is already active."
                return response

            self.recording = True
            self.synced_frame_counter = 0
            self.session_enqueued_frames = 0

        response.success = True
        response.message = (
            "Continuous recording started. "
            f"Saving every {self.save_every_n_frames} synchronized frame(s)."
        )
        return response

    def _stop_recording_callback(
        self,
        request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        del request
        with self.state_lock:
            was_recording = self.recording
            self.recording = False
            session_frames = self.session_enqueued_frames

        response.success = was_recording
        response.message = (
            f"Continuous recording stopped; {session_frames} frame(s) queued "
            f"in this session. Writer backlog={self.write_queue.qsize()}."
            if was_recording
            else "Continuous recording was not active."
        )
        return response

    def _status_callback(
        self,
        request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        del request
        with self.state_lock:
            recording = self.recording
            pending = self.pending_single_capture
            session_enqueued = self.session_enqueued_frames
            total_enqueued = self.total_enqueued_frames
            saved = self.total_saved_frames
            dropped = self.total_dropped_frames

        response.success = True
        response.message = (
            f"recording={recording}, pending_single={pending}, "
            f"session_enqueued={session_enqueued}, "
            f"total_enqueued={total_enqueued}, saved={saved}, "
            f"dropped={dropped}, writer_backlog={self.write_queue.qsize()}, "
            f"next_index={self.next_index}"
        )
        return response

    def _synced_image_callback(self, rgb_msg: Image, depth_msg: Image) -> None:
        with self.state_lock:
            camera_info = self.camera_info
            one_shot = self.pending_single_capture
            recording = self.recording

            if not one_shot and not recording:
                return

            if camera_info is None:
                self.get_logger().warning(
                    "Capture requested, but CameraInfo has not been received yet."
                )
                return

            save_for_recording = False
            if recording:
                frame_number = self.synced_frame_counter
                self.synced_frame_counter += 1
                save_for_recording = (
                    frame_number % self.save_every_n_frames == 0
                )

            should_enqueue = one_shot or save_for_recording
            if not should_enqueue:
                return

        try:
            self.write_queue.put_nowait((rgb_msg, depth_msg, camera_info))
        except queue.Full:
            with self.state_lock:
                self.total_dropped_frames += 1
                # Keep a one-shot request armed so it can be retried.
                if one_shot:
                    self.pending_single_capture = True
            self._warn_queue_full()
            return

        with self.state_lock:
            if one_shot:
                self.pending_single_capture = False
            if save_for_recording:
                self.session_enqueued_frames += 1
            self.total_enqueued_frames += 1

            if (
                self.max_recorded_frames > 0
                and self.session_enqueued_frames >= self.max_recorded_frames
            ):
                self.recording = False
                self.get_logger().info(
                    "Reached max_recorded_frames="
                    f"{self.max_recorded_frames}; recording stopped."
                )

    def _warn_queue_full(self) -> None:
        now = time.monotonic()
        if now - self.last_drop_warning_time >= 1.0:
            self.last_drop_warning_time = now
            self.get_logger().warning(
                "Writer queue is full; an RGB-D frame was dropped. "
                "Use a faster disk, disable depth previews, increase "
                "save_every_n_frames, or increase writer_queue_size."
            )

    def _writer_loop(self) -> None:
        while not self.writer_stop_event.is_set() or not self.write_queue.empty():
            try:
                rgb_msg, depth_msg, camera_info = self.write_queue.get(
                    timeout=0.1
                )
            except queue.Empty:
                continue

            try:
                frame_id, valid_ratio, time_difference_ms = self._save_capture(
                    rgb_msg,
                    depth_msg,
                    camera_info,
                )
                with self.state_lock:
                    self.total_saved_frames += 1
                    saved_count = self.total_saved_frames

                if saved_count == 1 or saved_count % self.log_every_n_saved == 0:
                    self.get_logger().info(
                        f"Saved {saved_count} frame(s); latest={frame_id}, "
                        f"valid_depth={valid_ratio:.1%}, "
                        f"RGB/depth_dt={time_difference_ms:.2f} ms, "
                        f"writer_backlog={self.write_queue.qsize()}"
                    )
            except (CvBridgeError, ValueError, OSError, RuntimeError) as exc:
                self.get_logger().error(f"Capture write failed: {exc}")
            except Exception as exc:
                self.get_logger().error(
                    f"Unexpected capture write failure: {exc!r}"
                )
            finally:
                self.write_queue.task_done()

    def _save_capture(
        self,
        rgb_msg: Image,
        depth_msg: Image,
        camera_info: CameraInfo,
    ) -> Tuple[str, float, float]:
        bgr = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding="bgr8")
        depth_raw = self.bridge.imgmsg_to_cv2(
            depth_msg,
            desired_encoding="passthrough",
        )

        if bgr.ndim != 3 or bgr.shape[2] != 3:
            raise ValueError(f"RGB image has unexpected shape: {bgr.shape}")

        depth_m = self._depth_to_meters(depth_raw, depth_msg.encoding)

        if depth_m.ndim != 2:
            raise ValueError(f"Depth image has unexpected shape: {depth_m.shape}")
        if bgr.shape[:2] != depth_m.shape:
            raise ValueError(
                "RGB/depth size mismatch: "
                f"RGB={bgr.shape[:2]}, depth={depth_m.shape}. "
                "Use registered depth with the rectified color image."
            )

        height, width = depth_m.shape
        if camera_info.width not in (0, width) or camera_info.height not in (
            0,
            height,
        ):
            raise ValueError(
                "CameraInfo resolution does not match saved images: "
                f"CameraInfo={camera_info.width}x{camera_info.height}, "
                f"images={width}x{height}."
            )

        K = np.asarray(camera_info.k, dtype=np.float64).reshape(3, 3)
        if not np.all(np.isfinite(K)):
            raise ValueError("CameraInfo K contains NaN or Inf.")
        if K[0, 0] <= 0.0 or K[1, 1] <= 0.0:
            raise ValueError(f"Invalid focal lengths in K:\n{K}")

        valid = np.isfinite(depth_m) & (depth_m > 0.0)
        if self.max_depth_m > 0.0:
            valid &= depth_m <= self.max_depth_m

        clean_depth_m = np.zeros_like(depth_m, dtype=np.float32)
        clean_depth_m[valid] = depth_m[valid].astype(np.float32)

        # FoundationPose YcbineoatReader reads uint16 PNG in millimeters and
        # divides by 1000, so convert ZED's meter depth to that exact format.
        depth_mm = np.zeros_like(clean_depth_m, dtype=np.uint16)
        if np.any(valid):
            millimeters = np.rint(clean_depth_m[valid] * 1000.0)
            millimeters = np.clip(millimeters, 1.0, 65535.0)
            depth_mm[valid] = millimeters.astype(np.uint16)

        frame_id = f"{self.next_index:06d}"
        rgb_path = self.rgb_dir / f"{frame_id}.png"
        depth_path = self.depth_dir / f"{frame_id}.png"
        mask_path = self.mask_dir / f"{frame_id}.png"
        k_path = self.scene_dir / "cam_K.txt"
        meta_path = self.meta_dir / f"{frame_id}.json"

        if not cv2.imwrite(str(rgb_path), bgr):
            raise OSError(f"cv2.imwrite failed for {rgb_path}")
        if not cv2.imwrite(str(depth_path), depth_mm):
            raise OSError(f"cv2.imwrite failed for {depth_path}")

        # A blank mask is created for each frame. For the official demo, edit
        # the first frame's mask only; subsequent frames use track_one().
        blank_mask = np.zeros((height, width), dtype=np.uint8)
        if not cv2.imwrite(str(mask_path), blank_mask):
            raise OSError(f"cv2.imwrite failed for {mask_path}")

        # K is constant for this scene. Rewriting it is harmless and keeps the
        # scene self-contained even if recording starts with a non-zero index.
        np.savetxt(str(k_path), K, fmt="%.12g")

        if self.save_depth_preview:
            preview = self._make_depth_preview(clean_depth_m, valid)
            preview_path = self.depth_preview_dir / f"{frame_id}.png"
            if not cv2.imwrite(str(preview_path), preview):
                raise OSError(f"cv2.imwrite failed for {preview_path}")

        rgb_stamp_ns = self._stamp_to_ns(rgb_msg)
        depth_stamp_ns = self._stamp_to_ns(depth_msg)
        time_difference_ms = abs(rgb_stamp_ns - depth_stamp_ns) / 1e6

        valid_count = int(np.count_nonzero(valid))
        pixel_count = int(valid.size)
        valid_ratio = valid_count / pixel_count if pixel_count else 0.0

        metadata = {
            "frame_id": frame_id,
            "rgb_topic": self.rgb_topic,
            "depth_topic": self.depth_topic,
            "camera_info_topic": self.camera_info_topic,
            "rgb_encoding": rgb_msg.encoding,
            "depth_encoding": depth_msg.encoding,
            "rgb_frame": rgb_msg.header.frame_id,
            "depth_frame": depth_msg.header.frame_id,
            "camera_info_frame": camera_info.header.frame_id,
            "rgb_stamp_ns": rgb_stamp_ns,
            "depth_stamp_ns": depth_stamp_ns,
            "timestamp_difference_ms": time_difference_ms,
            "width": width,
            "height": height,
            "valid_depth_ratio": valid_ratio,
            "valid_depth_min_m": (
                float(clean_depth_m[valid].min()) if valid_count else None
            ),
            "valid_depth_max_m": (
                float(clean_depth_m[valid].max()) if valid_count else None
            ),
            "K": K.tolist(),
        }
        meta_path.write_text(
            json.dumps(metadata, indent=2, ensure_ascii=False),
            encoding="utf-8",
        )

        self.next_index += 1
        return frame_id, valid_ratio, time_difference_ms

    @staticmethod
    def _stamp_to_ns(msg: Image) -> int:
        return (
            int(msg.header.stamp.sec) * 1_000_000_000
            + int(msg.header.stamp.nanosec)
        )

    @staticmethod
    def _depth_to_meters(depth: np.ndarray, encoding: str) -> np.ndarray:
        normalized_encoding = encoding.upper()

        if normalized_encoding == "32FC1":
            return np.asarray(depth, dtype=np.float32)

        if normalized_encoding in ("16UC1", "MONO16"):
            return np.asarray(depth, dtype=np.float32) * 0.001

        raise ValueError(
            f"Unsupported depth encoding '{encoding}'. "
            "Expected 32FC1 (meters) or 16UC1/mono16 (millimeters)."
        )

    @staticmethod
    def _make_depth_preview(
        depth_m: np.ndarray,
        valid: np.ndarray,
    ) -> np.ndarray:
        preview = np.zeros(depth_m.shape, dtype=np.uint8)
        if not np.any(valid):
            return preview

        values = depth_m[valid]
        near = float(np.percentile(values, 2.0))
        far = float(np.percentile(values, 98.0))
        if not math.isfinite(near) or not math.isfinite(far) or far <= near:
            near = float(values.min())
            far = float(values.max())

        if far <= near:
            preview[valid] = 255
            return preview

        normalized = (depth_m - near) / (far - near)
        normalized = np.clip(normalized, 0.0, 1.0)
        preview[valid] = np.rint(
            (1.0 - normalized[valid]) * 255.0
        ).astype(np.uint8)
        return preview

    def destroy_node(self) -> bool:
        with self.state_lock:
            self.recording = False
            self.pending_single_capture = False

        self.writer_stop_event.set()
        self.writer_thread.join(timeout=10.0)
        if self.writer_thread.is_alive():
            self.get_logger().warning(
                "Writer thread did not finish before node shutdown."
            )
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node: Optional[FoundationPoseDataRecorder] = None

    try:
        node = FoundationPoseDataRecorder()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
