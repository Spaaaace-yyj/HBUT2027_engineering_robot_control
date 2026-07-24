#include "foundationpose_shm_bridge/control_shm_writer.hpp"
#include "foundationpose_shm_bridge/result_shm_reader.hpp"
#include "foundationpose_shm_bridge/shared_rgbd_writer.hpp"

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <opencv2/core.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <iterator>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace foundationpose_shm_bridge
{
    class FoundationPoseBridgeNode final : public rclcpp::Node
    {
    public:
        using Image = sensor_msgs::msg::Image;
        using ApproximatePolicy =
        message_filters::sync_policies::ApproximateTime<Image, Image>;

        FoundationPoseBridgeNode()
            : Node("foundationpose_bridge")
        {
            declareAndReadParameters();
            validateStartupParameters();

            //共享内存通讯
            control_writer_ = std::make_unique<ControlShmWriter>(control_shm_name_, true);
            result_reader_ = std::make_unique<ResultShmReader>(result_shm_name_);

            const auto pose_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
            const auto image_qos = rclcpp::SensorDataQoS();
            const auto status_qos = rclcpp::QoS(1).reliable().transient_local();

            //foundationpose结果发布
            pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
                "/foundationpose/pose", pose_qos);
            debug_image_pub_ = create_publisher<sensor_msgs::msg::Image>(
                "/foundationpose/debug_image", image_qos);
            mask_pub_ = create_publisher<sensor_msgs::msg::Image>(
                "/foundationpose/mask", image_qos);
            status_pub_ = create_publisher<std_msgs::msg::String>(
                "/foundationpose/status", status_qos);
            state_pub_ = create_publisher<std_msgs::msg::UInt8>(
                "/foundationpose/state", status_qos);

            tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

            //控制服务
            set_enabled_service_ = create_service<std_srvs::srv::SetBool>(
                "/foundationpose/set_enabled",
                std::bind(
                    &FoundationPoseBridgeNode::setEnabledService,
                    this,
                    std::placeholders::_1,
                    std::placeholders::_2));

            reinitialize_service_ = create_service<std_srvs::srv::Trigger>(
                "/foundationpose/reinitialize",
                std::bind(
                    &FoundationPoseBridgeNode::reinitializeService,
                    this,
                    std::placeholders::_1,
                    std::placeholders::_2));

            shutdown_service_ = create_service<std_srvs::srv::Trigger>(
                "/foundationpose/shutdown_worker",
                std::bind(
                    &FoundationPoseBridgeNode::shutdownWorkerService,
                    this,
                    std::placeholders::_1,
                    std::placeholders::_2));

            get_status_service_ = create_service<std_srvs::srv::Trigger>(
                "/foundationpose/get_status",
                std::bind(
                    &FoundationPoseBridgeNode::getStatusService,
                    this,
                    std::placeholders::_1,
                    std::placeholders::_2));

            parameter_callback_handle_ = add_on_set_parameters_callback(
                std::bind(
                    &FoundationPoseBridgeNode::onSetParameters,
                    this,
                    std::placeholders::_1));

            setupImageSubscriptions();

            status_timer_ = create_wall_timer(
                std::chrono::duration<double>(log_period_sec_),
                std::bind(&FoundationPoseBridgeNode::logStatus, this));

            result_timer_ = create_wall_timer(
                std::chrono::duration<double>(1.0 / result_poll_hz_),
                std::bind(&FoundationPoseBridgeNode::pollWorkerResult, this));

            publishControlSnapshot();

            RCLCPP_INFO(get_logger(), "FoundationPose ROS bridge started");
            RCLCPP_INFO(get_logger(), "RGB topic: %s", rgb_topic_.c_str());
            RCLCPP_INFO(get_logger(), "Depth topic: %s", depth_topic_.c_str());
            RCLCPP_INFO(get_logger(), "CameraInfo topic: %s", camera_info_topic_.c_str());
            RCLCPP_INFO(get_logger(), "RGB-D SHM: %s", rgbd_shm_name_.c_str());
            RCLCPP_INFO(get_logger(), "Control SHM: %s", control_shm_name_.c_str());
            RCLCPP_INFO(get_logger(), "Result SHM: %s", result_shm_name_.c_str());
        }

    private:
        void declareAndReadParameters()
        {
            rgb_topic_ = declare_parameter<std::string>(
                "rgb_topic", "/zed/zed_node/rgb/color/rect/image");
            depth_topic_ = declare_parameter<std::string>(
                "depth_topic", "/zed/zed_node/depth/depth_registered");
            camera_info_topic_ = declare_parameter<std::string>(
                "camera_info_topic", "/zed/zed_node/rgb/color/rect/camera_info");

            rgbd_shm_name_ = declare_parameter<std::string>(
                "rgbd_shm_name", "/foundationpose_rgbd");
            control_shm_name_ = declare_parameter<std::string>(
                "control_shm_name", "/foundationpose_control");
            result_shm_name_ = declare_parameter<std::string>(
                "result_shm_name", "/foundationpose_result");

            sync_queue_size_ = declare_parameter<int>("sync_queue_size", 10);
            sync_slop_sec_ = declare_parameter<double>("sync_slop_sec", 0.01);
            unlink_rgbd_on_exit_ = declare_parameter<bool>("unlink_rgbd_on_exit", false);
            log_period_sec_ = declare_parameter<double>("log_period_sec", 2.0);
            result_poll_hz_ = declare_parameter<double>("result_poll_hz", 100.0);

            enabled_ = declare_parameter<bool>("enabled", true);
            yolo_conf_ = declare_parameter<double>("yolo_conf", 0.5);
            mask_threshold_ = declare_parameter<double>("mask_threshold", 0.5);
            yolo_imgsz_ = declare_parameter<int>("yolo_imgsz", 640);
            est_refine_iter_ = declare_parameter<int>("est_refine_iter", 5);
            track_refine_iter_ = declare_parameter<int>("track_refine_iter", 2);
            min_depth_m_ = declare_parameter<double>("min_depth_m", 0.05);
            max_depth_m_ = declare_parameter<double>("max_depth_m", 5.0);
            min_mask_pixels_ = declare_parameter<int>("min_mask_pixels", 500);
            min_valid_depth_ratio_ = declare_parameter<double>("min_valid_depth_ratio", 0.05);
            mask_close_kernel_ = declare_parameter<int>("mask_close_kernel", 0);

            publish_pose_ = declare_parameter<bool>("publish_pose", true);
            publish_tf_ = declare_parameter<bool>("publish_tf", true);
            publish_debug_image_ = declare_parameter<bool>("publish_debug_image", true);
            publish_mask_ = declare_parameter<bool>("publish_mask", true);

            camera_frame_id_ = declare_parameter<std::string>("camera_frame_id", "");
            object_frame_id_ = declare_parameter<std::string>(
                "object_frame_id", "foundationpose_object");
        }

        void validateStartupParameters() const
        {
            if (sync_queue_size_ < 2)
            {
                throw std::invalid_argument("sync_queue_size must be at least 2");
            }
            if (sync_slop_sec_ < 0.0)
            {
                throw std::invalid_argument("sync_slop_sec must be non-negative");
            }
            if (log_period_sec_ <= 0.0 || result_poll_hz_ <= 0.0)
            {
                throw std::invalid_argument("timer frequencies must be positive");
            }
            if (object_frame_id_.empty())
            {
                throw std::invalid_argument("object_frame_id cannot be empty");
            }
            validateWorkerConfig(
                yolo_conf_, mask_threshold_, yolo_imgsz_, est_refine_iter_,
                track_refine_iter_, min_depth_m_, max_depth_m_, min_mask_pixels_,
                min_valid_depth_ratio_, mask_close_kernel_);
        }

        static void validateWorkerConfig(
            const double yolo_conf,
            const double mask_threshold,
            const int yolo_imgsz,
            const int est_refine_iter,
            const int track_refine_iter,
            const double min_depth_m,
            const double max_depth_m,
            const int min_mask_pixels,
            const double min_valid_depth_ratio,
            const int mask_close_kernel)
        {
            if (yolo_conf < 0.0 || yolo_conf > 1.0 ||
                mask_threshold < 0.0 || mask_threshold > 1.0 ||
                min_valid_depth_ratio < 0.0 || min_valid_depth_ratio > 1.0)
            {
                throw std::invalid_argument("confidence and ratio parameters must be in [0, 1]");
            }
            if (yolo_imgsz <= 0 || est_refine_iter <= 0 || track_refine_iter <= 0 ||
                min_mask_pixels <= 0 || mask_close_kernel < 0)
            {
                throw std::invalid_argument("integer worker parameters are invalid");
            }
            if (min_depth_m < 0.0 || max_depth_m <= min_depth_m)
            {
                throw std::invalid_argument("depth range is invalid");
            }
        }

        void setupImageSubscriptions()
        {
            // 当前 ZED 发布端为 RELIABLE。若以后换成 BEST_EFFORT 相机，
            // 可将这里改成 SensorDataQoS 对应的 rmw profile。
            const auto image_qos =
                rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();

            camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
                camera_info_topic_,
                image_qos,
                std::bind(
                    &FoundationPoseBridgeNode::cameraInfoCallback,
                    this,
                    std::placeholders::_1));

            rgb_sub_.subscribe(this, rgb_topic_, image_qos.get_rmw_qos_profile());
            depth_sub_.subscribe(this, depth_topic_, image_qos.get_rmw_qos_profile());

            ApproximatePolicy policy(static_cast<std::uint32_t>(sync_queue_size_));
            policy.setMaxIntervalDuration(rclcpp::Duration::from_seconds(sync_slop_sec_));

            synchronizer_ =
                std::make_shared<message_filters::Synchronizer<ApproximatePolicy>>(policy);
            synchronizer_->connectInput(rgb_sub_, depth_sub_);
            synchronizer_->registerCallback(
                std::bind(
                    &FoundationPoseBridgeNode::rgbdCallback,
                    this,
                    std::placeholders::_1,
                    std::placeholders::_2));
        }

        void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr msg)
        {
            std::lock_guard<std::mutex> lock(camera_info_mutex_);
            for (std::size_t index = 0; index < camera_k_.size(); ++index)
            {
                camera_k_[index] = msg->k[index];
            }
            camera_info_width_ = msg->width;
            camera_info_height_ = msg->height;
            has_camera_info_ = true;
        }

        void rgbdCallback(
            const Image::ConstSharedPtr& rgb_msg,
            const Image::ConstSharedPtr& depth_msg)
        {
            ++received_frames_;
            latest_rgb_frame_id_ = rgb_msg->header.frame_id;

            std::array<double, 9> camera_k{};
            std::uint32_t camera_width = 0U;
            std::uint32_t camera_height = 0U;

            {
                std::lock_guard<std::mutex> lock(camera_info_mutex_);
                if (!has_camera_info_)
                {
                    ++dropped_frames_;
                    RCLCPP_WARN_THROTTLE(
                        get_logger(), *get_clock(), 2000,
                        "Waiting for CameraInfo on %s", camera_info_topic_.c_str());
                    return;
                }
                camera_k = camera_k_;
                camera_width = camera_info_width_;
                camera_height = camera_info_height_;
            }

            try
            {
                const auto rgb_cv = cv_bridge::toCvShare(
                    rgb_msg, sensor_msgs::image_encodings::RGB8);
                const cv::Mat& rgb = rgb_cv->image;

                cv::Mat converted_depth;
                const cv::Mat* depth_m = nullptr;
                cv_bridge::CvImageConstPtr depth_cv;

                if (depth_msg->encoding == sensor_msgs::image_encodings::TYPE_32FC1)
                {
                    depth_cv = cv_bridge::toCvShare(
                        depth_msg, sensor_msgs::image_encodings::TYPE_32FC1);
                    depth_m = &depth_cv->image;
                }
                else if (
                    depth_msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1 ||
                    depth_msg->encoding == sensor_msgs::image_encodings::MONO16)
                {
                    depth_cv = cv_bridge::toCvShare(
                        depth_msg, sensor_msgs::image_encodings::TYPE_16UC1);
                    depth_cv->image.convertTo(converted_depth, CV_32FC1, 0.001);
                    depth_m = &converted_depth;
                }
                else
                {
                    ++invalid_frames_;
                    RCLCPP_ERROR_THROTTLE(
                        get_logger(), *get_clock(), 2000,
                        "Unsupported depth encoding: %s", depth_msg->encoding.c_str());
                    return;
                }

                if (rgb.rows != depth_m->rows || rgb.cols != depth_m->cols)
                {
                    ++invalid_frames_;
                    RCLCPP_ERROR_THROTTLE(
                        get_logger(), *get_clock(), 2000,
                        "RGB/depth size mismatch: RGB=%dx%d Depth=%dx%d",
                        rgb.cols, rgb.rows, depth_m->cols, depth_m->rows);
                    return;
                }

                if ((camera_width != 0U && camera_width != static_cast<std::uint32_t>(rgb.cols)) ||
                    (camera_height != 0U && camera_height != static_cast<std::uint32_t>(rgb.rows)))
                {
                    ++invalid_frames_;
                    RCLCPP_ERROR_THROTTLE(
                        get_logger(), *get_clock(), 2000,
                        "CameraInfo size mismatch: K=%ux%u RGB-D=%dx%d",
                        camera_width, camera_height, rgb.cols, rgb.rows);
                    return;
                }

                if (!rgbd_writer_ ||
                    rgbd_writer_->width() != static_cast<std::uint32_t>(rgb.cols) ||
                    rgbd_writer_->height() != static_cast<std::uint32_t>(rgb.rows))
                {
                    rgbd_writer_.reset();
                    rgbd_writer_ = std::make_unique<SharedRgbdWriter>(
                        rgbd_shm_name_,
                        static_cast<std::uint32_t>(rgb.cols),
                        static_cast<std::uint32_t>(rgb.rows),
                        unlink_rgbd_on_exit_);

                    RCLCPP_INFO(
                        get_logger(),
                        "Created RGB-D shared memory %s: %dx%d, %.2f MiB",
                        rgbd_shm_name_.c_str(), rgb.cols, rgb.rows,
                        static_cast<double>(rgbd_writer_->mappedSize()) / 1024.0 / 1024.0);
                }

                const auto rgb_stamp_ns = static_cast<std::uint64_t>(
                    rclcpp::Time(rgb_msg->header.stamp).nanoseconds());
                const auto depth_stamp_ns = static_cast<std::uint64_t>(
                    rclcpp::Time(depth_msg->header.stamp).nanoseconds());

                rgbd_writer_->write(rgb, *depth_m, camera_k, rgb_stamp_ns, depth_stamp_ns);
                ++written_frames_;
            }
            catch (const cv_bridge::Exception& error)
            {
                ++invalid_frames_;
                RCLCPP_ERROR_THROTTLE(
                    get_logger(), *get_clock(), 2000,
                    "cv_bridge conversion failed: %s", error.what());
            }
            catch (const std::exception& error)
            {
                ++invalid_frames_;
                RCLCPP_ERROR_THROTTLE(
                    get_logger(), *get_clock(), 2000,
                    "RGB-D shared-memory write failed: %s", error.what());
            }
        }

        ControlConfig currentControlConfig() const
        {
            ControlConfig config;
            config.enabled = enabled_;
            config.yolo_imgsz = static_cast<std::uint32_t>(yolo_imgsz_);
            config.est_refine_iter = static_cast<std::uint32_t>(est_refine_iter_);
            config.track_refine_iter = static_cast<std::uint32_t>(track_refine_iter_);
            config.min_mask_pixels = static_cast<std::uint32_t>(min_mask_pixels_);
            config.mask_close_kernel = static_cast<std::uint32_t>(mask_close_kernel_);
            config.publish_debug_image = publish_debug_image_;
            config.publish_mask = publish_mask_;
            config.yolo_conf = yolo_conf_;
            config.mask_threshold = mask_threshold_;
            config.min_depth_m = min_depth_m_;
            config.max_depth_m = max_depth_m_;
            config.min_valid_depth_ratio = min_valid_depth_ratio_;
            return config;
        }

        void publishControlSnapshot()
        {
            if (control_writer_)
            {
                control_writer_->updateConfig(currentControlConfig());
            }
        }

        rcl_interfaces::msg::SetParametersResult onSetParameters(
            const std::vector<rclcpp::Parameter>& parameters)
        {
            auto enabled = enabled_;
            auto yolo_conf = yolo_conf_;
            auto mask_threshold = mask_threshold_;
            auto yolo_imgsz = yolo_imgsz_;
            auto est_refine_iter = est_refine_iter_;
            auto track_refine_iter = track_refine_iter_;
            auto min_depth_m = min_depth_m_;
            auto max_depth_m = max_depth_m_;
            auto min_mask_pixels = min_mask_pixels_;
            auto min_valid_depth_ratio = min_valid_depth_ratio_;
            auto mask_close_kernel = mask_close_kernel_;
            auto publish_pose = publish_pose_;
            auto publish_tf = publish_tf_;
            auto publish_debug_image = publish_debug_image_;
            auto publish_mask = publish_mask_;
            auto camera_frame_id = camera_frame_id_;
            auto object_frame_id = object_frame_id_;

            rcl_interfaces::msg::SetParametersResult result;
            result.successful = false;

            try
            {
                for (const auto& parameter : parameters)
                {
                    const auto& name = parameter.get_name();
                    if (name == "enabled")
                    {
                        enabled = parameter.as_bool();
                    }
                    else if (name == "yolo_conf")
                    {
                        yolo_conf = parameter.as_double();
                    }
                    else if (name == "mask_threshold")
                    {
                        mask_threshold = parameter.as_double();
                    }
                    else if (name == "yolo_imgsz")
                    {
                        yolo_imgsz = static_cast<int>(parameter.as_int());
                    }
                    else if (name == "est_refine_iter")
                    {
                        est_refine_iter = static_cast<int>(parameter.as_int());
                    }
                    else if (name == "track_refine_iter")
                    {
                        track_refine_iter = static_cast<int>(parameter.as_int());
                    }
                    else if (name == "min_depth_m")
                    {
                        min_depth_m = parameter.as_double();
                    }
                    else if (name == "max_depth_m")
                    {
                        max_depth_m = parameter.as_double();
                    }
                    else if (name == "min_mask_pixels")
                    {
                        min_mask_pixels = static_cast<int>(parameter.as_int());
                    }
                    else if (name == "min_valid_depth_ratio")
                    {
                        min_valid_depth_ratio = parameter.as_double();
                    }
                    else if (name == "mask_close_kernel")
                    {
                        mask_close_kernel = static_cast<int>(parameter.as_int());
                    }
                    else if (name == "publish_pose")
                    {
                        publish_pose = parameter.as_bool();
                    }
                    else if (name == "publish_tf")
                    {
                        publish_tf = parameter.as_bool();
                    }
                    else if (name == "publish_debug_image")
                    {
                        publish_debug_image = parameter.as_bool();
                    }
                    else if (name == "publish_mask")
                    {
                        publish_mask = parameter.as_bool();
                    }
                    else if (name == "camera_frame_id")
                    {
                        camera_frame_id = parameter.as_string();
                    }
                    else if (name == "object_frame_id")
                    {
                        object_frame_id = parameter.as_string();
                    }
                    else
                    {
                        result.reason = "Parameter is startup-only or unknown: " + name;
                        return result;
                    }
                }

                validateWorkerConfig(
                    yolo_conf, mask_threshold, yolo_imgsz, est_refine_iter,
                    track_refine_iter, min_depth_m, max_depth_m, min_mask_pixels,
                    min_valid_depth_ratio, mask_close_kernel);

                if (object_frame_id.empty())
                {
                    throw std::invalid_argument("object_frame_id cannot be empty");
                }

                const bool enable_transition = !enabled_ && enabled;
                const bool reinitialize_required = enabled && (
                    yolo_conf != yolo_conf_ ||
                    mask_threshold != mask_threshold_ ||
                    yolo_imgsz != yolo_imgsz_ ||
                    est_refine_iter != est_refine_iter_ ||
                    min_mask_pixels != min_mask_pixels_ ||
                    min_valid_depth_ratio != min_valid_depth_ratio_ ||
                    mask_close_kernel != mask_close_kernel_);

                enabled_ = enabled;
                yolo_conf_ = yolo_conf;
                mask_threshold_ = mask_threshold;
                yolo_imgsz_ = yolo_imgsz;
                est_refine_iter_ = est_refine_iter;
                track_refine_iter_ = track_refine_iter;
                min_depth_m_ = min_depth_m;
                max_depth_m_ = max_depth_m;
                min_mask_pixels_ = min_mask_pixels;
                min_valid_depth_ratio_ = min_valid_depth_ratio;
                mask_close_kernel_ = mask_close_kernel;
                publish_pose_ = publish_pose;
                publish_tf_ = publish_tf;
                publish_debug_image_ = publish_debug_image;
                publish_mask_ = publish_mask;
                camera_frame_id_ = camera_frame_id;
                object_frame_id_ = object_frame_id;

                publishControlSnapshot();
                if ((enable_transition || reinitialize_required) && control_writer_)
                {
                    // 恢复估计，或修改了分割/注册参数后，重新运行 YOLO + register。
                    control_writer_->requestReinitialize();
                }

                result.successful = true;
                result.reason = "accepted";
                return result;
            }
            catch (const std::exception& error)
            {
                result.reason = error.what();
                return result;
            }
        }

        void setEnabledService(
            const std_srvs::srv::SetBool::Request::SharedPtr request,
            std_srvs::srv::SetBool::Response::SharedPtr response)
        {
            const auto set_result = set_parameters_atomically(
                {rclcpp::Parameter("enabled", request->data)});
            response->success = set_result.successful;
            response->message = set_result.successful
                                    ? (request->data
                                           ? "FoundationPose estimation enabled"
                                           : "FoundationPose estimation disabled")
                                    : set_result.reason;
        }

        void reinitializeService(
            const std_srvs::srv::Trigger::Request::SharedPtr,
            std_srvs::srv::Trigger::Response::SharedPtr response)
        {
            control_writer_->requestReinitialize();
            response->success = true;
            response->message = "Reinitialization scheduled";
        }

        void shutdownWorkerService(
            const std_srvs::srv::Trigger::Request::SharedPtr,
            std_srvs::srv::Trigger::Response::SharedPtr response)
        {
            control_writer_->requestShutdown();
            response->success = true;
            response->message = "Worker shutdown requested";
        }

        void getStatusService(
            const std_srvs::srv::Trigger::Request::SharedPtr,
            std_srvs::srv::Trigger::Response::SharedPtr response)
        {
            response->success = worker_result_received_;
            response->message = worker_result_received_ ? latest_worker_status_ : "No worker result received";
        }

        std::string effectiveCameraFrameId() const
        {
            if (!camera_frame_id_.empty())
            {
                return camera_frame_id_;
            }
            if (!latest_rgb_frame_id_.empty())
            {
                return latest_rgb_frame_id_;
            }
            return "camera_optical_frame";
        }

        rclcpp::Time resultStamp(const ResultFrame& result) const
        {
            if (result.source_timestamp_ns == 0U)
            {
                return now();
            }
            return rclcpp::Time(
                static_cast<std::int64_t>(result.source_timestamp_ns),
                get_clock()->get_clock_type());
        }

        geometry_msgs::msg::PoseStamped poseMessage(const ResultFrame& result) const
        {
            geometry_msgs::msg::PoseStamped message;
            message.header.stamp = resultStamp(result);
            message.header.frame_id = effectiveCameraFrameId();

            message.pose.position.x = result.pose[3];
            message.pose.position.y = result.pose[7];
            message.pose.position.z = result.pose[11];

            tf2::Matrix3x3 rotation(
                result.pose[0], result.pose[1], result.pose[2],
                result.pose[4], result.pose[5], result.pose[6],
                result.pose[8], result.pose[9], result.pose[10]);
            tf2::Quaternion quaternion;
            rotation.getRotation(quaternion);
            quaternion.normalize();

            message.pose.orientation.x = quaternion.x();
            message.pose.orientation.y = quaternion.y();
            message.pose.orientation.z = quaternion.z();
            message.pose.orientation.w = quaternion.w();
            return message;
        }

        void publishResult(const ResultFrame& result)
        {
            const auto stamp = resultStamp(result);
            const auto camera_frame = effectiveCameraFrameId();

            if (result.pose_valid)
            {
                const auto pose_message = poseMessage(result);
                if (publish_pose_)
                {
                    pose_pub_->publish(pose_message);
                }

                if (publish_tf_)
                {
                    geometry_msgs::msg::TransformStamped transform;
                    transform.header = pose_message.header;
                    transform.child_frame_id = object_frame_id_;
                    transform.transform.translation.x = pose_message.pose.position.x;
                    transform.transform.translation.y = pose_message.pose.position.y;
                    transform.transform.translation.z = pose_message.pose.position.z;
                    transform.transform.rotation = pose_message.pose.orientation;
                    tf_broadcaster_->sendTransform(transform);
                }
            }

            if (publish_debug_image_ && result.debug_valid && !result.debug_bgr.empty())
            {
                std_msgs::msg::Header header;
                header.stamp = stamp;
                header.frame_id = camera_frame;
                debug_image_pub_->publish(
                    *cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, result.debug_bgr)
                    .toImageMsg());
            }

            if (publish_mask_ && result.mask_valid && !result.mask_mono8.empty())
            {
                std_msgs::msg::Header header;
                header.stamp = stamp;
                header.frame_id = camera_frame;
                mask_pub_->publish(
                    *cv_bridge::CvImage(header, sensor_msgs::image_encodings::MONO8, result.mask_mono8)
                    .toImageMsg());
            }

            std_msgs::msg::UInt8 state_message;
            state_message.data = static_cast<std::uint8_t>(result.state);
            state_pub_->publish(state_message);

            std::ostringstream stream;
            stream << result.status
                << " state=" << result.state
                << " pose_valid=" << (result.pose_valid ? "true" : "false")
                << " inference_ms=" << result.inference_ms
                << " fps=" << result.fps;
            latest_worker_status_ = stream.str();

            std_msgs::msg::String status_message;
            status_message.data = latest_worker_status_;
            status_pub_->publish(status_message);
        }

        void pollWorkerResult()
        {
            if (!result_reader_->isOpen() || !result_reader_->mappingIsCurrent())
            {
                result_reader_->close();
                if (!result_reader_->tryOpen())
                {
                    return;
                }
                last_result_sequence_ = 0U;
                RCLCPP_INFO(get_logger(), "Mapped worker result shared memory: %s", result_shm_name_.c_str());
            }

            ResultFrame result;
            if (!result_reader_->readLatest(last_result_sequence_, result))
            {
                return;
            }

            last_result_sequence_ = result.sequence;
            worker_result_received_ = true;
            ++published_results_;
            publishResult(result);
        }

        void logStatus()
        {
            RCLCPP_INFO(
                get_logger(),
                "rgbd received=%llu written=%llu dropped=%llu invalid=%llu | worker_results=%llu | %s",
                static_cast<unsigned long long>(received_frames_),
                static_cast<unsigned long long>(written_frames_),
                static_cast<unsigned long long>(dropped_frames_),
                static_cast<unsigned long long>(invalid_frames_),
                static_cast<unsigned long long>(published_results_),
                worker_result_received_ ? latest_worker_status_.c_str() : "worker not connected");
        }

        std::string rgb_topic_;
        std::string depth_topic_;
        std::string camera_info_topic_;
        std::string rgbd_shm_name_;
        std::string control_shm_name_;
        std::string result_shm_name_;
        int sync_queue_size_{10};
        double sync_slop_sec_{0.01};
        bool unlink_rgbd_on_exit_{false};
        double log_period_sec_{2.0};
        double result_poll_hz_{100.0};

        bool enabled_{true};
        double yolo_conf_{0.5};
        double mask_threshold_{0.5};
        int yolo_imgsz_{640};
        int est_refine_iter_{5};
        int track_refine_iter_{2};
        double min_depth_m_{0.05};
        double max_depth_m_{5.0};
        int min_mask_pixels_{500};
        double min_valid_depth_ratio_{0.05};
        int mask_close_kernel_{0};

        bool publish_pose_{true};
        bool publish_tf_{true};
        bool publish_debug_image_{true};
        bool publish_mask_{true};
        std::string camera_frame_id_;
        std::string object_frame_id_;
        std::string latest_rgb_frame_id_;

        message_filters::Subscriber<Image> rgb_sub_;
        message_filters::Subscriber<Image> depth_sub_;
        std::shared_ptr<message_filters::Synchronizer<ApproximatePolicy>> synchronizer_;
        rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;

        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_image_pub_;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr mask_pub_;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
        rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr state_pub_;
        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

        rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr set_enabled_service_;
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reinitialize_service_;
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr shutdown_service_;
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr get_status_service_;
        rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;

        rclcpp::TimerBase::SharedPtr status_timer_;
        rclcpp::TimerBase::SharedPtr result_timer_;

        std::mutex camera_info_mutex_;
        std::array<double, 9> camera_k_{};
        std::uint32_t camera_info_width_{0U};
        std::uint32_t camera_info_height_{0U};
        bool has_camera_info_{false};

        std::unique_ptr<SharedRgbdWriter> rgbd_writer_;
        std::unique_ptr<ControlShmWriter> control_writer_;
        std::unique_ptr<ResultShmReader> result_reader_;

        std::uint64_t received_frames_{0U};
        std::uint64_t written_frames_{0U};
        std::uint64_t dropped_frames_{0U};
        std::uint64_t invalid_frames_{0U};
        std::uint64_t last_result_sequence_{0U};
        std::uint64_t published_results_{0U};
        bool worker_result_received_{false};
        std::string latest_worker_status_{"worker not connected"};
    };
} // namespace foundationpose_shm_bridge

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    try
    {
        rclcpp::spin(
            std::make_shared<foundationpose_shm_bridge::FoundationPoseBridgeNode>());
    }
    catch (const std::exception& error)
    {
        RCLCPP_FATAL(rclcpp::get_logger("foundationpose_bridge"), "%s", error.what());
        rclcpp::shutdown();
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}
