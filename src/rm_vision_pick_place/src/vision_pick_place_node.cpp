#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
#include <moveit/task_constructor/task.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Matrix3x3.h>
#if __has_include(<tf2_geometry_msgs/tf2_geometry_msgs.hpp>)
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <functional>
#include <future>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace rm_vision_pick_place
{
    namespace mtc = moveit::task_constructor;
    using namespace std::chrono_literals;

    class VisionPickPlaceNode final : public rclcpp::Node
    {
    public:
        explicit VisionPickPlaceNode(const rclcpp::NodeOptions& options)
            : Node("vision_pick_place", options)
        {
            //获取参数
            declareAndReadParameters();
            //检查参数合法
            validateParameters();

            tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
            tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

            const auto pose_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
            const auto state_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();

            pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
                foundation_pose_topic_, pose_qos,
                std::bind(&VisionPickPlaceNode::poseCallback, this, std::placeholders::_1));

            foundation_state_sub_ = create_subscription<std_msgs::msg::UInt8>(
                foundation_state_topic_, state_qos,
                std::bind(&VisionPickPlaceNode::foundationStateCallback, this, std::placeholders::_1));

            foundation_status_sub_ = create_subscription<std_msgs::msg::String>(
                foundation_status_topic_, state_qos,
                std::bind(&VisionPickPlaceNode::foundationStatusCallback, this, std::placeholders::_1));

            foundation_enable_client_ =
                create_client<std_srvs::srv::SetBool>(foundation_enable_service_);
            foundation_reinitialize_client_ =
                create_client<std_srvs::srv::Trigger>(foundation_reinitialize_service_);

            start_service_ = create_service<std_srvs::srv::Trigger>(
                "/vision_pick_place/start",
                std::bind(&VisionPickPlaceNode::startService, this,
                          std::placeholders::_1, std::placeholders::_2));

            status_pub_ = create_publisher<std_msgs::msg::String>(
                "/vision_pick_place/status", state_qos);
            state_pub_ = create_publisher<std_msgs::msg::UInt8>(
                "/vision_pick_place/state", state_qos);

            publishPipelineState(PipelineState::IDLE, "ready");
            RCLCPP_INFO(get_logger(), "Vision pick-place node ready");
        }

        ~VisionPickPlaceNode() override
        {
            if (worker_thread_.joinable())
            {
                worker_thread_.join();
            }
        }

    private:
        template <typename T>
        T declareOrGetParameter(
            const std::string& name,
            const T& default_value)
        {
            /*
             * ros2 launch + automatically_declare_parameters_from_overrides(true)
             * 时，参数可能已经由 launch/YAML 自动声明。
             *
             * ros2 run 时，参数可能不存在，此时再声明默认值。
             */

            if (!has_parameter(name))
            {
                return declare_parameter<T>(
                    name,
                    default_value);
            }

            try
            {
                /*
                 * ParameterValue 对整数、浮点数、字符串、
                 * bool 和数组都支持模板读取。
                 */
                return get_parameter(name).get_value<T>();
            }
            catch (const rclcpp::ParameterTypeException& error)
            {
                throw std::runtime_error(
                    "Parameter '" + name +
                    "' has wrong type: " +
                    error.what());
            }
        }

        enum class PipelineState : std::uint8_t
        {
            IDLE = 0,
            ENABLING_FOUNDATIONPOSE = 1,
            REINITIALIZING = 2,
            WAITING_STABLE_POSE = 3,
            POSE_FROZEN = 4,
            UPDATING_SCENE = 5,
            BUILDING_TASK = 6,
            PLANNING = 7,
            EXECUTING = 8,
            SUCCEEDED = 9,
            FAILED = 10,
        };

        void declareAndReadParameters()
        {
            //moveit规划组
            arm_group_ = declareOrGetParameter<std::string>("arm_group", "rm_robot_arm");
            //moveit规划坐标系
            planning_frame_ = declareOrGetParameter<std::string>("planning_frame", "world");
            //末端坐标系
            end_effector_frame_ =
                declareOrGetParameter<std::string>("end_effector_frame", "end_effect_link");
            //触碰坐标系
            touch_links_ = declareOrGetParameter<std::vector<std::string>>(
                "touch_links", std::vector<std::string>{"link6", "end_effect_link"});

            //foundationpose_bridge输出的话题/服务名字
            foundation_pose_topic_ = declareOrGetParameter<std::string>(
                "foundation_pose_topic", "/foundationpose/pose");
            foundation_state_topic_ = declareOrGetParameter<std::string>(
                "foundation_state_topic", "/foundationpose/state");
            foundation_status_topic_ = declareOrGetParameter<std::string>(
                "foundation_status_topic", "/foundationpose/status");
            foundation_enable_service_ = declareOrGetParameter<std::string>(
                "foundation_enable_service", "/foundationpose/set_enabled");
            foundation_reinitialize_service_ = declareOrGetParameter<std::string>(
                "foundation_reinitialize_service", "/foundationpose/reinitialize");
            foundation_tracking_state_ = declareOrGetParameter<int>("foundation_tracking_state", 4);

            pose_wait_timeout_sec_ = declareOrGetParameter<double>("pose_wait_timeout_sec", 15.0);
            stable_sample_count_ = declareOrGetParameter<int>("stable_sample_count", 10);
            stable_position_threshold_m_ =
                declareOrGetParameter<double>("stable_position_threshold_m", 0.008);
            stable_angle_threshold_deg_ =
                declareOrGetParameter<double>("stable_angle_threshold_deg", 4.0);
            tf_timeout_sec_ = declareOrGetParameter<double>("tf_timeout_sec", 0.30);

            object_id_ = declareOrGetParameter<std::string>("object_id", "target_object");
            object_height_m_ = declareOrGetParameter<double>("object_height_m", 0.150);
            object_radius_m_ = declareOrGetParameter<double>("object_radius_m", 0.048);

            object_to_cylinder_xyz_ = declareOrGetParameter<std::vector<double>>(
                "object_to_cylinder_xyz", std::vector<double>{0.0, 0.0, 0.0});
            object_to_cylinder_rpy_ = declareOrGetParameter<std::vector<double>>(
                "object_to_cylinder_rpy", std::vector<double>{1.57079632679, 0.0, 0.0});

            object_to_tool_xyz_ = declareOrGetParameter<std::vector<double>>(
                "object_to_tool_xyz", std::vector<double>{0.0, 0.0, 0.0});
            object_to_tool_rpy_ = declareOrGetParameter<std::vector<double>>(
                "object_to_tool_rpy", std::vector<double>{0.0, 0.0, 0.0});
            place_xyz_ = declareOrGetParameter<std::vector<double>>(
                "place_xyz", std::vector<double>{0.35, 0.25, 0.25});
            place_rpy_ = declareOrGetParameter<std::vector<double>>(
                "place_rpy", std::vector<double>{0.0, 0.0, 0.0});

            max_velocity_scaling_ = declareOrGetParameter<double>("max_velocity_scaling", 0.20);
            max_acceleration_scaling_ = declareOrGetParameter<double>("max_acceleration_scaling", 0.20);
            cartesian_step_m_ = declareOrGetParameter<double>("cartesian_step_m", 0.005);
            connect_timeout_sec_ = declareOrGetParameter<double>("connect_timeout_sec", 10.0);
            ik_timeout_sec_ = declareOrGetParameter<double>("ik_timeout_sec", 0.20);
            max_ik_solutions_ = declareOrGetParameter<int>("max_ik_solutions", 8);
            min_ik_solution_distance_ =
                declareOrGetParameter<double>("min_ik_solution_distance", 0.10);
            max_task_solutions_ = declareOrGetParameter<int>("max_task_solutions", 1);

            approach_min_m_ = declareOrGetParameter<double>("approach_min_m", 0.05);
            approach_max_m_ = declareOrGetParameter<double>("approach_max_m", 0.10);
            lift_min_m_ = declareOrGetParameter<double>("lift_min_m", 0.10);
            lift_max_m_ = declareOrGetParameter<double>("lift_max_m", 0.15);
            lower_min_m_ = declareOrGetParameter<double>("lower_min_m", 0.05);
            lower_max_m_ = declareOrGetParameter<double>("lower_max_m", 0.10);
            retreat_min_m_ = declareOrGetParameter<double>("retreat_min_m", 0.05);
            retreat_max_m_ = declareOrGetParameter<double>("retreat_max_m", 0.10);

            auto_execute_ = declareOrGetParameter<bool>("auto_execute", true);

            side_grasp_azimuth_samples_ =
                declareOrGetParameter<int>(
                    "side_grasp_azimuth_samples",
                    4);

            side_grasp_roll_samples_ =
                declareOrGetParameter<int>(
                    "side_grasp_roll_samples",
                    1);

            side_grasp_radius_m_ =
                declareOrGetParameter<double>(
                    "side_grasp_radius_m",
                    0.080);

            side_grasp_height_offset_m_ =
                declareOrGetParameter<double>(
                    "side_grasp_height_offset_m",
                    0.0);
        }

        void validateParameters() const
        {
            const auto require3 = [](const std::vector<double>& value, const char* name)
            {
                if (value.size() != 3U)
                {
                    throw std::invalid_argument(std::string(name) + " must have exactly 3 values");
                }
            };
            require3(object_to_cylinder_xyz_, "object_to_cylinder_xyz");
            require3(object_to_cylinder_rpy_, "object_to_cylinder_rpy");
            require3(object_to_tool_xyz_, "object_to_tool_xyz");
            require3(object_to_tool_rpy_, "object_to_tool_rpy");
            require3(place_xyz_, "place_xyz");
            require3(place_rpy_, "place_rpy");

            if (arm_group_.empty() || planning_frame_.empty() ||
                end_effector_frame_.empty() || object_id_.empty())
            {
                throw std::invalid_argument("group/frame/object parameters cannot be empty");
            }
            if (stable_sample_count_ < 3 || pose_wait_timeout_sec_ <= 0.0 ||
                stable_position_threshold_m_ <= 0.0 || stable_angle_threshold_deg_ <= 0.0)
            {
                throw std::invalid_argument("pose stability parameters are invalid");
            }
            if (object_height_m_ <= 0.0 || object_radius_m_ <= 0.0)
            {
                throw std::invalid_argument("object dimensions must be positive");
            }
            if (side_grasp_azimuth_samples_ <= 0 || side_grasp_roll_samples_ <= 0 ||
                side_grasp_radius_m_ <= 0.0 || ik_timeout_sec_ <= 0.0)
            {
                throw std::invalid_argument("side grasp / IK parameters are invalid");
            }
        }

        void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            latest_pose_ = *msg;
            ++pose_sequence_;
            has_pose_ = true;
        }

        void foundationStateCallback(const std_msgs::msg::UInt8::SharedPtr msg)
        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            foundation_state_ = msg->data;
        }

        void foundationStatusCallback(const std_msgs::msg::String::SharedPtr msg)
        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            foundation_status_ = msg->data;
        }

        void startService(const std_srvs::srv::Trigger::Request::SharedPtr,
                          std_srvs::srv::Trigger::Response::SharedPtr response)
        {
            if (busy_.exchange(true))
            {
                response->success = false;
                response->message = "pipeline is already running";
                return;
            }

            if (worker_thread_.joinable())
            {
                worker_thread_.join();
            }
            worker_thread_ = std::thread(&VisionPickPlaceNode::runPipeline, this);

            response->success = true;
            response->message = "pipeline accepted";
        }

        tf2::Transform makeObjectToCylinderTransform() const
        {
            tf2::Quaternion rotation;
            rotation.setRPY(
                object_to_cylinder_rpy_[0],
                object_to_cylinder_rpy_[1],
                object_to_cylinder_rpy_[2]);
            rotation.normalize();

            return tf2::Transform(
                rotation,
                tf2::Vector3(
                    object_to_cylinder_xyz_[0],
                    object_to_cylinder_xyz_[1],
                    object_to_cylinder_xyz_[2]));
        }

        geometry_msgs::msg::PoseStamped makeSideGraspPose(
            const geometry_msgs::msg::Pose& object_pose,
            const double azimuth_rad,
            const double roll_rad) const
        {
            /*
             * 假设：
             *
             * 1. foundationpose_object 的 +Z 是圆柱轴
             * 2. end_effect_link 的 +X 是接近方向
             * 3. +X 应从圆柱外部指向圆柱中心
             */

            const double cosine = std::cos(azimuth_rad);
            const double sine = std::sin(azimuth_rad);

            /*
             * end_effect_link 原点在物体坐标系中的位置。
             *
             * 圆柱侧面方位：
             *   θ = 0       位于物体 +X 侧
             *   θ = π/2     位于物体 +Y 侧
             *   θ = π       位于物体 -X 侧
             */
            const tf2::Vector3 position_in_object(
                side_grasp_radius_m_ * cosine,
                side_grasp_radius_m_ * sine,
                side_grasp_height_offset_m_);

            /*
             * 工具局部 +X 指向圆柱中心。
             *
             * 工具在 outward = [cosθ, sinθ, 0] 方向，
             * 所以 inward = [-cosθ, -sinθ, 0]。
             */
            const tf2::Vector3 tool_x(
                -cosine,
                -sine,
                0.0);

            /*
             * 不加滚转时，工具局部 +Z 与圆柱轴平行。
             */
            const tf2::Vector3 base_tool_z(
                0.0,
                0.0,
                1.0);

            /*
             * 构造右手坐标系：
             *
             * X × Y = Z
             * 因而 Y = Z × X
             */
            const tf2::Vector3 base_tool_y =
                base_tool_z.cross(tool_x).normalized();

            /*
             * 绕工具自身 +X 轴增加 roll。
             *
             * 这不会改变接近方向，只改变末端绕接近轴的姿态。
             */
            const double roll_cosine = std::cos(roll_rad);
            const double roll_sine = std::sin(roll_rad);

            const tf2::Vector3 tool_y =
                base_tool_y * roll_cosine +
                base_tool_z * roll_sine;

            const tf2::Vector3 tool_z =
                -base_tool_y * roll_sine +
                base_tool_z * roll_cosine;

            /*
             * 旋转矩阵的三列分别是：
             *
             * end_effect_link 的 X/Y/Z 轴
             * 在 object frame 下的表示。
             *
             * tf2::Matrix3x3 构造参数按行排列。
             */
            const tf2::Matrix3x3 rotation_matrix(
                tool_x.x(), tool_y.x(), tool_z.x(),
                tool_x.y(), tool_y.y(), tool_z.y(),
                tool_x.z(), tool_y.z(), tool_z.z());

            tf2::Quaternion object_to_tool_rotation;
            rotation_matrix.getRotation(
                object_to_tool_rotation);

            object_to_tool_rotation.normalize();

            // This pose is expressed in the canonical cylinder frame C.
            const tf2::Transform cylinder_to_tool(
                object_to_tool_rotation,
                position_in_object);

            /*
             * planning_T_tool =
             * planning_T_object × object_T_cylinder × cylinder_T_tool
             */
            tf2::Transform planning_to_object;
            tf2::fromMsg(object_pose, planning_to_object);

            const tf2::Transform planning_to_tool =
                planning_to_object *
                makeObjectToCylinderTransform() *
                cylinder_to_tool;

            geometry_msgs::msg::PoseStamped result;

            result.header.frame_id = planning_frame_;
            result.header.stamp = now();
            tf2::toMsg(planning_to_tool, result.pose);

            return result;
        }

        void runPipeline()
        {
            //抓取任务构造流水线
            try
            {
                publishPipelineState(PipelineState::ENABLING_FOUNDATIONPOSE,
                                     "enabling FoundationPose");
                //初始化启动foundationpose
                if (!callFoundationSetEnabled(true))
                {
                    finishFailure("failed to enable FoundationPose");
                    return;
                }

                std::uint64_t minimum_sequence = 0;
                {
                    std::lock_guard<std::mutex> lock(data_mutex_);
                    minimum_sequence = pose_sequence_;
                }

                publishPipelineState(PipelineState::REINITIALIZING,
                                     "requesting FoundationPose reinitialization");
                //重定位foundationpose
                if (!callFoundationReinitialize())
                {
                    finishFailure("FoundationPose reinitialize service failed");
                    return;
                }

                publishPipelineState(PipelineState::WAITING_STABLE_POSE,
                                     "waiting for stable object pose");
                //求平均物体位姿，并固定下frozen_pose
                geometry_msgs::msg::PoseStamped frozen_pose;
                if (!waitForStablePose(minimum_sequence, frozen_pose))
                {
                    finishFailure("stable FoundationPose pose timeout");
                    return;
                }

                frozen_object_pose_ = frozen_pose;
                publishPipelineState(PipelineState::POSE_FROZEN, poseText(frozen_pose));

                // 后续只使用 frozen_pose，避免相机随机械臂运动后继续改变目标。
                if (!callFoundationSetEnabled(false))
                {
                    RCLCPP_WARN(get_logger(), "Failed to pause FoundationPose after freezing pose");
                }

                publishPipelineState(PipelineState::UPDATING_SCENE,
                                     "adding target CollisionObject");
                //将抓取物体加入环境
                if (!applyTargetCollisionObject(frozen_pose.pose))
                {
                    finishFailure("failed to add CollisionObject");
                    return;
                }

                //构造任务
                publishPipelineState(PipelineState::BUILDING_TASK, "building MTC task");
                // task_ = createTask(frozen_pose.pose, makePlaceObjectPose());
                task_ = createApproachAttachTask(frozen_pose.pose);
                try
                {
                    task_.init();
                }
                catch (const mtc::InitStageException& error)
                {
                    std::ostringstream stream;
                    stream << "MTC init failed:\n" << error;
                    finishFailure(stream.str());
                    return;
                }

                //规划，执行
                publishPipelineState(PipelineState::PLANNING, "planning MTC task");
                const auto planned = task_.plan(static_cast<std::size_t>(max_task_solutions_));

                std::ostringstream task_state;
                task_.printState(task_state);
                RCLCPP_INFO_STREAM(get_logger(), "MTC state:\n" << task_state.str());

                if (!planned || task_.solutions().empty())
                {
                    std::ostringstream failure;
                    task_.explainFailure(failure);
                    finishFailure("MTC planning failed:\n" + failure.str());
                    return;
                }

                task_.introspection().publishSolution(*task_.solutions().front());

                if (!auto_execute_)
                {
                    finishSuccess("planning succeeded; auto_execute=false");
                    return;
                }

                publishPipelineState(PipelineState::EXECUTING, "executing best solution");
                const auto result = task_.execute(*task_.solutions().front());
                if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
                {
                    finishFailure("execution failed, code=" + std::to_string(result.val));
                    return;
                }

                finishSuccess("pick-place task completed");
            }
            catch (const std::exception& error)
            {
                finishFailure(std::string("pipeline exception: ") + error.what());
            }
        }

        bool callFoundationSetEnabled(bool enabled)
        {
            if (!foundation_enable_client_->wait_for_service(3s))
            {
                RCLCPP_ERROR(get_logger(), "Service unavailable: %s",
                             foundation_enable_service_.c_str());
                return false;
            }
            auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
            request->data = enabled;
            auto future = foundation_enable_client_->async_send_request(request);
            if (future.wait_for(5s) != std::future_status::ready)
            {
                return false;
            }
            return future.get()->success;
        }

        bool callFoundationReinitialize()
        {
            if (!foundation_reinitialize_client_->wait_for_service(3s))
            {
                RCLCPP_ERROR(get_logger(), "Service unavailable: %s",
                             foundation_reinitialize_service_.c_str());
                return false;
            }
            auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
            auto future = foundation_reinitialize_client_->async_send_request(request);
            if (future.wait_for(5s) != std::future_status::ready)
            {
                return false;
            }
            return future.get()->success;
        }

        bool waitForStablePose(std::uint64_t minimum_sequence,
                               geometry_msgs::msg::PoseStamped& frozen_pose)
        {
            std::deque<geometry_msgs::msg::PoseStamped> samples;
            std::uint64_t last_sequence = minimum_sequence;
            const auto deadline = std::chrono::steady_clock::now() +
                std::chrono::duration<double>(pose_wait_timeout_sec_);

            while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline)
            {
                geometry_msgs::msg::PoseStamped pose;
                std::uint64_t sequence = 0;
                std::uint8_t state = 0;
                bool has_pose = false;

                {
                    std::lock_guard<std::mutex> lock(data_mutex_);
                    pose = latest_pose_;
                    sequence = pose_sequence_;
                    state = foundation_state_;
                    has_pose = has_pose_;
                }

                if (!has_pose || sequence <= last_sequence || sequence <= minimum_sequence ||
                    state != static_cast<std::uint8_t>(foundation_tracking_state_))
                {
                    std::this_thread::sleep_for(20ms);
                    continue;
                }
                last_sequence = sequence;

                geometry_msgs::msg::PoseStamped transformed;
                if (!transformPoseToPlanningFrame(pose, transformed))
                {
                    std::this_thread::sleep_for(20ms);
                    continue;
                }

                samples.push_back(transformed);
                while (samples.size() > static_cast<std::size_t>(stable_sample_count_))
                {
                    samples.pop_front();
                }

                if (samples.size() == static_cast<std::size_t>(stable_sample_count_))
                {
                    double max_pos_error = 0.0;
                    double max_angle_error = 0.0;
                    if (calculateStableAverage(samples, frozen_pose,
                                               max_pos_error, max_angle_error))
                    {
                        RCLCPP_INFO(get_logger(),
                                    "Stable pose accepted: position %.4f m, angle %.2f deg",
                                    max_pos_error, max_angle_error);
                        return true;
                    }
                    RCLCPP_INFO_THROTTLE(
                        get_logger(), *get_clock(), 1000,
                        "Pose not stable: position %.4f m, angle %.2f deg",
                        max_pos_error, max_angle_error);
                }

                std::this_thread::sleep_for(10ms);
            }
            return false;
        }

        bool transformPoseToPlanningFrame(
            const geometry_msgs::msg::PoseStamped& source,
            geometry_msgs::msg::PoseStamped& target)
        {
            if (source.header.frame_id.empty())
            {
                return false;
            }
            if (source.header.frame_id == planning_frame_)
            {
                target = source;
                return true;
            }

            try
            {
                const rclcpp::Time stamp(source.header.stamp);
                geometry_msgs::msg::TransformStamped transform;

                if (stamp.nanoseconds() == 0)
                {
                    transform = tf_buffer_->lookupTransform(
                        planning_frame_, source.header.frame_id,
                        tf2::TimePointZero, tf2::durationFromSec(tf_timeout_sec_));
                }
                else
                {
                    transform = tf_buffer_->lookupTransform(
                        planning_frame_, source.header.frame_id,
                        stamp, rclcpp::Duration::from_seconds(tf_timeout_sec_));
                }

                tf2::doTransform(source, target, transform);
                target.header.frame_id = planning_frame_;
                return true;
            }
            catch (const tf2::TransformException& error)
            {
                RCLCPP_WARN_THROTTLE(
                    get_logger(), *get_clock(), 1000,
                    "TF %s -> %s failed: %s",
                    source.header.frame_id.c_str(), planning_frame_.c_str(), error.what());
                return false;
            }
        }

        bool calculateStableAverage(
            const std::deque<geometry_msgs::msg::PoseStamped>& samples,
            geometry_msgs::msg::PoseStamped& average,
            double& max_position_error,
            double& max_angle_error_deg) const
        {
            if (samples.empty())
            {
                return false;
            }

            double sx = 0.0, sy = 0.0, sz = 0.0;
            double qx = 0.0, qy = 0.0, qz = 0.0, qw = 0.0;

            tf2::Quaternion reference(
                samples.front().pose.orientation.x,
                samples.front().pose.orientation.y,
                samples.front().pose.orientation.z,
                samples.front().pose.orientation.w);
            if (reference.length2() < 1e-12)
            {
                return false;
            }
            reference.normalize();

            for (const auto& sample : samples)
            {
                sx += sample.pose.position.x;
                sy += sample.pose.position.y;
                sz += sample.pose.position.z;

                tf2::Quaternion q(
                    sample.pose.orientation.x,
                    sample.pose.orientation.y,
                    sample.pose.orientation.z,
                    sample.pose.orientation.w);
                if (q.length2() < 1e-12)
                {
                    return false;
                }
                q.normalize();

                const double dot = reference.x() * q.x() + reference.y() * q.y() +
                    reference.z() * q.z() + reference.w() * q.w();
                if (dot < 0.0)
                {
                    q = tf2::Quaternion(-q.x(), -q.y(), -q.z(), -q.w());
                }
                qx += q.x();
                qy += q.y();
                qz += q.z();
                qw += q.w();
            }

            const double n = static_cast<double>(samples.size());
            tf2::Quaternion average_q(qx / n, qy / n, qz / n, qw / n);
            if (average_q.length2() < 1e-12)
            {
                return false;
            }
            average_q.normalize();

            average.header.frame_id = planning_frame_;
            average.header.stamp = samples.back().header.stamp;
            average.pose.position.x = sx / n;
            average.pose.position.y = sy / n;
            average.pose.position.z = sz / n;
            average.pose.orientation.x = average_q.x();
            average.pose.orientation.y = average_q.y();
            average.pose.orientation.z = average_q.z();
            average.pose.orientation.w = average_q.w();

            max_position_error = 0.0;
            max_angle_error_deg = 0.0;
            for (const auto& sample : samples)
            {
                const double dx = sample.pose.position.x - average.pose.position.x;
                const double dy = sample.pose.position.y - average.pose.position.y;
                const double dz = sample.pose.position.z - average.pose.position.z;
                max_position_error = std::max(
                    max_position_error, std::sqrt(dx * dx + dy * dy + dz * dz));

                tf2::Quaternion q(
                    sample.pose.orientation.x,
                    sample.pose.orientation.y,
                    sample.pose.orientation.z,
                    sample.pose.orientation.w);
                q.normalize();
                const double dot = std::clamp(
                    std::abs(average_q.x() * q.x() + average_q.y() * q.y() +
                        average_q.z() * q.z() + average_q.w() * q.w()),
                    0.0, 1.0);
                max_angle_error_deg = std::max(
                    max_angle_error_deg, 2.0 * std::acos(dot) * 57.29577951308232);
            }

            return max_position_error <= stable_position_threshold_m_ &&
                max_angle_error_deg <= stable_angle_threshold_deg_;
        }

        bool applyTargetCollisionObject(const geometry_msgs::msg::Pose& object_pose)
        {
            moveit_msgs::msg::CollisionObject object;
            object.id = object_id_;
            object.header.frame_id = planning_frame_;
            object.primitives.resize(1);
            object.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
            object.primitives[0].dimensions = {object_height_m_, object_radius_m_};

            tf2::Transform planning_to_object;
            tf2::fromMsg(object_pose, planning_to_object);
            const tf2::Transform planning_to_cylinder =
                planning_to_object * makeObjectToCylinderTransform();

            tf2::toMsg(planning_to_cylinder, object.pose);
            object.operation = moveit_msgs::msg::CollisionObject::ADD;

            const bool success = planning_scene_interface_.applyCollisionObject(object);
            if (success)
            {
                RCLCPP_INFO(get_logger(),
                            "Added '%s' at [%.3f %.3f %.3f] in %s",
                            object_id_.c_str(), object_pose.position.x,
                            object_pose.position.y, object_pose.position.z,
                            planning_frame_.c_str());
            }
            return success;
        }

        geometry_msgs::msg::PoseStamped makeGraspToolPose(
            const geometry_msgs::msg::Pose& object_pose) const
        {
            tf2::Transform planning_T_object;
            tf2::fromMsg(object_pose, planning_T_object);

            tf2::Quaternion q;
            q.setRPY(object_to_tool_rpy_[0], object_to_tool_rpy_[1],
                     object_to_tool_rpy_[2]);
            q.normalize();

            const tf2::Transform object_T_tool(
                q, tf2::Vector3(object_to_tool_xyz_[0], object_to_tool_xyz_[1],
                                object_to_tool_xyz_[2]));

            geometry_msgs::msg::PoseStamped result;
            result.header.frame_id = planning_frame_;
            result.header.stamp = now();
            const tf2::Transform planning_T_tool = planning_T_object * object_T_tool;
            const tf2::Vector3 translation = planning_T_tool.getOrigin();
            tf2::Quaternion rotation = planning_T_tool.getRotation();
            rotation.normalize();

            result.pose.position.x = translation.x();
            result.pose.position.y = translation.y();
            result.pose.position.z = translation.z();
            result.pose.orientation.x = rotation.x();
            result.pose.orientation.y = rotation.y();
            result.pose.orientation.z = rotation.z();
            result.pose.orientation.w = rotation.w();
            return result;
        }

        geometry_msgs::msg::Pose makePlaceObjectPose() const
        {
            geometry_msgs::msg::Pose pose;
            pose.position.x = place_xyz_[0];
            pose.position.y = place_xyz_[1];
            pose.position.z = place_xyz_[2];

            tf2::Quaternion q;
            q.setRPY(place_rpy_[0], place_rpy_[1], place_rpy_[2]);
            q.normalize();
            pose.orientation.x = q.x();
            pose.orientation.y = q.y();
            pose.orientation.z = q.z();
            pose.orientation.w = q.w();
            return pose;
        }

        mtc::Task createApproachAttachTask(
            const geometry_msgs::msg::Pose& object_pose)
        {
            constexpr double kPi =
                3.14159265358979323846;

            mtc::Task task;

            task.stages()->setName("approach cylinder and attach");

            task.loadRobotModel(shared_from_this());

            const auto robot_model =
                task.getRobotModel();

            const auto* joint_model_group = robot_model->getJointModelGroup(arm_group_);

            if (joint_model_group == nullptr)
            {
                throw std::runtime_error(
                    "JointModelGroup not found: " +
                    arm_group_);
            }

            const auto solver =
                joint_model_group->getSolverInstance();

            RCLCPP_INFO(
                get_logger(),
                "IK diagnosis: model_frame=%s, group=%s, "
                "is_chain=%d, variables=%u, solver=%s, "
                "ik_frame_exists=%d, can_set_ik_for_frame=%d, "
                "default_timeout=%.6f",
                robot_model->getModelFrame().c_str(),
                arm_group_.c_str(),
                static_cast<int>(joint_model_group->isChain()),
                joint_model_group->getVariableCount(),
                solver ? "loaded" : "NOT LOADED",
                static_cast<int>(
                    robot_model->hasLinkModel(
                        end_effector_frame_)),
                static_cast<int>(
                    joint_model_group->canSetStateFromIK(
                        end_effector_frame_)),
                joint_model_group->getDefaultIKTimeout());

            if (!solver)
            {
                throw std::runtime_error(
                    "No IK solver loaded for group '" + arm_group_ +
                    "'. Start this node with vision_pick_place.launch.py so "
                    "robot_description_kinematics is loaded.");
            }

            if (!joint_model_group->canSetStateFromIK(end_effector_frame_))
            {
                throw std::runtime_error(
                    "IK cannot be solved for frame '" + end_effector_frame_ + "'.");
            }

            if (solver)
            {
                RCLCPP_INFO(
                    get_logger(),
                    "IK solver: base=%s, group=%s",
                    solver->getBaseFrame().c_str(),
                    solver->getGroupName().c_str());

                for (const auto& tip :
                     solver->getTipFrames())
                {
                    RCLCPP_INFO(
                        get_logger(),
                        "IK solver tip: %s",
                        tip.c_str());
                }
            }

            /*
             * rm_robot_arm 是真正进行 IK 和运动规划的组。
             *
             * frame_end_effect 是 SRDF EndEffector 名称；
             * end_effect_link 才是本任务使用的 IK Frame。
             */
            task.setProperty(
                "group",
                arm_group_);

            task.setProperty(
                "ik_frame",
                end_effector_frame_);

            mtc::Stage* current_state_ptr = nullptr;
            mtc::Stage* collision_allowed_state_ptr = nullptr;

            //保存当前机器人状态的stage
            {
                auto current_state =
                    std::make_unique<
                        mtc::stages::CurrentState>(
                        "current state");

                current_state_ptr =
                    current_state.get();

                task.add(
                    std::move(current_state));
            }

            {
                auto stage =
                    std::make_unique<
                        mtc::stages::ModifyPlanningScene>(
                        "allow target contact before IK");

                stage->allowCollisions(
                    object_id_,
                    touch_links_,
                    true);

                collision_allowed_state_ptr =
                    stage.get();

                task.add(std::move(stage));
            }

            // 当前状态到预接近状态使用 MoveIt planning pipeline。
            auto sampling_planner =
                std::make_shared<
                    mtc::solvers::PipelinePlanner>(
                    shared_from_this());

            // 预接近状态到接触状态：使用笛卡尔直线规划。
            auto cartesian_planner =
                std::make_shared<
                    mtc::solvers::CartesianPath>();

            cartesian_planner->
                setMaxVelocityScalingFactor(
                    max_velocity_scaling_);

            cartesian_planner->
                setMaxAccelerationScalingFactor(
                    max_acceleration_scaling_);

            cartesian_planner->setStepSize(
                cartesian_step_m_);

            //当前状态链接到接近状态
            {
                auto connect = std::make_unique<mtc::stages::Connect>("move to pre-approach",
                                                                      mtc::stages::Connect::GroupPlannerVector{
                                                                          {arm_group_, sampling_planner}
                                                                      });
                connect->setTimeout(
                    connect_timeout_sec_);
                connect->properties().
                         configureInitFrom(
                             mtc::Stage::PARENT);
                task.add(
                    std::move(connect));
            }

            //接近并且附着
            {
                auto approach_and_attach =
                    std::make_unique<
                        mtc::SerialContainer>(
                        "approach and attach");

                task.properties().exposeTo(
                    approach_and_attach->properties(),
                    {
                        "group",
                        "ik_frame"
                    });

                approach_and_attach->properties().
                                     configureInitFrom(
                                         mtc::Stage::PARENT,
                                         {
                                             "group",
                                             "ik_frame"
                                         });

                //从预接近状态直线移动到接触状态
                {
                    auto approach = std::make_unique<mtc::stages::MoveRelative>("radial approach", cartesian_planner);

                    approach->properties().set(
                        "marker_ns",
                        "radial_approach");

                    approach->properties().set(
                        "link",
                        end_effector_frame_);

                    approach->properties().
                              configureInitFrom(
                                  mtc::Stage::PARENT,
                                  {"group"});

                    approach->setMinMaxDistance(approach_min_m_, approach_max_m_);

                    geometry_msgs::msg::Vector3Stamped direction;

                    // 所有候选姿态都让 end_effect_link 的 +X
                    // 指向圆柱中心。
                    //
                    // 因此不论从圆柱哪一侧抓，
                    // 沿末端局部 +X 都是向圆柱中心接近。
                    direction.header.frame_id = end_effector_frame_;
                    direction.vector.x = 1.0;
                    direction.vector.y = 0.0;
                    direction.vector.z = 0.0;

                    approach->setDirection(
                        direction);

                    approach_and_attach->insert(
                        std::move(approach));
                }

                //生成多个侧面姿态候选
                {
                    auto alternatives =
                        std::make_unique<
                            mtc::Alternatives>(
                            "side approach candidates");

                    approach_and_attach->
                        properties().exposeTo(
                            alternatives->properties(),
                            {
                                "group",
                                "ik_frame"
                            });

                    alternatives->properties().
                                  configureInitFrom(
                                      mtc::Stage::PARENT,
                                      {
                                          "group",
                                          "ik_frame"
                                      });

                    for (int azimuth_index = 0; azimuth_index < side_grasp_azimuth_samples_; ++azimuth_index)
                    {
                        const double azimuth = 2.0 * kPi * static_cast<double>(azimuth_index) / static_cast<double>(
                            side_grasp_azimuth_samples_);

                        for (int roll_index = 0; roll_index < side_grasp_roll_samples_; ++roll_index)
                        {
                            const double roll = 2.0 * kPi * static_cast<double>(roll_index) / static_cast<double>(
                                side_grasp_roll_samples_);
                            const double azimuth_deg = azimuth * 180.0 / kPi;

                            const double roll_deg = roll * 180.0 / kPi;

                            std::ostringstream name;

                            name << "side " << static_cast<int>(std::round(azimuth_deg)) << " deg, roll " << static_cast
                                <int>(std::round(roll_deg)) << " deg";


                            // GeneratePose 只生成一个
                            // end_effect_link 目标位姿。

                            auto pose_generator =
                                std::make_unique<
                                    mtc::stages::GeneratePose>(
                                    name.str());

                            pose_generator->properties().
                                            configureInitFrom(
                                                mtc::Stage::PARENT);

                            pose_generator->properties().set(
                                "marker_ns",
                                "side_grasp_candidates");

                            const auto candidate_pose = makeSideGraspPose(
                                object_pose, azimuth, roll);

                            if (azimuth_index == 0 && roll_index == 0)
                            {
                                RCLCPP_INFO(
                                    get_logger(),
                                    "First side candidate: frame=%s xyz=[%.4f %.4f %.4f] "
                                    "q=[%.4f %.4f %.4f %.4f]",
                                    candidate_pose.header.frame_id.c_str(),
                                    candidate_pose.pose.position.x,
                                    candidate_pose.pose.position.y,
                                    candidate_pose.pose.position.z,
                                    candidate_pose.pose.orientation.x,
                                    candidate_pose.pose.orientation.y,
                                    candidate_pose.pose.orientation.z,
                                    candidate_pose.pose.orientation.w);
                            }

                            pose_generator->setPose(candidate_pose);
                            pose_generator->setMonitoredStage(collision_allowed_state_ptr);

                            // 将目标笛卡尔 Pose
                            // 转换为机械臂关节状态。
                            auto compute_ik =
                                std::make_unique<
                                    mtc::stages::ComputeIK>(
                                    name.str() + " IK",
                                    std::move(
                                        pose_generator));

                            compute_ik->
                                setMaxIKSolutions(
                                    static_cast<std::size_t>(
                                        max_ik_solutions_));
                            compute_ik->setTimeout(ik_timeout_sec_);

                            compute_ik->
                                setMinSolutionDistance(
                                    min_ik_solution_distance_);

                            compute_ik->setIKFrame(
                                end_effector_frame_);

                            compute_ik->properties().
                                        configureInitFrom(
                                            mtc::Stage::PARENT,
                                            {"group"});

                            compute_ik->properties().
                                        configureInitFrom(
                                            mtc::Stage::INTERFACE,
                                            {"target_pose"});

                            alternatives->add(
                                std::move(compute_ik));
                        }
                    }

                    approach_and_attach->insert(
                        std::move(alternatives));
                }

                //允许末端和目标物体接触
                {
                    auto allow_collision =
                        std::make_unique<
                            mtc::stages::
                            ModifyPlanningScene>(
                            "allow target contact");

                    allow_collision->allowCollisions(
                        object_id_,
                        touch_links_,
                        true);

                    approach_and_attach->insert(
                        std::move(allow_collision));
                }

                // 虚拟附着
                {
                    auto attach =
                        std::make_unique<
                            mtc::stages::
                            ModifyPlanningScene>(
                            "attach target object");

                    attach->attachObject(
                        object_id_,
                        end_effector_frame_);

                    approach_and_attach->insert(
                        std::move(attach));
                }

                task.add(
                    std::move(
                        approach_and_attach));
            }

            return task;
        }

        mtc::Task createTask(const geometry_msgs::msg::Pose& object_pose,
                             const geometry_msgs::msg::Pose& place_object_pose)
        {
            mtc::Task task;
            task.stages()->setName("FoundationPose pick-place");
            task.loadRobotModel(shared_from_this());
            task.setProperty("group", arm_group_);
            task.setProperty("ik_frame", end_effector_frame_);

            mtc::Stage* current_state_ptr = nullptr;
            mtc::Stage* attach_object_stage = nullptr;

            {
                auto stage = std::make_unique<mtc::stages::CurrentState>("current state");
                current_state_ptr = stage.get();
                task.add(std::move(stage));
            }

            //设置规划器
            auto sampling_planner =
                std::make_shared<mtc::solvers::PipelinePlanner>(shared_from_this());
            auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
            cartesian_planner->setMaxVelocityScalingFactor(max_velocity_scaling_);
            cartesian_planner->setMaxAccelerationScalingFactor(max_acceleration_scaling_);
            cartesian_planner->setStepSize(cartesian_step_m_);

            {
                auto stage = std::make_unique<mtc::stages::Connect>(
                    "move to pre-pick",
                    mtc::stages::Connect::GroupPlannerVector{{arm_group_, sampling_planner}});
                stage->setTimeout(connect_timeout_sec_);
                stage->properties().configureInitFrom(mtc::Stage::PARENT);
                task.add(std::move(stage));
            }

            {
                auto pick = std::make_unique<mtc::SerialContainer>("pick object");
                task.properties().exposeTo(pick->properties(), {"group", "ik_frame"});
                pick->properties().configureInitFrom(
                    mtc::Stage::PARENT, {"group", "ik_frame"});

                {
                    auto stage = std::make_unique<mtc::stages::MoveRelative>(
                        "approach object", cartesian_planner);
                    stage->properties().set("marker_ns", "approach_object");
                    stage->properties().set("link", end_effector_frame_);
                    stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
                    stage->setMinMaxDistance(approach_min_m_, approach_max_m_);

                    geometry_msgs::msg::Vector3Stamped direction;
                    direction.header.frame_id = planning_frame_;
                    direction.vector.x = -1.0;
                    stage->setDirection(direction);
                    pick->insert(std::move(stage));
                }

                {
                    auto generator = std::make_unique<mtc::stages::GeneratePose>(
                        "generate grasp tool pose");
                    generator->properties().configureInitFrom(mtc::Stage::PARENT);
                    generator->properties().set("marker_ns", "grasp_tool_pose");
                    generator->setPose(makeGraspToolPose(object_pose));
                    generator->setMonitoredStage(current_state_ptr);

                    auto wrapper = std::make_unique<mtc::stages::ComputeIK>(
                        "grasp tool pose IK", std::move(generator));
                    wrapper->setMaxIKSolutions(static_cast<std::size_t>(max_ik_solutions_));
                    wrapper->setMinSolutionDistance(min_ik_solution_distance_);
                    wrapper->setIKFrame(end_effector_frame_);
                    wrapper->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
                    wrapper->properties().configureInitFrom(
                        mtc::Stage::INTERFACE, {"target_pose"});
                    pick->insert(std::move(wrapper));
                }

                {
                    auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>(
                        "allow touch links to contact object");
                    stage->allowCollisions(object_id_, touch_links_, true);
                    pick->insert(std::move(stage));
                }

                {
                    auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>(
                        "virtually attach object");
                    stage->attachObject(object_id_, end_effector_frame_);
                    attach_object_stage = stage.get();
                    pick->insert(std::move(stage));
                }

                {
                    auto stage = std::make_unique<mtc::stages::MoveRelative>(
                        "lift virtual object", cartesian_planner);
                    stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
                    stage->setIKFrame(end_effector_frame_);
                    stage->setMinMaxDistance(lift_min_m_, lift_max_m_);

                    geometry_msgs::msg::Vector3Stamped direction;
                    direction.header.frame_id = planning_frame_;
                    direction.vector.z = 1.0;
                    stage->setDirection(direction);
                    pick->insert(std::move(stage));
                }

                task.add(std::move(pick));
            }

            {
                auto stage = std::make_unique<mtc::stages::Connect>(
                    "transport to pre-place",
                    mtc::stages::Connect::GroupPlannerVector{{arm_group_, sampling_planner}});
                stage->setTimeout(connect_timeout_sec_);
                stage->properties().configureInitFrom(mtc::Stage::PARENT);
                task.add(std::move(stage));
            }

            {
                auto place = std::make_unique<mtc::SerialContainer>("place object");
                task.properties().exposeTo(place->properties(), {"group", "ik_frame"});
                place->properties().configureInitFrom(
                    mtc::Stage::PARENT, {"group", "ik_frame"});

                {
                    auto stage = std::make_unique<mtc::stages::MoveRelative>(
                        "lower virtual object", cartesian_planner);
                    stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
                    stage->setIKFrame(end_effector_frame_);
                    stage->setMinMaxDistance(lower_min_m_, lower_max_m_);

                    geometry_msgs::msg::Vector3Stamped direction;
                    direction.header.frame_id = planning_frame_;
                    direction.vector.z = -1.0;
                    stage->setDirection(direction);
                    place->insert(std::move(stage));
                }

                {
                    auto generator = std::make_unique<mtc::stages::GeneratePlacePose>(
                        "generate fixed object place pose");
                    generator->properties().configureInitFrom(mtc::Stage::PARENT);
                    generator->properties().set("marker_ns", "object_place_pose");
                    generator->setObject(object_id_);

                    geometry_msgs::msg::PoseStamped place_pose;
                    place_pose.header.frame_id = planning_frame_;
                    place_pose.header.stamp = now();
                    place_pose.pose = place_object_pose;
                    generator->setPose(place_pose);
                    generator->setMonitoredStage(attach_object_stage);

                    auto wrapper = std::make_unique<mtc::stages::ComputeIK>(
                        "place pose IK", std::move(generator));
                    wrapper->setMaxIKSolutions(static_cast<std::size_t>(max_ik_solutions_));
                    wrapper->setMinSolutionDistance(min_ik_solution_distance_);
                    wrapper->setIKFrame(object_id_);
                    wrapper->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
                    wrapper->properties().configureInitFrom(
                        mtc::Stage::INTERFACE, {"target_pose"});
                    place->insert(std::move(wrapper));
                }

                {
                    auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>(
                        "virtually detach object");
                    stage->detachObject(object_id_, end_effector_frame_);
                    place->insert(std::move(stage));
                }

                {
                    auto stage = std::make_unique<mtc::stages::MoveRelative>(
                        "retreat from placed object", cartesian_planner);
                    stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
                    stage->setIKFrame(end_effector_frame_);
                    stage->setMinMaxDistance(retreat_min_m_, retreat_max_m_);

                    geometry_msgs::msg::Vector3Stamped direction;
                    direction.header.frame_id = planning_frame_;
                    direction.vector.z = 1.0;
                    stage->setDirection(direction);
                    place->insert(std::move(stage));
                }

                {
                    auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>(
                        "restore object collision checking");
                    stage->allowCollisions(object_id_, touch_links_, false);
                    place->insert(std::move(stage));
                }

                task.add(std::move(place));
            }

            return task;
        }

        void publishPipelineState(PipelineState state, const std::string& text)
        {
            std_msgs::msg::UInt8 state_msg;
            state_msg.data = static_cast<std::uint8_t>(state);
            state_pub_->publish(state_msg);

            std_msgs::msg::String status_msg;
            status_msg.data = text;
            status_pub_->publish(status_msg);

            RCLCPP_INFO(get_logger(), "[pipeline state=%u] %s",
                        static_cast<unsigned int>(state_msg.data), text.c_str());
        }

        void finishFailure(const std::string& message)
        {
            publishPipelineState(PipelineState::FAILED, message);
            busy_.store(false);
        }

        void finishSuccess(const std::string& message)
        {
            publishPipelineState(PipelineState::SUCCEEDED, message);
            busy_.store(false);
        }

        static std::string poseText(const geometry_msgs::msg::PoseStamped& pose)
        {
            std::ostringstream stream;
            stream << "object pose frozen frame=" << pose.header.frame_id
                << " xyz=[" << pose.pose.position.x << ", "
                << pose.pose.position.y << ", " << pose.pose.position.z << "]";
            return stream.str();
        }

        std::string arm_group_;
        std::string planning_frame_;
        std::string end_effector_frame_;
        std::vector<std::string> touch_links_;

        std::string foundation_pose_topic_;
        std::string foundation_state_topic_;
        std::string foundation_status_topic_;
        std::string foundation_enable_service_;
        std::string foundation_reinitialize_service_;
        int foundation_tracking_state_{4};

        double pose_wait_timeout_sec_{15.0};
        int stable_sample_count_{10};
        double stable_position_threshold_m_{0.008};
        double stable_angle_threshold_deg_{4.0};
        double tf_timeout_sec_{0.30};

        int side_grasp_azimuth_samples_{4};
        int side_grasp_roll_samples_{1};

        double side_grasp_radius_m_{0.080};
        double side_grasp_height_offset_m_{0.0};

        std::string object_id_;
        double object_height_m_{0.150};
        double object_radius_m_{0.048};
        std::vector<double> object_to_cylinder_xyz_;
        std::vector<double> object_to_cylinder_rpy_;
        std::vector<double> object_to_tool_xyz_;
        std::vector<double> object_to_tool_rpy_;
        std::vector<double> place_xyz_;
        std::vector<double> place_rpy_;

        double max_velocity_scaling_{0.05};
        double max_acceleration_scaling_{0.05};
        double cartesian_step_m_{0.005};
        double connect_timeout_sec_{10.0};
        double ik_timeout_sec_{0.20};
        int max_ik_solutions_{8};
        double min_ik_solution_distance_{0.10};
        int max_task_solutions_{1};

        double approach_min_m_{0.05};
        double approach_max_m_{0.10};
        double lift_min_m_{0.10};
        double lift_max_m_{0.15};
        double lower_min_m_{0.05};
        double lower_max_m_{0.10};
        double retreat_min_m_{0.05};
        double retreat_max_m_{0.10};
        bool auto_execute_{true};

        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
        rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr foundation_state_sub_;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr foundation_status_sub_;
        rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr foundation_enable_client_;
        rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr foundation_reinitialize_client_;
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_service_;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
        rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr state_pub_;

        moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
        mtc::Task task_;

        std::mutex data_mutex_;
        geometry_msgs::msg::PoseStamped latest_pose_;
        std::uint64_t pose_sequence_{0};
        std::uint8_t foundation_state_{0};
        std::string foundation_status_{"not connected"};
        bool has_pose_{false};
        geometry_msgs::msg::PoseStamped frozen_object_pose_;

        std::atomic_bool busy_{false};
        std::thread worker_thread_;
    };
} // namespace rm_vision_pick_place

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions options;
    options.automatically_declare_parameters_from_overrides(true);

    auto node = std::make_shared<rm_vision_pick_place::VisionPickPlaceNode>(options);
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4U);
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
