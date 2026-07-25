#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/msg/transform_stamped.hpp>

#include <opencv2/aruco.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <numeric>
#include <optional>
#include <sstream>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace rm_handeye_calibration
{
    namespace fs = std::filesystem;
    using namespace std::chrono_literals;

    constexpr double kPi = 3.14159265358979323846;

    struct DetectionSnapshot
    {
        bool image_valid{false};
        bool pose_valid{false};
        int64_t stamp_ns{0};
        std::string frame_id;
        Eigen::Isometry3d camera_T_target{Eigen::Isometry3d::Identity()};
        int marker_count{0};
        int charuco_corner_count{0};
        double reprojection_error_px{std::numeric_limits<double>::infinity()};
        uint64_t sequence{0};
    };

    struct HandEyeSample
    {
        Eigen::Isometry3d base_T_hand{Eigen::Isometry3d::Identity()};
        Eigen::Isometry3d camera_T_target{Eigen::Isometry3d::Identity()};
        int64_t stamp_ns{0};
        double reprojection_error_px{0.0};
        int charuco_corner_count{0};
    };

    struct CalibrationSolution
    {
        bool valid{false};
        std::string method_name;
        Eigen::Isometry3d hand_T_camera{Eigen::Isometry3d::Identity()};
        double translation_rms_m{std::numeric_limits<double>::infinity()};
        double rotation_rms_rad{std::numeric_limits<double>::infinity()};
        double translation_max_m{std::numeric_limits<double>::infinity()};
        double rotation_max_rad{std::numeric_limits<double>::infinity()};
        double score{std::numeric_limits<double>::infinity()};
    };

    class HandEyeCalibrationNode final : public rclcpp::Node
    {
    public:
        HandEyeCalibrationNode()
            : Node("handeye_calibration"),
              tf_buffer_(std::make_unique<tf2_ros::Buffer>(get_clock())),
              tf_listener_(std::make_unique<tf2_ros::TransformListener>(*tf_buffer_))
        {
            declareAndReadParameters();
            validateParameters();
            setupBoard();

            const auto image_qos = rclcpp::SensorDataQoS();
            image_sub_ = create_subscription<sensor_msgs::msg::Image>(
                image_topic_, image_qos,
                std::bind(&HandEyeCalibrationNode::imageCallback, this, std::placeholders::_1));

            camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
                camera_info_topic_, image_qos,
                std::bind(&HandEyeCalibrationNode::cameraInfoCallback, this, std::placeholders::_1));

            running_.store(true);
            ui_thread_ = std::thread(&HandEyeCalibrationNode::uiLoop, this);

            RCLCPP_INFO(get_logger(), "Hand-eye calibration GUI started");
            RCLCPP_INFO(get_logger(), "Image topic: %s", image_topic_.c_str());
            RCLCPP_INFO(get_logger(), "CameraInfo topic: %s", camera_info_topic_.c_str());
            RCLCPP_INFO(get_logger(), "Frames: base=%s hand=%s camera_root=%s",
                        base_frame_.c_str(), hand_frame_.c_str(), camera_root_frame_.c_str());
        }

        ~HandEyeCalibrationNode() override
        {
            running_.store(false);
            if (ui_thread_.joinable())
            {
                ui_thread_.join();
            }
            cv::destroyAllWindows();
        }

    private:
        enum class UiCommand : int
        {
            NONE = 0,
            CAPTURE,
            UNDO,
            SOLVE,
            SAVE,
            CLEAR,
            QUIT
        };

        struct Button
        {
            cv::Rect rect;
            std::string label;
            UiCommand command{UiCommand::NONE};
        };

        void declareAndReadParameters()
        {
            image_topic_ = declare_parameter<std::string>(
                "image_topic", "/zed/zed_node/rgb/color/rect/image");
            camera_info_topic_ = declare_parameter<std::string>(
                "camera_info_topic", "/zed/zed_node/rgb/color/rect/camera_info");

            base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
            hand_frame_ = declare_parameter<std::string>("hand_frame", "link3");
            camera_frame_override_ = declare_parameter<std::string>("camera_frame", "");
            camera_root_frame_ = declare_parameter<std::string>(
                "camera_root_frame", "zed_camera_link");

            squares_x_ = declare_parameter<int>("board.squares_x", 7);
            squares_y_ = declare_parameter<int>("board.squares_y", 5);
            square_length_m_ = declare_parameter<double>("board.square_length_m", 0.030);
            marker_length_m_ = declare_parameter<double>("board.marker_length_m", 0.022);
            dictionary_name_ = declare_parameter<std::string>(
                "board.dictionary", "DICT_5X5_100");
            axis_length_m_ = declare_parameter<double>("board.axis_length_m", 0.060);

            min_charuco_corners_ = declare_parameter<int>("quality.min_charuco_corners", 12);
            max_reprojection_error_px_ = declare_parameter<double>(
                "quality.max_reprojection_error_px", 1.5);
            min_translation_delta_m_ = declare_parameter<double>(
                "quality.min_translation_delta_m", 0.030);
            min_rotation_delta_deg_ = declare_parameter<double>(
                "quality.min_rotation_delta_deg", 8.0);
            min_samples_ = declare_parameter<int>("quality.min_samples", 10);
            rotation_weight_m_per_rad_ = declare_parameter<double>(
                "quality.rotation_weight_m_per_rad", 0.05);

            detection_scale_ = declare_parameter<double>("performance.detection_scale", 1.0);
            max_detection_hz_ = declare_parameter<double>("performance.max_detection_hz", 15.0);
            ui_hz_ = declare_parameter<double>("performance.ui_hz", 30.0);
            opencv_threads_ = declare_parameter<int>("performance.opencv_threads", 2);
            max_display_width_ = declare_parameter<int>("performance.max_display_width", 1280);

            tf_timeout_sec_ = declare_parameter<double>("tf_timeout_sec", 0.25);
            output_directory_ = declare_parameter<std::string>(
                "output_directory", "~/handeye_calibration");
            autosave_samples_ = declare_parameter<bool>("autosave_samples", true);
            window_name_ = declare_parameter<std::string>(
                "window_name", "RM Hand-Eye Calibration");
        }

        void validateParameters() const
        {
            if (base_frame_.empty() || hand_frame_.empty())
            {
                throw std::invalid_argument("base_frame and hand_frame cannot be empty");
            }
            if (squares_x_ < 3 || squares_y_ < 3)
            {
                throw std::invalid_argument("ChArUco board must have at least 3x3 squares");
            }
            if (square_length_m_ <= 0.0 || marker_length_m_ <= 0.0 ||
                marker_length_m_ >= square_length_m_)
            {
                throw std::invalid_argument("Invalid square/marker length");
            }
            if (min_charuco_corners_ < 4 || min_samples_ < 3)
            {
                throw std::invalid_argument("Quality thresholds are too small");
            }
            if (detection_scale_ <= 0.1 || detection_scale_ > 1.0)
            {
                throw std::invalid_argument("performance.detection_scale must be in (0.1, 1.0]");
            }
            if (max_detection_hz_ <= 0.0 || ui_hz_ <= 0.0)
            {
                throw std::invalid_argument("UI frequencies must be positive");
            }
        }

        cv::aruco::PREDEFINED_DICTIONARY_NAME dictionaryFromString(const std::string& name) const
        {
            static const std::map<std::string, cv::aruco::PREDEFINED_DICTIONARY_NAME> dictionaries = {
                {"DICT_4X4_50", cv::aruco::DICT_4X4_50},
                {"DICT_4X4_100", cv::aruco::DICT_4X4_100},
                {"DICT_4X4_250", cv::aruco::DICT_4X4_250},
                {"DICT_4X4_1000", cv::aruco::DICT_4X4_1000},
                {"DICT_5X5_50", cv::aruco::DICT_5X5_50},
                {"DICT_5X5_100", cv::aruco::DICT_5X5_100},
                {"DICT_5X5_250", cv::aruco::DICT_5X5_250},
                {"DICT_5X5_1000", cv::aruco::DICT_5X5_1000},
                {"DICT_6X6_50", cv::aruco::DICT_6X6_50},
                {"DICT_6X6_100", cv::aruco::DICT_6X6_100},
                {"DICT_6X6_250", cv::aruco::DICT_6X6_250},
                {"DICT_6X6_1000", cv::aruco::DICT_6X6_1000},
                {"DICT_7X7_50", cv::aruco::DICT_7X7_50},
                {"DICT_7X7_100", cv::aruco::DICT_7X7_100},
                {"DICT_7X7_250", cv::aruco::DICT_7X7_250},
                {"DICT_7X7_1000", cv::aruco::DICT_7X7_1000},
                {"DICT_ARUCO_ORIGINAL", cv::aruco::DICT_ARUCO_ORIGINAL}
            };

            const auto iter = dictionaries.find(name);
            if (iter == dictionaries.end())
            {
                throw std::invalid_argument("Unknown ArUco dictionary: " + name);
            }
            return iter->second;
        }

        void setupBoard()
        {
            cv::setNumThreads(opencv_threads_);
            dictionary_ = cv::aruco::getPredefinedDictionary(dictionaryFromString(dictionary_name_));
            board_ = cv::aruco::CharucoBoard::create(
                squares_x_, squares_y_,
                static_cast<float>(square_length_m_),
                static_cast<float>(marker_length_m_),
                dictionary_);
            detector_parameters_ = cv::aruco::DetectorParameters::create();

            detector_parameters_->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
            detector_parameters_->cornerRefinementWinSize = 5;
            detector_parameters_->cornerRefinementMaxIterations = 30;
            detector_parameters_->cornerRefinementMinAccuracy = 0.05;
        }

        void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg)
        {
            std::lock_guard<std::mutex> lock(image_mutex_);
            latest_image_msg_ = msg;
            ++latest_image_sequence_;
        }

        void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr msg)
        {
            cv::Mat camera_matrix(3, 3, CV_64F);
            for (int row = 0; row < 3; ++row)
            {
                for (int col = 0; col < 3; ++col)
                {
                    camera_matrix.at<double>(row, col) = msg->k[static_cast<std::size_t>(row * 3 + col)];
                }
            }

            cv::Mat distortion(static_cast<int>(msg->d.size()), 1, CV_64F);
            for (std::size_t index = 0; index < msg->d.size(); ++index)
            {
                distortion.at<double>(static_cast<int>(index), 0) = msg->d[index];
            }

            std::lock_guard<std::mutex> lock(camera_info_mutex_);
            camera_matrix_ = camera_matrix;
            distortion_coefficients_ = distortion;
            camera_info_width_ = msg->width;
            camera_info_height_ = msg->height;
            has_camera_info_ = true;
        }

        static Eigen::Isometry3d transformMsgToEigen(const geometry_msgs::msg::Transform& transform)
        {
            Eigen::Quaterniond quaternion(
                transform.rotation.w,
                transform.rotation.x,
                transform.rotation.y,
                transform.rotation.z);
            quaternion.normalize();

            Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
            result.linear() = quaternion.toRotationMatrix();
            result.translation() = Eigen::Vector3d(
                transform.translation.x,
                transform.translation.y,
                transform.translation.z);
            return result;
        }

        static Eigen::Isometry3d rtToEigen(const cv::Mat& rotation, const cv::Mat& translation)
        {
            cv::Mat rotation_64;
            cv::Mat translation_64;
            rotation.convertTo(rotation_64, CV_64F);
            translation.convertTo(translation_64, CV_64F);

            Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
            for (int row = 0; row < 3; ++row)
            {
                for (int col = 0; col < 3; ++col)
                {
                    result.linear()(row, col) = rotation_64.at<double>(row, col);
                }
                if (translation_64.rows == 3)
                {
                    result.translation()(row) = translation_64.at<double>(row, 0);
                }
                else
                {
                    result.translation()(row) = translation_64.at<double>(0, row);
                }
            }
            return result;
        }

        static void eigenToRt(const Eigen::Isometry3d& transform, cv::Mat& rotation, cv::Mat& translation)
        {
            rotation = cv::Mat(3, 3, CV_64F);
            translation = cv::Mat(3, 1, CV_64F);
            for (int row = 0; row < 3; ++row)
            {
                for (int col = 0; col < 3; ++col)
                {
                    rotation.at<double>(row, col) = transform.linear()(row, col);
                }
                translation.at<double>(row, 0) = transform.translation()(row);
            }
        }

        static double rotationAngle(const Eigen::Matrix3d& rotation)
        {
            Eigen::AngleAxisd angle_axis(rotation);
            return std::abs(angle_axis.angle());
        }

        double computeReprojectionError(
            const std::vector<cv::Point2f>& charuco_corners,
            const std::vector<int>& charuco_ids,
            const cv::Vec3d& rvec,
            const cv::Vec3d& tvec,
            const cv::Mat& camera_matrix,
            const cv::Mat& distortion) const
        {
            if (charuco_corners.empty() || charuco_corners.size() != charuco_ids.size())
            {
                return std::numeric_limits<double>::infinity();
            }

            std::vector<cv::Point3f> object_points;
            object_points.reserve(charuco_ids.size());
            for (const int id : charuco_ids)
            {
                if (id < 0 || static_cast<std::size_t>(id) >= board_->chessboardCorners.size())
                {
                    return std::numeric_limits<double>::infinity();
                }
                object_points.push_back(board_->chessboardCorners[static_cast<std::size_t>(id)]);
            }

            std::vector<cv::Point2f> projected_points;
            cv::projectPoints(
                object_points, rvec, tvec,
                camera_matrix, distortion,
                projected_points);

            double squared_sum = 0.0;
            for (std::size_t index = 0; index < charuco_corners.size(); ++index)
            {
                const cv::Point2f difference = charuco_corners[index] - projected_points[index];
                squared_sum += static_cast<double>(difference.dot(difference));
            }
            return std::sqrt(squared_sum / static_cast<double>(charuco_corners.size()));
        }

        std::optional<std::pair<sensor_msgs::msg::Image::ConstSharedPtr, uint64_t>> latestImage() const
        {
            std::lock_guard<std::mutex> lock(image_mutex_);
            if (!latest_image_msg_)
            {
                return std::nullopt;
            }
            return std::make_pair(latest_image_msg_, latest_image_sequence_);
        }

        bool cameraParametersForImage(
            int image_width,
            int image_height,
            cv::Mat& camera_matrix,
            cv::Mat& distortion) const
        {
            std::lock_guard<std::mutex> lock(camera_info_mutex_);
            if (!has_camera_info_)
            {
                return false;
            }

            camera_matrix = camera_matrix_.clone();
            distortion = distortion_coefficients_.clone();

            if (camera_info_width_ > 0U && camera_info_height_ > 0U &&
                (static_cast<int>(camera_info_width_) != image_width ||
                    static_cast<int>(camera_info_height_) != image_height))
            {
                const double scale_x = static_cast<double>(image_width) /
                    static_cast<double>(camera_info_width_);
                const double scale_y = static_cast<double>(image_height) /
                    static_cast<double>(camera_info_height_);
                camera_matrix.at<double>(0, 0) *= scale_x;
                camera_matrix.at<double>(0, 2) *= scale_x;
                camera_matrix.at<double>(1, 1) *= scale_y;
                camera_matrix.at<double>(1, 2) *= scale_y;
            }
            return true;
        }

        void processLatestImage()
        {
            const auto latest = latestImage();
            if (!latest.has_value())
            {
                setStatus("Waiting for image...");
                return;
            }

            const auto& image_msg = latest->first;
            const uint64_t sequence = latest->second;
            if (sequence == last_processed_image_sequence_)
            {
                return;
            }

            cv_bridge::CvImageConstPtr cv_image;
            try
            {
                cv_image = cv_bridge::toCvShare(image_msg, sensor_msgs::image_encodings::BGR8);
            }
            catch (const cv_bridge::Exception& error)
            {
                setStatus(std::string("cv_bridge error: ") + error.what());
                return;
            }

            cv::Mat camera_matrix;
            cv::Mat distortion;
            if (!cameraParametersForImage(
                cv_image->image.cols, cv_image->image.rows,
                camera_matrix, distortion))
            {
                cv::Mat waiting = cv_image->image.clone();
                cv::putText(waiting, "Waiting for CameraInfo", cv::Point(20, 40),
                            cv::FONT_HERSHEY_SIMPLEX, 0.9, cv::Scalar(0, 0, 255), 2, cv::LINE_AA);
                {
                    std::lock_guard<std::mutex> lock(display_mutex_);
                    latest_processed_image_ = waiting;
                }
                setStatus("Waiting for CameraInfo...");
                return;
            }

            cv::Mat working_image;
            cv::Mat working_camera_matrix = camera_matrix.clone();
            if (detection_scale_ < 0.999)
            {
                cv::resize(
                    cv_image->image, working_image, cv::Size(),
                    detection_scale_, detection_scale_, cv::INTER_AREA);
                working_camera_matrix.at<double>(0, 0) *= detection_scale_;
                working_camera_matrix.at<double>(0, 2) *= detection_scale_;
                working_camera_matrix.at<double>(1, 1) *= detection_scale_;
                working_camera_matrix.at<double>(1, 2) *= detection_scale_;
            }
            else
            {
                working_image = cv_image->image.clone();
            }

            cv::Mat grayscale;
            cv::cvtColor(working_image, grayscale, cv::COLOR_BGR2GRAY);

            std::vector<std::vector<cv::Point2f>> marker_corners;
            std::vector<std::vector<cv::Point2f>> rejected_corners;
            std::vector<int> marker_ids;
            cv::aruco::detectMarkers(
                grayscale, dictionary_, marker_corners, marker_ids,
                detector_parameters_, rejected_corners);

            std::vector<cv::Point2f> charuco_corners;
            std::vector<int> charuco_ids;
            if (!marker_ids.empty())
            {
                cv::aruco::interpolateCornersCharuco(
                    marker_corners, marker_ids, grayscale, board_,
                    charuco_corners, charuco_ids,
                    working_camera_matrix, distortion);
            }

            if (!marker_ids.empty())
            {
                // cv::aruco::drawDetectedMarkers(working_image, marker_corners, marker_ids);
                // for (size_t i = 0; i < marker_ids.size(); i++)
                // {
                //     for (size_t j = 0; j <= 5; j++)
                //     {
                //         cv::line(working_image, marker_corners[i][j], marker_corners[i][(j + 1) % 4], cv::Scalar(0, 255, 0), 1);
                //     }
                // }
            }
            if (!charuco_ids.empty())
            {
                cv::aruco::drawDetectedCornersCharuco(
                    working_image, charuco_corners, charuco_ids,
                    cv::Scalar(0, 255, 0));
            }

            DetectionSnapshot snapshot;
            snapshot.image_valid = true;
            snapshot.stamp_ns = rclcpp::Time(image_msg->header.stamp).nanoseconds();
            snapshot.frame_id = image_msg->header.frame_id;
            snapshot.marker_count = static_cast<int>(marker_ids.size());
            snapshot.charuco_corner_count = static_cast<int>(charuco_ids.size());
            snapshot.sequence = sequence;

            if (static_cast<int>(charuco_ids.size()) >= min_charuco_corners_)
            {
                cv::Vec3d rvec;
                cv::Vec3d tvec;
                const bool pose_valid = cv::aruco::estimatePoseCharucoBoard(
                    charuco_corners, charuco_ids, board_,
                    working_camera_matrix, distortion,
                    rvec, tvec);

                if (pose_valid)
                {
                    cv::Mat rotation;
                    cv::Rodrigues(rvec, rotation);
                    cv::Mat translation = (cv::Mat_<double>(3, 1) << tvec[0], tvec[1], tvec[2]);
                    snapshot.camera_T_target = rtToEigen(rotation, translation);
                    snapshot.reprojection_error_px = computeReprojectionError(
                        charuco_corners, charuco_ids, rvec, tvec,
                        working_camera_matrix, distortion);
                    snapshot.pose_valid = std::isfinite(snapshot.reprojection_error_px);

                    cv::drawFrameAxes(
                        working_image, working_camera_matrix, distortion,
                        rvec, tvec, static_cast<float>(axis_length_m_), 2);
                }
            }

            const cv::Scalar quality_color =
            (snapshot.pose_valid &&
                snapshot.reprojection_error_px <= max_reprojection_error_px_)
                ? cv::Scalar(0, 220, 0)
                : cv::Scalar(0, 0, 255);

            std::ostringstream detection_text;
            detection_text << "markers=" << snapshot.marker_count
                << " corners=" << snapshot.charuco_corner_count;
            if (snapshot.pose_valid)
            {
                detection_text << " reproj=" << std::fixed << std::setprecision(2)
                    << snapshot.reprojection_error_px << "px";
            }
            cv::putText(
                working_image, detection_text.str(), cv::Point(15, 30),
                cv::FONT_HERSHEY_SIMPLEX, 0.7, quality_color, 1, cv::LINE_AA);

            {
                std::lock_guard<std::mutex> lock(detection_mutex_);
                latest_detection_ = snapshot;
            }
            {
                std::lock_guard<std::mutex> lock(display_mutex_);
                latest_processed_image_ = working_image;
            }

            last_processed_image_sequence_ = sequence;
        }

        DetectionSnapshot detectionSnapshot() const
        {
            std::lock_guard<std::mutex> lock(detection_mutex_);
            return latest_detection_;
        }

        void setStatus(const std::string& status)
        {
            std::lock_guard<std::mutex> lock(status_mutex_);
            status_text_ = status;
        }

        std::string statusText() const
        {
            std::lock_guard<std::mutex> lock(status_mutex_);
            return status_text_;
        }

        bool isPoseDiverse(const Eigen::Isometry3d& candidate) const
        {
            const double min_rotation_rad = min_rotation_delta_deg_ * kPi / 180.0;
            std::lock_guard<std::mutex> lock(samples_mutex_);
            for (const auto& sample : samples_)
            {
                const Eigen::Isometry3d delta = sample.base_T_hand.inverse() * candidate;
                const double translation_delta = delta.translation().norm();
                const double rotation_delta = rotationAngle(delta.linear());
                if (translation_delta < min_translation_delta_m_ &&
                    rotation_delta < min_rotation_rad)
                {
                    return false;
                }
            }
            return true;
        }

        void captureSample()
        {
            const DetectionSnapshot detection = detectionSnapshot();
            if (!detection.image_valid)
            {
                setStatus("CAPTURE rejected: no image");
                return;
            }
            if (!detection.pose_valid)
            {
                setStatus("CAPTURE rejected: ChArUco pose is invalid");
                return;
            }
            if (detection.charuco_corner_count < min_charuco_corners_)
            {
                setStatus("CAPTURE rejected: not enough ChArUco corners");
                return;
            }
            if (detection.reprojection_error_px > max_reprojection_error_px_)
            {
                std::ostringstream message;
                message << "CAPTURE rejected: reprojection error "
                    << std::fixed << std::setprecision(2)
                    << detection.reprojection_error_px << " px";
                setStatus(message.str());
                return;
            }

            const rclcpp::Time image_time(
                detection.stamp_ns, get_clock()->get_clock_type());

            const std::string calibration_camera_frame = camera_frame_override_.empty()
                                                             ? detection.frame_id
                                                             : camera_frame_override_;
            if (calibration_camera_frame.empty())
            {
                setStatus("CAPTURE rejected: image frame_id is empty");
                return;
            }

            geometry_msgs::msg::TransformStamped base_to_hand;
            Eigen::Isometry3d calibration_camera_T_target = detection.camera_T_target;
            try
            {
                base_to_hand = tf_buffer_->lookupTransform(
                    base_frame_, hand_frame_, image_time,
                    rclcpp::Duration::from_seconds(tf_timeout_sec_));

                if (calibration_camera_frame != detection.frame_id)
                {
                    const auto calibration_camera_to_image = tf_buffer_->lookupTransform(
                        calibration_camera_frame, detection.frame_id, image_time,
                        rclcpp::Duration::from_seconds(tf_timeout_sec_));
                    calibration_camera_T_target =
                        transformMsgToEigen(calibration_camera_to_image.transform) * detection.camera_T_target;
                }
            }
            catch (const std::exception& error)
            {
                setStatus(std::string("CAPTURE rejected: TF error: ") + error.what());
                return;
            }

            {
                std::lock_guard<std::mutex> lock(samples_mutex_);
                if (!sample_camera_frame_.empty() &&
                    sample_camera_frame_ != calibration_camera_frame)
                {
                    setStatus("CAPTURE rejected: camera frame changed from " +
                        sample_camera_frame_ + " to " + calibration_camera_frame);
                    return;
                }
            }

            const Eigen::Isometry3d base_T_hand = transformMsgToEigen(base_to_hand.transform);
            if (!isPoseDiverse(base_T_hand))
            {
                setStatus("CAPTURE rejected: pose too similar to an existing sample");
                return;
            }

            HandEyeSample sample;
            sample.base_T_hand = base_T_hand;
            sample.camera_T_target = calibration_camera_T_target;
            sample.stamp_ns = detection.stamp_ns;
            sample.reprojection_error_px = detection.reprojection_error_px;
            sample.charuco_corner_count = detection.charuco_corner_count;

            std::size_t count = 0;
            {
                std::lock_guard<std::mutex> lock(samples_mutex_);
                samples_.push_back(sample);
                sample_camera_frame_ = calibration_camera_frame;
                count = samples_.size();
                latest_solution_ = CalibrationSolution{};
            }

            std::ostringstream message;
            message << "Captured sample " << count
                << " | corners=" << sample.charuco_corner_count
                << " reproj=" << std::fixed << std::setprecision(2)
                << sample.reprojection_error_px << "px";
            setStatus(message.str());

            if (autosave_samples_)
            {
                saveSamplesOnly("autosave_samples.yaml");
            }
        }

        void undoSample()
        {
            std::size_t count = 0;
            {
                std::lock_guard<std::mutex> lock(samples_mutex_);
                if (samples_.empty())
                {
                    setStatus("UNDO: no sample to remove");
                    return;
                }
                samples_.pop_back();
                if (samples_.empty())
                {
                    sample_camera_frame_.clear();
                }
                latest_solution_ = CalibrationSolution{};
                count = samples_.size();
            }
            setStatus("Removed the last sample. Remaining: " + std::to_string(count));
            if (autosave_samples_)
            {
                saveSamplesOnly("autosave_samples.yaml");
            }
        }

        void clearSamples()
        {
            {
                std::lock_guard<std::mutex> lock(samples_mutex_);
                samples_.clear();
                sample_camera_frame_.clear();
                latest_solution_ = CalibrationSolution{};
            }
            setStatus("All samples cleared");
            if (autosave_samples_)
            {
                saveSamplesOnly("autosave_samples.yaml");
            }
        }

        static Eigen::Quaterniond averageQuaternion(
            const std::vector<Eigen::Quaterniond>& quaternions)
        {
            Eigen::Matrix4d accumulator = Eigen::Matrix4d::Zero();
            if (quaternions.empty())
            {
                return Eigen::Quaterniond::Identity();
            }

            Eigen::Quaterniond reference = quaternions.front().normalized();
            for (Eigen::Quaterniond quaternion : quaternions)
            {
                quaternion.normalize();
                if (reference.dot(quaternion) < 0.0)
                {
                    quaternion.coeffs() *= -1.0;
                }
                const Eigen::Vector4d vector(
                    quaternion.w(), quaternion.x(), quaternion.y(), quaternion.z());
                accumulator += vector * vector.transpose();
            }

            Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d> solver(accumulator);
            const Eigen::Vector4d mean = solver.eigenvectors().col(3);
            Eigen::Quaterniond result(mean(0), mean(1), mean(2), mean(3));
            result.normalize();
            return result;
        }

        CalibrationSolution evaluateSolution(
            const std::string& method_name,
            const Eigen::Isometry3d& hand_T_camera,
            const std::vector<HandEyeSample>& samples) const
        {
            CalibrationSolution solution;
            solution.method_name = method_name;
            solution.hand_T_camera = hand_T_camera;

            std::vector<Eigen::Isometry3d> base_T_target_values;
            base_T_target_values.reserve(samples.size());
            std::vector<Eigen::Quaterniond> target_quaternions;
            target_quaternions.reserve(samples.size());
            Eigen::Vector3d translation_sum = Eigen::Vector3d::Zero();

            for (const auto& sample : samples)
            {
                const Eigen::Isometry3d base_T_target =
                    sample.base_T_hand * hand_T_camera * sample.camera_T_target;
                base_T_target_values.push_back(base_T_target);
                translation_sum += base_T_target.translation();
                target_quaternions.emplace_back(base_T_target.linear());
            }

            const Eigen::Vector3d mean_translation =
                translation_sum / static_cast<double>(samples.size());
            const Eigen::Quaterniond mean_quaternion = averageQuaternion(target_quaternions);

            double translation_squared_sum = 0.0;
            double rotation_squared_sum = 0.0;
            double translation_max = 0.0;
            double rotation_max = 0.0;

            for (const auto& base_T_target : base_T_target_values)
            {
                const double translation_error =
                    (base_T_target.translation() - mean_translation).norm();
                Eigen::Quaterniond quaternion(base_T_target.linear());
                quaternion.normalize();
                Eigen::Quaterniond relative = mean_quaternion.conjugate() * quaternion;
                relative.normalize();
                const double clamped_w = std::clamp(std::abs(relative.w()), 0.0, 1.0);
                const double rotation_error = 2.0 * std::acos(clamped_w);

                translation_squared_sum += translation_error * translation_error;
                rotation_squared_sum += rotation_error * rotation_error;
                translation_max = std::max(translation_max, translation_error);
                rotation_max = std::max(rotation_max, rotation_error);
            }

            solution.translation_rms_m = std::sqrt(
                translation_squared_sum / static_cast<double>(samples.size()));
            solution.rotation_rms_rad = std::sqrt(
                rotation_squared_sum / static_cast<double>(samples.size()));
            solution.translation_max_m = translation_max;
            solution.rotation_max_rad = rotation_max;
            solution.score = solution.translation_rms_m +
                rotation_weight_m_per_rad_ * solution.rotation_rms_rad;
            solution.valid = std::isfinite(solution.score) &&
                hand_T_camera.linear().determinant() > 0.0;
            return solution;
        }

        CalibrationSolution solveMethod(
            const std::string& method_name,
            int method,
            const std::vector<HandEyeSample>& samples) const
        {
            std::vector<cv::Mat> rotations_gripper_to_base;
            std::vector<cv::Mat> translations_gripper_to_base;
            std::vector<cv::Mat> rotations_target_to_camera;
            std::vector<cv::Mat> translations_target_to_camera;

            rotations_gripper_to_base.reserve(samples.size());
            translations_gripper_to_base.reserve(samples.size());
            rotations_target_to_camera.reserve(samples.size());
            translations_target_to_camera.reserve(samples.size());

            for (const auto& sample : samples)
            {
                cv::Mat rotation;
                cv::Mat translation;
                eigenToRt(sample.base_T_hand, rotation, translation);
                rotations_gripper_to_base.push_back(rotation);
                translations_gripper_to_base.push_back(translation);

                eigenToRt(sample.camera_T_target, rotation, translation);
                rotations_target_to_camera.push_back(rotation);
                translations_target_to_camera.push_back(translation);
            }

            cv::Mat rotation_camera_to_gripper;
            cv::Mat translation_camera_to_gripper;
            cv::calibrateHandEye(
                rotations_gripper_to_base,
                translations_gripper_to_base,
                rotations_target_to_camera,
                translations_target_to_camera,
                rotation_camera_to_gripper,
                translation_camera_to_gripper,
                static_cast<cv::HandEyeCalibrationMethod>(method));

            const Eigen::Isometry3d hand_T_camera =
                rtToEigen(rotation_camera_to_gripper, translation_camera_to_gripper);
            return evaluateSolution(method_name, hand_T_camera, samples);
        }

        void solveCalibration()
        {
            std::vector<HandEyeSample> samples;
            {
                std::lock_guard<std::mutex> lock(samples_mutex_);
                samples = samples_;
            }

            if (static_cast<int>(samples.size()) < min_samples_)
            {
                setStatus(
                    "SOLVE rejected: need at least " + std::to_string(min_samples_) +
                    " samples, current=" + std::to_string(samples.size()));
                return;
            }

            const std::array<std::pair<const char*, int>, 5> methods = {
                {
                    {"TSAI", cv::CALIB_HAND_EYE_TSAI},
                    {"PARK", cv::CALIB_HAND_EYE_PARK},
                    {"HORAUD", cv::CALIB_HAND_EYE_HORAUD},
                    {"ANDREFF", cv::CALIB_HAND_EYE_ANDREFF},
                    {"DANIILIDIS", cv::CALIB_HAND_EYE_DANIILIDIS}
                }
            };

            CalibrationSolution best_solution;
            std::ostringstream comparison;
            comparison << "Hand-eye candidates:";

            for (const auto& [name, method] : methods)
            {
                try
                {
                    const CalibrationSolution solution = solveMethod(name, method, samples);
                    if (!solution.valid)
                    {
                        continue;
                    }
                    comparison << " " << name
                        << "=" << std::fixed << std::setprecision(2)
                        << solution.translation_rms_m * 1000.0 << "mm/"
                        << solution.rotation_rms_rad * 180.0 / kPi << "deg";
                    if (!best_solution.valid || solution.score < best_solution.score)
                    {
                        best_solution = solution;
                    }
                }
                catch (const cv::Exception& error)
                {
                    RCLCPP_WARN(get_logger(), "%s hand-eye method failed: %s", name, error.what());
                }
            }

            if (!best_solution.valid)
            {
                setStatus("SOLVE failed: all OpenCV hand-eye methods failed");
                return;
            }

            {
                std::lock_guard<std::mutex> lock(samples_mutex_);
                latest_solution_ = best_solution;
            }

            std::ostringstream message;
            message << "SOLVED " << best_solution.method_name
                << " | RMS=" << std::fixed << std::setprecision(2)
                << best_solution.translation_rms_m * 1000.0 << "mm, "
                << best_solution.rotation_rms_rad * 180.0 / kPi << "deg"
                << " | MAX=" << best_solution.translation_max_m * 1000.0 << "mm, "
                << best_solution.rotation_max_rad * 180.0 / kPi << "deg";
            setStatus(message.str());
            RCLCPP_INFO(get_logger(), "%s", comparison.str().c_str());
            RCLCPP_INFO(get_logger(), "%s", message.str().c_str());
        }

        static std::string expandUserPath(const std::string& path)
        {
            if (path.empty() || path[0] != '~')
            {
                return path;
            }
            const char* home = std::getenv("HOME");
            if (home == nullptr)
            {
                return path;
            }
            if (path.size() == 1)
            {
                return std::string(home);
            }
            if (path[1] == '/')
            {
                return std::string(home) + path.substr(1);
            }
            return path;
        }

        static std::string isoTimestamp()
        {
            const auto now = std::chrono::system_clock::now();
            const std::time_t time = std::chrono::system_clock::to_time_t(now);
            std::tm local_time{};
#if defined(_WIN32)
            localtime_s(&local_time, &time);
#else
            localtime_r(&time, &local_time);
#endif
            std::ostringstream stream;
            stream << std::put_time(&local_time, "%Y%m%d_%H%M%S");
            return stream.str();
        }

        static void writeMatrixYaml(
            std::ostream& stream,
            const std::string& indent,
            const Eigen::Isometry3d& transform)
        {
            stream << indent << "matrix: [";
            const Eigen::Matrix4d matrix = transform.matrix();
            for (int row = 0; row < 4; ++row)
            {
                for (int col = 0; col < 4; ++col)
                {
                    if (row != 0 || col != 0)
                    {
                        stream << ", ";
                    }
                    stream << std::setprecision(12) << matrix(row, col);
                }
            }
            stream << "]\n";
        }

        static Eigen::Vector3d matrixToRpy(const Eigen::Matrix3d& rotation)
        {
            const double pitch = std::asin(std::clamp(-rotation(2, 0), -1.0, 1.0));
            const double cos_pitch = std::cos(pitch);
            double roll = 0.0;
            double yaw = 0.0;
            if (std::abs(cos_pitch) > 1e-9)
            {
                roll = std::atan2(rotation(2, 1), rotation(2, 2));
                yaw = std::atan2(rotation(1, 0), rotation(0, 0));
            }
            else
            {
                roll = std::atan2(-rotation(1, 2), rotation(1, 1));
                yaw = 0.0;
            }
            return Eigen::Vector3d(roll, pitch, yaw);
        }

        static void writeTransformYaml(
            std::ostream& stream,
            const std::string& key,
            const std::string& parent_frame,
            const std::string& child_frame,
            const Eigen::Isometry3d& transform)
        {
            const Eigen::Quaterniond quaternion(transform.linear());
            const Eigen::Vector3d rpy = matrixToRpy(transform.linear());

            stream << key << ":\n";
            stream << "  parent_frame: \"" << parent_frame << "\"\n";
            stream << "  child_frame: \"" << child_frame << "\"\n";
            stream << "  xyz: ["
                << std::setprecision(12) << transform.translation().x() << ", "
                << transform.translation().y() << ", "
                << transform.translation().z() << "]\n";
            stream << "  quaternion_xyzw: ["
                << quaternion.x() << ", " << quaternion.y() << ", "
                << quaternion.z() << ", " << quaternion.w() << "]\n";
            stream << "  rpy_rad: ["
                << rpy.x() << ", " << rpy.y() << ", " << rpy.z() << "]\n";
            writeMatrixYaml(stream, "  ", transform);
        }

        bool ensureOutputDirectory(fs::path& output_path) const
        {
            output_path = fs::path(expandUserPath(output_directory_));
            std::error_code error;
            fs::create_directories(output_path, error);
            if (error)
            {
                RCLCPP_ERROR(get_logger(), "Failed to create output directory: %s",
                             error.message().c_str());
                return false;
            }
            return true;
        }

        bool saveSamplesOnly(const std::string& filename)
        {
            fs::path directory;
            if (!ensureOutputDirectory(directory))
            {
                return false;
            }

            std::vector<HandEyeSample> samples;
            std::string sample_camera_frame;
            {
                std::lock_guard<std::mutex> lock(samples_mutex_);
                samples = samples_;
                sample_camera_frame = sample_camera_frame_;
            }

            std::ofstream stream(directory / filename);
            if (!stream)
            {
                return false;
            }

            stream << "base_frame: \"" << base_frame_ << "\"\n";
            stream << "hand_frame: \"" << hand_frame_ << "\"\n";
            stream << "camera_frame: \"" << sample_camera_frame << "\"\n";
            stream << "sample_count: " << samples.size() << "\n";
            stream << "samples:\n";
            for (std::size_t index = 0; index < samples.size(); ++index)
            {
                stream << "  - index: " << index << "\n";
                stream << "    stamp_ns: " << samples[index].stamp_ns << "\n";
                stream << "    reprojection_error_px: "
                    << samples[index].reprojection_error_px << "\n";
                stream << "    charuco_corner_count: "
                    << samples[index].charuco_corner_count << "\n";
                stream << "    base_T_hand:\n";
                writeMatrixYaml(stream, "      ", samples[index].base_T_hand);
                stream << "    camera_T_target:\n";
                writeMatrixYaml(stream, "      ", samples[index].camera_T_target);
            }
            return true;
        }

        void saveCalibration()
        {
            CalibrationSolution solution;
            std::vector<HandEyeSample> samples;
            {
                std::lock_guard<std::mutex> lock(samples_mutex_);
                solution = latest_solution_;
                samples = samples_;
            }

            if (!solution.valid)
            {
                setStatus("SAVE rejected: solve calibration first");
                return;
            }

            fs::path directory;
            if (!ensureOutputDirectory(directory))
            {
                setStatus("SAVE failed: cannot create output directory");
                return;
            }

            std::string camera_frame;
            {
                std::lock_guard<std::mutex> lock(samples_mutex_);
                camera_frame = sample_camera_frame_;
            }
            if (camera_frame.empty())
            {
                setStatus("SAVE failed: camera optical frame is unknown");
                return;
            }

            const std::string timestamp = isoTimestamp();
            const fs::path result_file = directory / ("handeye_result_" + timestamp + ".yaml");
            const fs::path latest_file = directory / "handeye_result_latest.yaml";
            const fs::path urdf_file = directory / "camera_mount_snippet.urdf";

            std::optional<Eigen::Isometry3d> hand_T_camera_root;
            if (!camera_root_frame_.empty())
            {
                try
                {
                    const auto root_to_camera = tf_buffer_->lookupTransform(
                        camera_root_frame_, camera_frame,
                        rclcpp::Time(0, 0, get_clock()->get_clock_type()),
                        rclcpp::Duration::from_seconds(tf_timeout_sec_));
                    const Eigen::Isometry3d root_T_camera =
                        transformMsgToEigen(root_to_camera.transform);
                    hand_T_camera_root = solution.hand_T_camera * root_T_camera.inverse();
                }
                catch (const std::exception& error)
                {
                    RCLCPP_WARN(get_logger(),
                                "Could not convert optical calibration to camera root frame: %s", error.what());
                }
            }

            auto write_result = [&](const fs::path& path) -> bool
            {
                std::ofstream stream(path);
                if (!stream)
                {
                    return false;
                }
                stream << "method: \"" << solution.method_name << "\"\n";
                stream << "sample_count: " << samples.size() << "\n";
                stream << "translation_rms_m: " << solution.translation_rms_m << "\n";
                stream << "rotation_rms_rad: " << solution.rotation_rms_rad << "\n";
                stream << "translation_max_m: " << solution.translation_max_m << "\n";
                stream << "rotation_max_rad: " << solution.rotation_max_rad << "\n";
                writeTransformYaml(
                    stream, "hand_to_camera_optical",
                    hand_frame_, camera_frame,
                    solution.hand_T_camera);
                if (hand_T_camera_root.has_value())
                {
                    writeTransformYaml(
                        stream, "hand_to_camera_root",
                        hand_frame_, camera_root_frame_,
                        hand_T_camera_root.value());
                }
                return true;
            };

            if (!write_result(result_file) || !write_result(latest_file))
            {
                setStatus("SAVE failed: cannot write result YAML");
                return;
            }

            saveSamplesOnly("handeye_samples_" + timestamp + ".yaml");
            saveSamplesOnly("handeye_samples_latest.yaml");

            if (hand_T_camera_root.has_value())
            {
                const Eigen::Isometry3d& transform = hand_T_camera_root.value();
                const Eigen::Vector3d rpy = matrixToRpy(transform.linear());
                std::ofstream urdf(urdf_file);
                urdf << "<!-- Generated by rm_handeye_calibration -->\n";
                urdf << "<link name=\"" << camera_root_frame_ << "\"/>\n\n";
                urdf << "<joint name=\"" << hand_frame_ << "_to_"
                    << camera_root_frame_ << "\" type=\"fixed\">\n";
                urdf << "  <parent link=\"" << hand_frame_ << "\"/>\n";
                urdf << "  <child link=\"" << camera_root_frame_ << "\"/>\n";
                urdf << "  <origin xyz=\""
                    << std::setprecision(12)
                    << transform.translation().x() << " "
                    << transform.translation().y() << " "
                    << transform.translation().z() << "\" rpy=\""
                    << rpy.x() << " " << rpy.y() << " " << rpy.z()
                    << "\"/>\n";
                urdf << "</joint>\n";
            }

            setStatus("Saved calibration to: " + latest_file.string());
            RCLCPP_INFO(get_logger(), "Calibration saved to %s", result_file.c_str());
        }

        std::size_t sampleCount() const
        {
            std::lock_guard<std::mutex> lock(samples_mutex_);
            return samples_.size();
        }

        CalibrationSolution solutionSnapshot() const
        {
            std::lock_guard<std::mutex> lock(samples_mutex_);
            return latest_solution_;
        }

        cv::Mat buildCanvas()
        {
            cv::Mat image;
            {
                std::lock_guard<std::mutex> lock(display_mutex_);
                if (!latest_processed_image_.empty())
                {
                    image = latest_processed_image_.clone();
                }
            }

            if (image.empty())
            {
                image = cv::Mat(540, 960, CV_8UC3, cv::Scalar(25, 25, 25));
                cv::putText(image, "Waiting for ZED image...", cv::Point(40, 80),
                            cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(230, 230, 230), 2, cv::LINE_AA);
            }

            if (max_display_width_ > 0 && image.cols > max_display_width_)
            {
                const double scale = static_cast<double>(max_display_width_) /
                    static_cast<double>(image.cols);
                cv::resize(image, image, cv::Size(), scale, scale, cv::INTER_AREA);
            }

            constexpr int information_height = 70;
            constexpr int button_height = 58;
            const int panel_height = information_height + button_height;
            cv::Mat canvas(
                image.rows + panel_height,
                image.cols, CV_8UC3,
                cv::Scalar(35, 35, 35));
            image.copyTo(canvas(cv::Rect(0, 0, image.cols, image.rows)));

            const DetectionSnapshot detection = detectionSnapshot();
            const CalibrationSolution solution = solutionSnapshot();
            const std::size_t samples = sampleCount();

            std::ostringstream line_one;
            line_one << "samples=" << samples << "/" << min_samples_
                << " | hand=" << hand_frame_
                << " | camera="
                << (!camera_frame_override_.empty() ? camera_frame_override_ : detection.frame_id);
            cv::putText(canvas, line_one.str(), cv::Point(12, image.rows + 24),
                        cv::FONT_HERSHEY_SIMPLEX, 0.56, cv::Scalar(235, 235, 235), 1, cv::LINE_AA);

            std::string second_line = statusText();
            if (solution.valid)
            {
                std::ostringstream solved;
                solved << " | best=" << solution.method_name
                    << " " << std::fixed << std::setprecision(2)
                    << solution.translation_rms_m * 1000.0 << "mm/"
                    << solution.rotation_rms_rad * 180.0 / kPi << "deg";
                second_line += solved.str();
            }
            cv::putText(canvas, second_line, cv::Point(12, image.rows + 52),
                        cv::FONT_HERSHEY_SIMPLEX, 0.51, cv::Scalar(160, 220, 255), 1, cv::LINE_AA);

            buttons_.clear();
            const std::array<std::pair<const char*, UiCommand>, 6> definitions = {
                {
                    {"CAPTURE [A]", UiCommand::CAPTURE},
                    {"UNDO [U]", UiCommand::UNDO},
                    {"SOLVE [S]", UiCommand::SOLVE},
                    {"SAVE [W]", UiCommand::SAVE},
                    {"CLEAR [C]", UiCommand::CLEAR},
                    {"QUIT [Q]", UiCommand::QUIT}
                }
            };

            const int button_top = image.rows + information_height;
            const int button_width = std::max(1, image.cols / static_cast<int>(definitions.size()));
            for (std::size_t index = 0; index < definitions.size(); ++index)
            {
                const int x = static_cast<int>(index) * button_width;
                const int width = (index + 1 == definitions.size()) ? image.cols - x : button_width;
                Button button;
                button.rect = cv::Rect(x + 2, button_top + 2, std::max(1, width - 4), button_height - 4);
                button.label = definitions[index].first;
                button.command = definitions[index].second;
                buttons_.push_back(button);

                cv::rectangle(canvas, button.rect, cv::Scalar(70, 70, 70), cv::FILLED);
                cv::rectangle(canvas, button.rect, cv::Scalar(145, 145, 145), 1);

                int baseline = 0;
                const cv::Size text_size = cv::getTextSize(
                    button.label, cv::FONT_HERSHEY_SIMPLEX, 0.48, 1, &baseline);
                const cv::Point origin(
                    button.rect.x + std::max(4, (button.rect.width - text_size.width) / 2),
                    button.rect.y + (button.rect.height + text_size.height) / 2);
                cv::putText(canvas, button.label, origin,
                            cv::FONT_HERSHEY_SIMPLEX, 0.48, cv::Scalar(245, 245, 245), 1, cv::LINE_AA);
            }

            return canvas;
        }

        static void mouseCallbackThunk(int event, int x, int y, int flags, void* userdata)
        {
            (void)flags;
            if (userdata != nullptr)
            {
                static_cast<HandEyeCalibrationNode*>(userdata)->mouseCallback(event, x, y);
            }
        }

        void mouseCallback(int event, int x, int y)
        {
            if (event != cv::EVENT_LBUTTONUP)
            {
                return;
            }
            for (const auto& button : buttons_)
            {
                if (button.rect.contains(cv::Point(x, y)))
                {
                    pending_command_.store(static_cast<int>(button.command));
                    return;
                }
            }
        }

        void handleCommand(UiCommand command)
        {
            switch (command)
            {
            case UiCommand::CAPTURE:
                captureSample();
                break;
            case UiCommand::UNDO:
                undoSample();
                break;
            case UiCommand::SOLVE:
                solveCalibration();
                break;
            case UiCommand::SAVE:
                saveCalibration();
                break;
            case UiCommand::CLEAR:
                clearSamples();
                break;
            case UiCommand::QUIT:
                running_.store(false);
                rclcpp::shutdown();
                break;
            case UiCommand::NONE:
            default:
                break;
            }
        }

        static UiCommand commandFromKey(int key)
        {
            switch (key & 0xFF)
            {
            case 'a':
            case 'A':
                return UiCommand::CAPTURE;
            case 'u':
            case 'U':
                return UiCommand::UNDO;
            case 's':
            case 'S':
                return UiCommand::SOLVE;
            case 'w':
            case 'W':
                return UiCommand::SAVE;
            case 'c':
            case 'C':
                return UiCommand::CLEAR;
            case 'q':
            case 'Q':
            case 27:
                return UiCommand::QUIT;
            default:
                return UiCommand::NONE;
            }
        }

        void uiLoop()
        {
            try
            {
                cv::namedWindow(window_name_, cv::WINDOW_NORMAL);
                cv::setMouseCallback(window_name_, &HandEyeCalibrationNode::mouseCallbackThunk, this);

                const auto detection_period = std::chrono::duration<double>(1.0 / max_detection_hz_);
                const auto ui_period = std::chrono::duration<double>(1.0 / ui_hz_);
                auto last_detection_time = std::chrono::steady_clock::now() - detection_period;

                while (running_.load() && rclcpp::ok())
                {
                    const auto loop_start = std::chrono::steady_clock::now();
                    if (loop_start - last_detection_time >= detection_period)
                    {
                        processLatestImage();
                        last_detection_time = loop_start;
                    }

                    cv::Mat canvas = buildCanvas();
                    cv::imshow(window_name_, canvas);
                    const int key = cv::waitKey(1);

                    UiCommand command = commandFromKey(key);
                    const int mouse_command = pending_command_.exchange(static_cast<int>(UiCommand::NONE));
                    if (mouse_command != static_cast<int>(UiCommand::NONE))
                    {
                        command = static_cast<UiCommand>(mouse_command);
                    }
                    if (command != UiCommand::NONE)
                    {
                        handleCommand(command);
                    }

                    const auto elapsed = std::chrono::steady_clock::now() - loop_start;
                    if (elapsed < ui_period)
                    {
                        std::this_thread::sleep_for(ui_period - elapsed);
                    }
                }
            }
            catch (const cv::Exception& error)
            {
                RCLCPP_FATAL(get_logger(), "OpenCV GUI failed: %s", error.what());
                running_.store(false);
                rclcpp::shutdown();
            }
            catch (const std::exception& error)
            {
                RCLCPP_FATAL(get_logger(), "Hand-eye UI thread failed: %s", error.what());
                running_.store(false);
                rclcpp::shutdown();
            }
        }

        std::string image_topic_;
        std::string camera_info_topic_;
        std::string base_frame_;
        std::string hand_frame_;
        std::string camera_frame_override_;
        std::string camera_root_frame_;

        int squares_x_{7};
        int squares_y_{5};
        double square_length_m_{0.030};
        double marker_length_m_{0.022};
        std::string dictionary_name_;
        double axis_length_m_{0.060};

        int min_charuco_corners_{12};
        double max_reprojection_error_px_{1.5};
        double min_translation_delta_m_{0.030};
        double min_rotation_delta_deg_{8.0};
        int min_samples_{10};
        double rotation_weight_m_per_rad_{0.05};

        double detection_scale_{1.0};
        double max_detection_hz_{15.0};
        double ui_hz_{30.0};
        int opencv_threads_{2};
        int max_display_width_{1280};

        double tf_timeout_sec_{0.25};
        std::string output_directory_;
        bool autosave_samples_{true};
        std::string window_name_;

        cv::Ptr<cv::aruco::Dictionary> dictionary_;
        cv::Ptr<cv::aruco::CharucoBoard> board_;
        cv::Ptr<cv::aruco::DetectorParameters> detector_parameters_;

        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
        rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
        std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

        mutable std::mutex image_mutex_;
        sensor_msgs::msg::Image::ConstSharedPtr latest_image_msg_;
        uint64_t latest_image_sequence_{0};
        uint64_t last_processed_image_sequence_{0};

        mutable std::mutex camera_info_mutex_;
        cv::Mat camera_matrix_;
        cv::Mat distortion_coefficients_;
        uint32_t camera_info_width_{0U};
        uint32_t camera_info_height_{0U};
        bool has_camera_info_{false};

        mutable std::mutex detection_mutex_;
        DetectionSnapshot latest_detection_;

        mutable std::mutex display_mutex_;
        cv::Mat latest_processed_image_;

        mutable std::mutex samples_mutex_;
        std::vector<HandEyeSample> samples_;
        std::string sample_camera_frame_;
        CalibrationSolution latest_solution_;

        mutable std::mutex status_mutex_;
        std::string status_text_{"Move robot, keep board fixed, then click CAPTURE"};

        std::atomic<bool> running_{false};
        std::atomic<int> pending_command_{static_cast<int>(UiCommand::NONE)};
        std::thread ui_thread_;
        std::vector<Button> buttons_;
    };
} // namespace rm_handeye_calibration

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    try
    {
        auto node = std::make_shared<rm_handeye_calibration::HandEyeCalibrationNode>();
        rclcpp::executors::MultiThreadedExecutor executor(
            rclcpp::ExecutorOptions(), 2U);
        executor.add_node(node);
        executor.spin();
        executor.remove_node(node);
        node.reset();
    }
    catch (const std::exception& error)
    {
        RCLCPP_FATAL(rclcpp::get_logger("handeye_calibration"), "%s", error.what());
        rclcpp::shutdown();
        return 1;
    }

    if (rclcpp::ok())
    {
        rclcpp::shutdown();
    }
    return 0;
}
