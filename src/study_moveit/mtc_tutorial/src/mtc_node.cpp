#include <sstream>
//ROS2
#include <rclcpp/rclcpp.hpp>
//moveit2
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
#if __has_include(<tf2_geometry_msgs/tf2_geometry_msgs.hpp>)
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif
#if __has_include(<tf2_eigen/tf2_eigen.hpp>)
#include <tf2_eigen/tf2_eigen.hpp>
#else
#include <tf2_eigen/tf2_eigen.h>
#endif

static const rclcpp::Logger LOGGER = rclcpp::get_logger("mtc_tutorial");
namespace mtc = moveit::task_constructor;

class MTCTaskNode
{
public:
    MTCTaskNode(const rclcpp::NodeOptions& options);

    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();

    void doTask();

    void setupPlanningScene();

private:
    // Compose an MTC task from a series of stages.
    mtc::Task createTask();
    mtc::Task task_;
    rclcpp::Node::SharedPtr node_;
};

//初始化构造函数
MTCTaskNode::MTCTaskNode(const rclcpp::NodeOptions& options)
    : node_{std::make_shared<rclcpp::Node>("mtc_node", options)}
{
}

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr MTCTaskNode::getNodeBaseInterface()
{
    return node_->get_node_base_interface();
}

//创建scene,这里是机械臂抓取的圆柱体碰撞体
void MTCTaskNode::setupPlanningScene()
{
    moveit_msgs::msg::CollisionObject object;
    object.id = "object";
    object.header.frame_id = "world";
    object.primitives.resize(1);
    object.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
    object.primitives[0].dimensions = {0.1, 0.02};

    geometry_msgs::msg::Pose pose;
    pose.position.x = 0.6;
    pose.position.y = -0.2;
    pose.position.z = 0.20;
    pose.orientation.w = 1.0;
    object.pose = pose;

    moveit::planning_interface::PlanningSceneInterface psi;
    if (!psi.applyCollisionObject(object))
    {
        RCLCPP_ERROR(
            LOGGER,
            "Failed to add collision object");
    }
}

//创建，规划，执行任务
void MTCTaskNode::doTask()
{
    try
    {
        task_ = createTask();
        task_.init();
    }
    catch (const mtc::InitStageException& e)
    {
        RCLCPP_ERROR_STREAM(LOGGER, "MTC initialization failed:\n" << e);
        return;
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(LOGGER, "MTC exception: %s", e.what());
        return;
    }

    // 调试阶段先只寻找一个完整解
    const auto result = task_.plan(1);

    // 无论成功或失败，都打印 Stage 状态
    std::ostringstream state_stream;
    task_.printState(state_stream);

    RCLCPP_INFO_STREAM(
        LOGGER,
        "MTC task state:\n" << state_stream.str());

    if (!result)
    {
        std::ostringstream failure_stream;
        task_.explainFailure(failure_stream);

        RCLCPP_ERROR_STREAM(
            LOGGER,
            "Task planning failed:\n"
                << failure_stream.str());

        return;
    }

    if (task_.solutions().empty())
    {
        RCLCPP_ERROR(LOGGER, "No complete MTC solution found");
        return;
    }

    task_.introspection().publishSolution(
        *task_.solutions().front());

    // 先注释执行，确认 RViz 中整条轨迹正确

     const auto execute_result =
         task_.execute(*task_.solutions().front());

     if (execute_result.val !=
         moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
     {
       RCLCPP_ERROR(
           LOGGER,
           "Task execution failed, code: %d",
           execute_result.val);
       return;
     }

}

//创建Task
mtc::Task MTCTaskNode::createTask()
{
    mtc::Task task;

    task.stages()->setName("attach transport detach");
    task.loadRobotModel(node_);

    const std::string arm_group_name = "rm_robot_arm";
    const std::string end_effector_frame = "end_effect_link";
    const std::string world_frame = "world";
    const std::string object_id = "object";

    // 只设置机械臂规划组和 IK 坐标系
    // 没有可开合的夹爪，因此不设置 eef 属性
    task.setProperty("group", arm_group_name);
    task.setProperty("ik_frame", end_effector_frame);

    // 保存 CurrentState 指针：
    // GeneratePose 后面需要监听它
    mtc::Stage* current_state_ptr = nullptr;

    // 保存 Attach Object Stage：
    // GeneratePlacePose 需要知道物体如何附着
    mtc::Stage* attach_object_stage = nullptr;

    /*
     * Stage 1：获取当前机械臂状态
     */
    {
        auto stage = std::make_unique<mtc::stages::CurrentState>("current state");
        current_state_ptr = stage.get();
        task.add(std::move(stage));
    }

    /*
     * 规划器
     */

    // 自由空间运动规划，例如 OMPL
    auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_);

    // 末端笛卡尔直线规划
    auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
    cartesian_planner->setMaxVelocityScalingFactor(0.3);
    cartesian_planner->setMaxAccelerationScalingFactor(0.3);
    cartesian_planner->setStepSize(0.005);

    /*
     * Stage 2：
     * 将当前状态连接到抓取前状态
     */
    {
        auto stage = std::make_unique<mtc::stages::Connect>("move to pre-pick", mtc::stages::Connect::GroupPlannerVector{{arm_group_name, sampling_planner}});
        stage->setTimeout(10.0);
        stage->properties().configureInitFrom(mtc::Stage::PARENT);
        task.add(std::move(stage));
    }

    /*
     * Stage 3：
     * 靠近物体、附着、抬起
     */
    {
        auto pick = std::make_unique<mtc::SerialContainer>("pick object");
        task.properties().exposeTo(pick->properties(), {"group", "ik_frame"});

        pick->properties().configureInitFrom(mtc::Stage::PARENT, {"group", "ik_frame"});

        /*
         * GeneratePose 在后面产生最终目标状态
         * MoveRelative 会从最终目标反向推导抓取前状态
         */
        {
            auto stage = std::make_unique<mtc::stages::MoveRelative>("approach object", cartesian_planner);
            stage->properties().set(
                "marker_ns",
                "approach_object");

            stage->properties().set(
                "link",
                end_effector_frame);

            stage->properties().configureInitFrom(
                mtc::Stage::PARENT,
                {
                    "group"
                });

            // 接近距离为 5～10 cm
            stage->setMinMaxDistance(0.01, 0.10);

            geometry_msgs::msg::Vector3Stamped direction;

            direction.header.frame_id =
                world_frame;

            direction.vector.x = -1.0;

            stage->setDirection(direction);

            pick->insert(std::move(stage));
        }

        /*
         * 3.2 生成末端吸附物体时的目标位姿
         */
        {
            auto generator =
                std::make_unique<
                    mtc::stages::GeneratePose>(
                    "generate attach pose");

            generator->properties()
                     .configureInitFrom(
                         mtc::Stage::PARENT);

            generator->properties().set(
                "marker_ns",
                "attach_pose");

            geometry_msgs::msg::PoseStamped target_pose;

            target_pose.header.frame_id =
                world_frame;

            // 圆柱中心为 (0.60, -0.20, 0.30)
            // 从圆柱 X 正方向一侧接近
            target_pose.pose.position.x = 0.67;
            target_pose.pose.position.y = -0.20;
            target_pose.pose.position.z = 0.20;

            target_pose.pose.orientation.w = 1.0;

            /*
             * 当前是单位四元数。
             * 如果末端方向不对，需要修改姿态。
             */
            target_pose.pose.orientation.w = 1.0;

            generator->setPose(target_pose);

            generator->setMonitoredStage(
                current_state_ptr);

            /*
             * 使用 ComputeIK 将末端位姿转换为关节状态
             */
            auto wrapper =
                std::make_unique<
                    mtc::stages::ComputeIK>(
                    "attach pose IK",
                    std::move(generator));

            wrapper->setMaxIKSolutions(16);

            wrapper->setMinSolutionDistance(0.1);

            wrapper->setIKFrame(
                end_effector_frame);

            wrapper->properties()
                   .configureInitFrom(
                       mtc::Stage::PARENT,
                       {
                           "group"
                       });

            wrapper->properties()
                   .configureInitFrom(
                       mtc::Stage::INTERFACE,
                       {
                           "target_pose"
                       });

            pick->insert(std::move(wrapper));
        }

        /*
         * 3.3 允许末端和物体接触
         */
        {
            auto stage =
                std::make_unique<
                    mtc::stages::
                    ModifyPlanningScene>(
                    "allow collision");

            stage->allowCollisions(
                object_id,
                end_effector_frame,
                true);

            pick->insert(std::move(stage));
        }

        /*
         * 3.4 将物体附着到末端
         */
        {
            auto stage =
                std::make_unique<
                    mtc::stages::
                    ModifyPlanningScene>(
                    "attach object");

            stage->attachObject(
                object_id,
                end_effector_frame);

            attach_object_stage =
                stage.get();

            pick->insert(std::move(stage));
        }

        /*
         * 3.5 向上抬起物体
         */
        {
            auto stage =
                std::make_unique<
                    mtc::stages::MoveRelative>(
                    "lift object",
                    cartesian_planner);

            stage->properties()
                 .configureInitFrom(
                     mtc::Stage::PARENT,
                     {
                         "group"
                     });

            stage->setIKFrame(
                end_effector_frame);

            // 向上抬升 10～15 cm
            stage->setMinMaxDistance(
                0.01,
                0.3);

            geometry_msgs::msg::Vector3Stamped direction;

            direction.header.frame_id =
                world_frame;

            direction.vector.z = 1.0;

            stage->setDirection(direction);

            pick->insert(std::move(stage));
        }

        task.add(std::move(pick));
    }

    /*
     * Stage 4：
     * 将物体从抓取区域移动到放置区域
     */
    {
        auto stage =
            std::make_unique<
                mtc::stages::Connect>(
                "move to pre-place",

                mtc::stages::Connect::
                GroupPlannerVector{
                    {
                        arm_group_name,
                        sampling_planner
                    }
                });

        stage->setTimeout(10.0);

        stage->properties()
             .configureInitFrom(
                 mtc::Stage::PARENT);

        task.add(std::move(stage));
    }

    /*
     * Stage 5：
     * 下放物体、分离、撤离
     */
    {
        auto place =
            std::make_unique<
                mtc::SerialContainer>(
                "place object");

        task.properties().exposeTo(
            place->properties(),
            {
                "group",
                "ik_frame"
            });

        place->properties().configureInitFrom(
            mtc::Stage::PARENT,
            {
                "group",
                "ik_frame"
            });

        /*
         * 5.1 向下移动到最终放置位置
         */
        {
            auto stage =
                std::make_unique<
                    mtc::stages::MoveRelative>(
                    "lower object",
                    cartesian_planner);

            stage->properties()
                 .configureInitFrom(
                     mtc::Stage::PARENT,
                     {
                         "group"
                     });

            stage->setIKFrame(
                end_effector_frame);

            stage->setMinMaxDistance(
                0.01,
                0.30);

            geometry_msgs::msg::Vector3Stamped direction;

            direction.header.frame_id =
                world_frame;

            direction.vector.z = -1.0;

            stage->setDirection(direction);

            place->insert(std::move(stage));
        }

        /*
         * 5.2 生成物体的最终放置位姿
         */
        {
            auto generator =
                std::make_unique<
                    mtc::stages::
                    GeneratePlacePose>(
                    "generate place pose");

            generator->properties()
                     .configureInitFrom(
                         mtc::Stage::PARENT);

            generator->properties().set(
                "marker_ns",
                "place_pose");

            generator->setObject(
                object_id);

            geometry_msgs::msg::PoseStamped target_pose;

            target_pose.header.frame_id =
                world_frame;

            // 物体从 (0.60, -0.20, 0.30)
            // 小范围移动到 (0.55, -0.15, 0.30)
            target_pose.pose.position.x = 0.55;
            target_pose.pose.position.y = -0.15;
            target_pose.pose.position.z = 0.20;

            target_pose.pose.orientation.w = 1.0;

            generator->setPose(target_pose);

            /*
             * 让 GeneratePlacePose 知道：
             * 物体之前附着到了哪个 Link，
             * 以及它与末端之间的相对变换。
             */
            generator->setMonitoredStage(
                attach_object_stage);

            auto wrapper =
                std::make_unique<
                    mtc::stages::ComputeIK>(
                    "place pose IK",
                    std::move(generator));

            wrapper->setMaxIKSolutions(16);

            wrapper->setMinSolutionDistance(0.1);

            /*
             * 这里使用物体坐标系作为 IK Frame。
             * 因为目标位姿描述的是物体最终位姿。
             */
            wrapper->setIKFrame(
                object_id);

            wrapper->properties()
                   .configureInitFrom(
                       mtc::Stage::PARENT,
                       {
                           "group"
                       });

            wrapper->properties()
                   .configureInitFrom(
                       mtc::Stage::INTERFACE,
                       {
                           "target_pose"
                       });

            place->insert(std::move(wrapper));
        }

        /*
         * 5.3 分离物体
         */
        {
            auto stage =
                std::make_unique<
                    mtc::stages::
                    ModifyPlanningScene>(
                    "detach object");

            stage->detachObject(
                object_id,
                end_effector_frame);

            place->insert(std::move(stage));
        }

        /*
         * 5.4 末端向上撤离
         *
         * 暂时保持末端和物体之间允许碰撞，
         * 避免刚分离时因为两者仍然接近而规划失败。
         */
        {
            auto stage =
                std::make_unique<
                    mtc::stages::MoveRelative>(
                    "retreat from object",
                    cartesian_planner);

            stage->properties()
                 .configureInitFrom(
                     mtc::Stage::PARENT,
                     {
                         "group"
                     });

            stage->setIKFrame(
                end_effector_frame);

            stage->setMinMaxDistance(
                0.01,
                0.30);

            geometry_msgs::msg::Vector3Stamped direction;

            direction.header.frame_id = world_frame;

            direction.vector.x = 1.0;
            direction.vector.y = 0.0;
            direction.vector.z = 0.0;

            stage->setDirection(direction);

            place->insert(std::move(stage));
        }

        /*
         * 5.5 末端离开物体后恢复正常碰撞检测
         */
        {
            auto stage =
                std::make_unique<
                    mtc::stages::
                    ModifyPlanningScene>(
                    "forbid collision");

            stage->allowCollisions(
                object_id,
                end_effector_frame,
                false);

            place->insert(std::move(stage));
        }

        task.add(std::move(place));
    }

    return task;
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions options;
    options.automatically_declare_parameters_from_overrides(true);

    auto mtc_task_node = std::make_shared<MTCTaskNode>(options);
    rclcpp::executors::MultiThreadedExecutor executor;

    auto spin_thread = std::make_unique<std::thread>([&executor, &mtc_task_node]()
    {
        executor.add_node(mtc_task_node->getNodeBaseInterface());
        executor.spin();
        executor.remove_node(mtc_task_node->getNodeBaseInterface());
    });

    mtc_task_node->setupPlanningScene();
    mtc_task_node->doTask();

    spin_thread->join();
    rclcpp::shutdown();
    return 0;
}
