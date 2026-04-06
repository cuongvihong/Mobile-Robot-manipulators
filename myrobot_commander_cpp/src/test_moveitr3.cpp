#include <thread>
#include <memory>
#include <vector>
#include <array>
#include <iomanip>
#include <sstream>
#include <mutex>
#include <chrono>
#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>

#include "myrobot_commander_cpp/ik_myrobot.hpp"

using myrobot_kinematics::JointArray;

static std::string jointsToString(const JointArray& q)
{
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(6)
        << "[" << q[0] << ", " << q[1] << ", " << q[2] << ", " << q[3] << "]";
    return oss.str();
}

static void printPose(rclcpp::Logger logger, const std::string& label, const geometry_msgs::msg::Pose& p)
{
    RCLCPP_INFO(
        logger,
        "%s: pos[x=%.6f, y=%.6f, z=%.6f] quat[x=%.6f, y=%.6f, z=%.6f, w=%.6f]",
        label.c_str(),
        p.position.x, p.position.y, p.position.z,
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w
    );
}

static geometry_msgs::msg::Pose eigenToRosPose(const Eigen::Matrix4d& T)
{
    geometry_msgs::msg::Pose p;
    p.position.x = T(0, 3);
    p.position.y = T(1, 3);
    p.position.z = T(2, 3);

    Eigen::Matrix3d R = T.block<3, 3>(0, 0);
    Eigen::Quaterniond q(R);
    q.normalize();

    p.orientation.x = q.x();
    p.orientation.y = q.y();
    p.orientation.z = q.z();
    p.orientation.w = q.w();
    return p;
}

struct WaypointData
{
    double x{0.0};
    double y{0.0};
    double z{0.0};
    double phi{0.0};
    bool received{false};
    uint64_t seq{0};
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("test_moveit");
    RCLCPP_INFO(node->get_logger(), "===== DEBUG BUILD ACTIVE =====");

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    auto spinner = std::thread([&executor]() { executor.spin(); });

    WaypointData waypoint;
    std::mutex waypoint_mutex;

    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();

    auto sub = node->create_subscription<std_msgs::msg::Float64MultiArray>(
        "waypoint",
        qos,
        [&](const std_msgs::msg::Float64MultiArray::SharedPtr msg)
        {
            if (msg->data.size() < 4) {
                RCLCPP_ERROR(
                    node->get_logger(),
                    "Nhan waypoint khong hop le. Can 4 gia tri [x, y, z, phi_rad], nhung chi co %zu",
                    msg->data.size()
                );
                return;
            }

            std::lock_guard<std::mutex> lock(waypoint_mutex);
            waypoint.x = msg->data[0];
            waypoint.y = msg->data[1];
            waypoint.z = msg->data[2];
            waypoint.phi = msg->data[3];
            waypoint.received = true;
            waypoint.seq++;

            RCLCPP_INFO(
                node->get_logger(),
                "Da nhan waypoint #%lu: x=%.6f y=%.6f z=%.6f phi=%.6f rad",
                waypoint.seq, waypoint.x, waypoint.y, waypoint.z, waypoint.phi
            );
        });

    RCLCPP_INFO(node->get_logger(), "Creating MoveGroupInterface for arm...");
    moveit::planning_interface::MoveGroupInterface::Options arm_options(
        "arm",
        "robot_description",
        "/robot3"
    );
    moveit::planning_interface::MoveGroupInterface arm(node, arm_options);
    RCLCPP_INFO(node->get_logger(), "Created MoveGroupInterface for arm");

    RCLCPP_INFO(node->get_logger(), "Creating MoveGroupInterface for gripper...");
    moveit::planning_interface::MoveGroupInterface::Options gripper_options(
        "gripper",
        "robot_description",
        "/robot3"
    );
    moveit::planning_interface::MoveGroupInterface gripper(node, gripper_options);
    RCLCPP_INFO(node->get_logger(), "Created MoveGroupInterface for gripper");

    (void)gripper;

    arm.setMaxVelocityScalingFactor(0.5);
    arm.setMaxAccelerationScalingFactor(0.5);
    arm.setPlanningTime(10.0);
    arm.setNumPlanningAttempts(10);

    RCLCPP_INFO(node->get_logger(), "MoveIt planning frame: %s", arm.getPlanningFrame().c_str());
    RCLCPP_INFO(node->get_logger(), "MoveIt end effector link: %s", arm.getEndEffectorLink().c_str());

    const auto joint_names = arm.getJointNames();
    for (size_t i = 0; i < joint_names.size(); ++i) {
        RCLCPP_INFO(node->get_logger(), "arm joint[%zu] = %s", i, joint_names[i].c_str());
    }

    RCLCPP_INFO(node->get_logger(), "Dang doi waypoint tu topic relative 'waypoint' ...");
    RCLCPP_INFO(node->get_logger(), "Node se chi thoat khi nhan Ctrl+C.");

    uint64_t last_processed_seq = 0;

    while (rclcpp::ok()) {
        WaypointData wp_local;
        bool has_new_waypoint = false;

        {
            std::lock_guard<std::mutex> lock(waypoint_mutex);
            if (waypoint.received && waypoint.seq > last_processed_seq) {
                wp_local = waypoint;
                last_processed_seq = waypoint.seq;
                has_new_waypoint = true;
            }
        }

        if (!has_new_waypoint) {
            rclcpp::sleep_for(std::chrono::milliseconds(100));
            continue;
        }

        const double x = wp_local.x;
        const double y = wp_local.y;
        const double z = wp_local.z;
        const double phi = wp_local.phi;

        RCLCPP_INFO(node->get_logger(), "=== Start processing waypoint #%lu ===", last_processed_seq);

        RCLCPP_INFO(node->get_logger(), "A0 waiting for current robot state...");
        moveit::core::RobotStatePtr current_state = arm.getCurrentState(5.0);

        if (!current_state) {
            RCLCPP_ERROR(node->get_logger(), "A0 failed: khong lay duoc current robot state trong 5 giay");
            continue;
        }
        RCLCPP_INFO(node->get_logger(), "A0 ok: da co current robot state");

        RCLCPP_INFO(node->get_logger(), "A1 before getCurrentJointValues");
        const auto current_joints = arm.getCurrentJointValues();
        RCLCPP_INFO(node->get_logger(), "A2 after getCurrentJointValues");

        for (size_t i = 0; i < current_joints.size(); ++i) {
            RCLCPP_INFO(node->get_logger(), "current joint[%zu] = %.6f", i, current_joints[i]);
        }

        RCLCPP_INFO(node->get_logger(), "A3 before getCurrentPose");
        const auto current_pose_before = arm.getCurrentPose();
        RCLCPP_INFO(node->get_logger(), "A4 after getCurrentPose");

        printPose(node->get_logger(), "Current EE pose before planning", current_pose_before.pose);

        const double x_shift = 0.00;
        const double x_offset_mobile = 0.08248;
        const double z_offset_mobile = 0.16124;

        const double x_ik = x - x_offset_mobile + x_shift;
        const double y_ik = y;
        const double z_ik = z - z_offset_mobile;

        RCLCPP_INFO(
            node->get_logger(),
            "Input target (mobile/base): x=%.6f y=%.6f z=%.6f phi=%.6f",
            x, y, z, phi
        );
        RCLCPP_INFO(
            node->get_logger(),
            "IK target after offset:    x=%.6f y=%.6f z=%.6f phi=%.6f",
            x_ik, y_ik, z_ik, phi
        );

        const Eigen::Matrix4d T_target =
            myrobot_kinematics::buildT05FromXYZPhi(x_ik, y_ik, z_ik, phi);

        const geometry_msgs::msg::Pose goal_pose_from_target = eigenToRosPose(T_target);
        printPose(node->get_logger(), "Goal pose built for IK", goal_pose_from_target);

        JointArray sol1{}, sol2{};
        const bool ik_ok = myrobot_kinematics::solveIK2Solutions(T_target, sol1, sol2);

        if (!ik_ok) {
            RCLCPP_ERROR(node->get_logger(), "IK failed: target unreachable");
            continue;
        }

        RCLCPP_INFO(node->get_logger(), "IK sol1 = %s", jointsToString(sol1).c_str());
        RCLCPP_INFO(node->get_logger(), "IK sol2 = %s", jointsToString(sol2).c_str());

        const Eigen::Matrix4d T1 = myrobot_kinematics::forwardKinematics(sol1);
        const Eigen::Matrix4d T2 = myrobot_kinematics::forwardKinematics(sol2);

        const double err1 = myrobot_kinematics::poseError(T_target, T1);
        const double err2 = myrobot_kinematics::poseError(T_target, T2);

        RCLCPP_INFO(node->get_logger(), "FK verify error sol1 = %.12f", err1);
        RCLCPP_INFO(node->get_logger(), "FK verify error sol2 = %.12f", err2);

        printPose(node->get_logger(), "FK pose from sol1", eigenToRosPose(T1));
        printPose(node->get_logger(), "FK pose from sol2", eigenToRosPose(T2));

        JointArray best_sol{};
        myrobot_kinematics::chooseBestSolution(T_target, sol1, sol2, current_joints, best_sol);

        RCLCPP_INFO(node->get_logger(), "Chosen IK solution = %s", jointsToString(best_sol).c_str());

        const Eigen::Matrix4d T_best = myrobot_kinematics::forwardKinematics(best_sol);
        printPose(node->get_logger(), "Pose from chosen IK solution", eigenToRosPose(T_best));

        std::vector<double> joints = {best_sol[0], best_sol[1], best_sol[2], best_sol[3]};
        RCLCPP_INFO(
            node->get_logger(),
            "Joint target = [%.6f, %.6f, %.6f, %.6f]",
            joints[0], joints[1], joints[2], joints[3]
        );

        arm.setStartStateToCurrentState();
        arm.setJointValueTarget(joints);

        RCLCPP_INFO(node->get_logger(), "B1 before arm.plan()");
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        const bool success = (arm.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
        RCLCPP_INFO(node->get_logger(), "B2 after arm.plan()");

        if (success) {
            RCLCPP_INFO(node->get_logger(), "Joint-space planning succeeded");

            RCLCPP_INFO(node->get_logger(), "C1 before arm.execute(plan)");
            auto exec_result = arm.execute(plan);
            RCLCPP_INFO(node->get_logger(), "C2 after arm.execute(plan)");

            if (exec_result == moveit::core::MoveItErrorCode::SUCCESS) {
                const auto current_pose_after = arm.getCurrentPose();
                printPose(node->get_logger(), "Current EE pose after execute", current_pose_after.pose);
            } else {
                RCLCPP_ERROR(node->get_logger(), "Execute failed");
            }
        } else {
            RCLCPP_ERROR(node->get_logger(), "Joint-space planning failed");
            continue;
        }

        std::vector<geometry_msgs::msg::Pose> waypoints;
        geometry_msgs::msg::Pose pose1 = arm.getCurrentPose().pose;
        pose1.position.x += 0.02;
        waypoints.push_back(pose1);

        moveit_msgs::msg::RobotTrajectory trajectory;

        RCLCPP_INFO(node->get_logger(), "D1 before computeCartesianPath()");
        double fraction = arm.computeCartesianPath(waypoints, 0.01, trajectory);
        RCLCPP_INFO(node->get_logger(), "D2 after computeCartesianPath(), fraction = %.6f", fraction);

        if (fraction == 1.0) {
            RCLCPP_INFO(node->get_logger(), "D3 before execute(cartesian)");
            auto cart_exec_result = arm.execute(trajectory);
            RCLCPP_INFO(node->get_logger(), "D4 after execute(cartesian)");

            if (cart_exec_result != moveit::core::MoveItErrorCode::SUCCESS) {
                RCLCPP_ERROR(node->get_logger(), "Cartesian execute failed");
            }
        } else {
            RCLCPP_WARN(node->get_logger(), "Cartesian path khong dat 100%%, bo qua execute");
        }

        RCLCPP_INFO(
            node->get_logger(),
            "Xu ly xong waypoint #%lu. Dang cho waypoint tiep theo...",
            last_processed_seq
        );
    }

    RCLCPP_INFO(node->get_logger(), "Nhan Ctrl+C, dang tat node test_moveit...");

    executor.cancel();
    if (spinner.joinable()) {
        spinner.join();
    }

    rclcpp::shutdown();
    return 0;
}