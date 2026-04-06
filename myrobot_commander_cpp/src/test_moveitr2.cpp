#include <thread>
#include <memory>
#include <vector>
#include <array>
#include <iomanip>
#include <sstream>
#include <mutex>
#include <chrono>
#include <cmath>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>

#include "myrobot_commander_cpp/ik_myrobot.hpp"

using myrobot_kinematics::JointArray;

// Helper: Chuyển mảng joint sang chuỗi để in log
static std::string jointsToString(const JointArray& q)
{
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(6)
        << "[" << q[0] << ", " << q[1] << ", " << q[2] << ", " << q[3] << "]";
    return oss.str();
}

// Helper: In thông tin Pose ra màn hình
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

// Helper: Chuyển Matrix4d sang Pose ROS
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
    double x{0.0}, y{0.0}, z{0.0}, phi{0.0};
    bool received{false};
    uint64_t seq{0};
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    // Node sẽ nhận namespace từ file launch (ví dụ: /robot2)
    auto node = std::make_shared<rclcpp::Node>("test_moveitr2");
    std::string ns = node->get_namespace();

    RCLCPP_INFO(node->get_logger(), "===== NODE KHOI DONG TAI NAMESPACE: %s =====", ns.c_str());

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    auto spinner = std::thread([&executor]() { executor.spin(); });

    WaypointData waypoint;
    std::mutex waypoint_mutex;

    // Subscriber dùng topic tương đối "waypoint" -> tự động thành /robot2/waypoint
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    auto sub = node->create_subscription<std_msgs::msg::Float64MultiArray>(
        "waypoint", 
        qos,
        [&](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
            if (msg->data.size() < 4) return;
            std::lock_guard<std::mutex> lock(waypoint_mutex);
            waypoint.x = msg->data[0];
            waypoint.y = msg->data[1];
            waypoint.z = msg->data[2];
            waypoint.phi = msg->data[3];
            waypoint.received = true;
            waypoint.seq++;
            RCLCPP_INFO(node->get_logger(), "Nhan waypoint #%lu", waypoint.seq);
        });

    // --- KHOI TAO MOVEGROUP VOI NAMESPACE DUNG ---
    moveit::planning_interface::MoveGroupInterface::Options arm_options("arm", "robot_description", ns);
    moveit::planning_interface::MoveGroupInterface arm(node, arm_options);

    moveit::planning_interface::MoveGroupInterface::Options gripper_options("gripper", "robot_description", ns);
    moveit::planning_interface::MoveGroupInterface gripper(node, gripper_options);
    (void)gripper;

    arm.setMaxVelocityScalingFactor(0.5);
    arm.setMaxAccelerationScalingFactor(0.5);
    arm.setPlanningTime(10.0);

    RCLCPP_INFO(node->get_logger(), "Dang cho waypoint tai topic: %s/waypoint", ns.c_str());

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

        RCLCPP_INFO(node->get_logger(), "=== Dang xu ly Waypoint #%lu ===", last_processed_seq);

        // Lay trang thai hien tai de tinh IK
        const auto current_joints = arm.getCurrentJointValues();
        
        // Offset mount tren mobile base
        const double x_offset = 0.08248;
        const double z_offset = 0.16124;

        const double x_ik = wp_local.x - x_offset;
        const double y_ik = wp_local.y;
        const double z_ik = wp_local.z - z_offset;

        const Eigen::Matrix4d T_target = myrobot_kinematics::buildT05FromXYZPhi(x_ik, y_ik, z_ik, wp_local.phi);

        JointArray sol1{}, sol2{}, best_sol{};
        if (!myrobot_kinematics::solveIK2Solutions(T_target, sol1, sol2)) {
            RCLCPP_ERROR(node->get_logger(), "IK Failed!");
            continue;
        }

        myrobot_kinematics::chooseBestSolution(T_target, sol1, sol2, current_joints, best_sol);
        std::vector<double> joint_targets = {best_sol[0], best_sol[1], best_sol[2], best_sol[3]};

        // Planning MoveGroup
        arm.setStartStateToCurrentState();
        arm.setJointValueTarget(joint_targets);

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        if (arm.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_INFO(node->get_logger(), "Planning Joint OK. Dang execute...");
            arm.execute(plan);
        } else {
            RCLCPP_ERROR(node->get_logger(), "Planning Joint Failed!");
            continue;
        }

        // Cartesian Path (di chuyen them 2cm theo X)
        std::vector<geometry_msgs::msg::Pose> cart_waypoints;
        geometry_msgs::msg::Pose current_p = arm.getCurrentPose().pose;
        current_p.position.x += 0.02;
        cart_waypoints.push_back(current_p);

        moveit_msgs::msg::RobotTrajectory trajectory;
        double fraction = arm.computeCartesianPath(cart_waypoints, 0.01, trajectory);

        if (fraction >= 0.9) { // Neu thanh cong tren 90%
            RCLCPP_INFO(node->get_logger(), "Cartesian Path OK (%.2f%%). Execute...", fraction*100);
            arm.execute(trajectory);
        }

        RCLCPP_INFO(node->get_logger(), "Hoan thanh Waypoint #%lu", last_processed_seq);
    }

    executor.cancel();
    if (spinner.joinable()) spinner.join();
    rclcpp::shutdown();
    return 0;
}