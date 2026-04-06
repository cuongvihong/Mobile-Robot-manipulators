#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("test_moveit");
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    auto spinner = std::thread([&executor]() { executor.spin(); });

    auto arm = moveit::planning_interface::MoveGroupInterface(node, "arm");
    arm.setMaxVelocityScalingFactor(1.0);
    arm.setMaxAccelerationScalingFactor(1.0);

    auto gripper = moveit::planning_interface::MoveGroupInterface(node, "gripper");



    
    // Named goal

    gripper.setStartStateToCurrentState();
    gripper.setNamedTarget("close");
    
    moveit::planning_interface::MoveGroupInterface::Plan plan1;
    bool success1 = (gripper.plan(plan1) == moveit::core::MoveItErrorCode::SUCCESS);

    if (success1) {
        gripper.execute(plan1);
    }

    gripper.setStartStateToCurrentState();
    gripper.setNamedTarget("open");
    
    moveit::planning_interface::MoveGroupInterface::Plan plan2;
    bool success2 = (gripper.plan(plan2) == moveit::core::MoveItErrorCode::SUCCESS);

    if (success2) {
        gripper.execute(plan2);
    }

    rclcpp::shutdown();
    spinner.join();
    return 0;
}