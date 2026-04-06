#include <iostream>
#include <memory>
#include <cmath>
#include <chrono>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("arm_waypoint_terminal");

    node->declare_parameter<std::string>("waypoint_topic", "/robot_1/waypoint");
    auto waypoint_topic = node->get_parameter("waypoint_topic").as_string();

    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    auto pub = node->create_publisher<std_msgs::msg::Float64MultiArray>(waypoint_topic, qos);

    RCLCPP_INFO(node->get_logger(), "Arm waypoint terminal dang chay.");
    RCLCPP_INFO(node->get_logger(), "Publish topic: %s", waypoint_topic.c_str());
    RCLCPP_INFO(node->get_logger(), "Nhap lien tuc waypoint. Go 'q' de thoat.");

    while (rclcpp::ok()) {
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
        double phi_deg = 0.0;
        std::string input;

        std::cout << "\nNhap x (m) [hoac q de thoat]: " << std::flush;
        if (!(std::cin >> input)) break;
        if (input == "q" || input == "Q") break;
        try { x = std::stod(input); } catch (...) { std::cerr << "x khong hop le\n"; continue; }

        std::cout << "Nhap y (m): " << std::flush;
        if (!(std::cin >> input)) break;
        if (input == "q" || input == "Q") break;
        try { y = std::stod(input); } catch (...) { std::cerr << "y khong hop le\n"; continue; }

        std::cout << "Nhap z (m): " << std::flush;
        if (!(std::cin >> input)) break;
        if (input == "q" || input == "Q") break;
        try { z = std::stod(input); } catch (...) { std::cerr << "z khong hop le\n"; continue; }

        std::cout << "Nhap phi (deg): " << std::flush;
        if (!(std::cin >> input)) break;
        if (input == "q" || input == "Q") break;
        try { phi_deg = std::stod(input); } catch (...) { std::cerr << "phi khong hop le\n"; continue; }

        std_msgs::msg::Float64MultiArray msg;
        msg.data = {x, y, z, phi_deg * M_PI / 180.0};

        rclcpp::sleep_for(std::chrono::milliseconds(100));
        pub->publish(msg);

        RCLCPP_INFO(node->get_logger(),
                    "Published to %s: [x=%.6f, y=%.6f, z=%.6f, phi_deg=%.6f]",
                    waypoint_topic.c_str(), x, y, z, phi_deg);

        rclcpp::spin_some(node);
    }

    RCLCPP_INFO(node->get_logger(), "Tat arm_waypoint_terminal.");
    rclcpp::shutdown();
    return 0;
}