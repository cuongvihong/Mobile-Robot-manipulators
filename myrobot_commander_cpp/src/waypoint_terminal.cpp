#include <iostream>
#include <memory>
#include <cmath>
#include <chrono>
#include <string>
#include <unordered_map>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

static bool isQuitCommand(const std::string& s)
{
    return s == "q" || s == "Q";
}

static bool isChangeRobotCommand(const std::string& s)
{
    return s == "c" || s == "C";
}

static std::string chooseRobotTopic()
{
    while (true) {
        std::string choice;
        std::cout << "\nChon robot de publish waypoint:\n";
        std::cout << "  1 -> robot1 (/robot1/waypoint)\n";
        std::cout << "  2 -> robot2 (/robot2/waypoint)\n";
        std::cout << "  3 -> robot3 (/robot3/waypoint)\n";
        std::cout << "Nhap lua chon (1/2/3, hoac q de thoat): " << std::flush;

        if (!(std::cin >> choice)) {
            return "";
        }

        if (isQuitCommand(choice)) {
            return "";
        }

        if (choice == "1") return "/robot1/waypoint";
        if (choice == "2") return "/robot2/waypoint";
        if (choice == "3") return "/robot3/waypoint";

        std::cout << "Lua chon khong hop le. Moi nhap lai.\n";
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("waypoint_terminal");

    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();

    std::string waypoint_topic = chooseRobotTopic();
    if (waypoint_topic.empty()) {
        std::cout << "Khong chon robot. Thoat chuong trinh.\n";
        rclcpp::shutdown();
        return 0;
    }

    auto pub = node->create_publisher<std_msgs::msg::Float64MultiArray>(waypoint_topic, qos);

    RCLCPP_INFO(node->get_logger(), "Waypoint terminal dang chay.");
    RCLCPP_INFO(node->get_logger(), "Robot hien tai publish topic: %s", waypoint_topic.c_str());
    RCLCPP_INFO(node->get_logger(), "Nhap waypoint lien tuc.");
    RCLCPP_INFO(node->get_logger(), "Go 'c' o bat ky buoc nhap nao de doi robot.");
    RCLCPP_INFO(node->get_logger(), "Go 'q' o bat ky buoc nhap nao de thoat.");

    while (rclcpp::ok()) {
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
        double phi_deg = 0.0;

        std::string input;

        std::cout << "\nRobot hien tai: " << waypoint_topic << "\n";

        std::cout << "Nhap x (m) [c: doi robot, q: thoat]: " << std::flush;
        if (!(std::cin >> input)) {
            std::cerr << "Loi doc du lieu. Thoat chuong trinh.\n";
            break;
        }
        if (isQuitCommand(input)) {
            break;
        }
        if (isChangeRobotCommand(input)) {
            waypoint_topic = chooseRobotTopic();
            if (waypoint_topic.empty()) {
                break;
            }
            pub = node->create_publisher<std_msgs::msg::Float64MultiArray>(waypoint_topic, qos);
            RCLCPP_INFO(node->get_logger(), "Da chuyen sang topic: %s", waypoint_topic.c_str());
            continue;
        }
        try {
            x = std::stod(input);
        } catch (...) {
            std::cerr << "Gia tri x khong hop le. Moi nhap lai.\n";
            continue;
        }

        std::cout << "Nhap y (m) [c: doi robot, q: thoat]: " << std::flush;
        if (!(std::cin >> input)) {
            std::cerr << "Loi doc du lieu. Thoat chuong trinh.\n";
            break;
        }
        if (isQuitCommand(input)) {
            break;
        }
        if (isChangeRobotCommand(input)) {
            waypoint_topic = chooseRobotTopic();
            if (waypoint_topic.empty()) {
                break;
            }
            pub = node->create_publisher<std_msgs::msg::Float64MultiArray>(waypoint_topic, qos);
            RCLCPP_INFO(node->get_logger(), "Da chuyen sang topic: %s", waypoint_topic.c_str());
            continue;
        }
        try {
            y = std::stod(input);
        } catch (...) {
            std::cerr << "Gia tri y khong hop le. Moi nhap lai.\n";
            continue;
        }

        std::cout << "Nhap z (m) [c: doi robot, q: thoat]: " << std::flush;
        if (!(std::cin >> input)) {
            std::cerr << "Loi doc du lieu. Thoat chuong trinh.\n";
            break;
        }
        if (isQuitCommand(input)) {
            break;
        }
        if (isChangeRobotCommand(input)) {
            waypoint_topic = chooseRobotTopic();
            if (waypoint_topic.empty()) {
                break;
            }
            pub = node->create_publisher<std_msgs::msg::Float64MultiArray>(waypoint_topic, qos);
            RCLCPP_INFO(node->get_logger(), "Da chuyen sang topic: %s", waypoint_topic.c_str());
            continue;
        }
        try {
            z = std::stod(input);
        } catch (...) {
            std::cerr << "Gia tri z khong hop le. Moi nhap lai.\n";
            continue;
        }

        std::cout << "Nhap phi (deg) [c: doi robot, q: thoat]: " << std::flush;
        if (!(std::cin >> input)) {
            std::cerr << "Loi doc du lieu. Thoat chuong trinh.\n";
            break;
        }
        if (isQuitCommand(input)) {
            break;
        }
        if (isChangeRobotCommand(input)) {
            waypoint_topic = chooseRobotTopic();
            if (waypoint_topic.empty()) {
                break;
            }
            pub = node->create_publisher<std_msgs::msg::Float64MultiArray>(waypoint_topic, qos);
            RCLCPP_INFO(node->get_logger(), "Da chuyen sang topic: %s", waypoint_topic.c_str());
            continue;
        }
        try {
            phi_deg = std::stod(input);
        } catch (...) {
            std::cerr << "Gia tri phi_deg khong hop le. Moi nhap lai.\n";
            continue;
        }

        const double phi_rad = phi_deg * M_PI / 180.0;

        std_msgs::msg::Float64MultiArray msg;
        msg.data = {x, y, z, phi_rad};

        rclcpp::sleep_for(std::chrono::milliseconds(100));
        pub->publish(msg);

        RCLCPP_INFO(node->get_logger(),
                    "Published to %s: [x=%.6f, y=%.6f, z=%.6f, phi_deg=%.6f, phi_rad=%.6f]",
                    waypoint_topic.c_str(), x, y, z, phi_deg, phi_rad);

        rclcpp::spin_some(node);
    }

    RCLCPP_INFO(node->get_logger(), "Tat waypoint_terminal.");
    rclcpp::shutdown();
    return 0;
}