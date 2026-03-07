#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>

void distance_callback(const std_msgs::msg::Float64::SharedPtr msg)
{
    RCLCPP_INFO(rclcpp::get_logger("distance_monitor"), "雷达与基地距离: %.3f 米", msg->data);
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("distance_monitor");
    auto sub = node->create_subscription<std_msgs::msg::Float64>("/radar/distance", 10, distance_callback);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}