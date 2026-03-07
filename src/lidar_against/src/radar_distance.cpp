#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <std_msgs/msg/float64.hpp>
#include <cmath>
#include <memory>
#include <optional>

class RadarDistanceNode : public rclcpp::Node
{
public:
    RadarDistanceNode() : Node("radar_distance_node")
    {
        // 订阅雷达位置
        radar_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
            "/radar/position", 10,
            std::bind(&RadarDistanceNode::radar_callback, this, std::placeholders::_1));

        // 订阅基地位置
        base_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
            "/radar/base_point", 10,
            std::bind(&RadarDistanceNode::base_callback, this, std::placeholders::_1));

        // 发布距离
        distance_pub_ = this->create_publisher<std_msgs::msg::Float64>("/radar/distance", 10);

        RCLCPP_INFO(this->get_logger(), "雷达距离计算节点已启动，等待数据...");
    }

private:
    void radar_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
    {
        last_radar_ = *msg;
        try_publish_distance();
    }

    void base_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
    {
        last_base_ = *msg;
        try_publish_distance();
    }

    void try_publish_distance()
    {
        if (!last_radar_ || !last_base_) {
            return; // 尚未收到两个点
        }

        // 可选：检查 frame_id 是否一致，若不一致可添加 TF 变换（本示例略）
        if (last_radar_->header.frame_id != last_base_->header.frame_id) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "雷达和基地的坐标系不同：雷达在 '%s'，基地在 '%s'。距离计算可能不准确。",
                last_radar_->header.frame_id.c_str(), last_base_->header.frame_id.c_str());
            // 为简化，仍然继续计算，但用户应确保坐标系一致
        }

        double dx = last_radar_->point.x - last_base_->point.x;
        double dy = last_radar_->point.y - last_base_->point.y;
        double dz = last_radar_->point.z - last_base_->point.z;
        double dist = std::sqrt(dx*dx + dy*dy + dz*dz);

        auto dist_msg = std_msgs::msg::Float64();
        dist_msg.data = dist;

        distance_pub_->publish(dist_msg);
        RCLCPP_DEBUG(this->get_logger(), "计算距离: %.3f 米", dist);
    }

    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr radar_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr base_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr distance_pub_;

    std::optional<geometry_msgs::msg::PointStamped> last_radar_;
    std::optional<geometry_msgs::msg::PointStamped> last_base_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RadarDistanceNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}