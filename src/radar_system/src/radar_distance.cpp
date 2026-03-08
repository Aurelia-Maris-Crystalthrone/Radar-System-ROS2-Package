#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <std_msgs/msg/float64.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>     
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/convert.h>
#include <cmath>
#include <memory>
#include <optional>

class RadarDistanceNode : public rclcpp::Node
{
public:
    RadarDistanceNode() : Node("radar_distance_node"),
        tf_buffer_(this->get_clock()),
        tf_listener_(tf_buffer_)
    {
        // 1. 声明参数
        this->declare_parameter<std::string>("radar_odom_topic", "/aft_mapped_to_init");
        this->declare_parameter<std::string>("base_point_topic", "/radar/base_point");
        this->declare_parameter<std::string>("target_frame", "map");

        // 2. 声明变量
        std::string radar_topic;
        std::string base_topic;

        // 3. 获取参数
        try {
            radar_topic = this->get_parameter("radar_odom_topic").as_string();
            base_topic = this->get_parameter("base_point_topic").as_string();
            target_frame_ = this->get_parameter("target_frame").as_string();
        } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException& e) {
            RCLCPP_ERROR(this->get_logger(), "参数获取失败: %s", e.what());
            rclcpp::shutdown();
            return;
        }

        // 4. 创建订阅器
        radar_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            radar_topic, 10, std::bind(&RadarDistanceNode::radar_callback, this, std::placeholders::_1));

        base_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
            base_topic, 10, std::bind(&RadarDistanceNode::base_callback, this, std::placeholders::_1));

        // 5. 创建发布器
        distance_pub_ = this->create_publisher<std_msgs::msg::Float64>("/radar/distance", 10);

        RCLCPP_INFO(this->get_logger(), "雷达距离计算节点启动成功！");
        RCLCPP_INFO(this->get_logger(), "雷达里程计话题: %s", radar_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "基地位置话题: %s", base_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "目标坐标系: %s", target_frame_.c_str());
    }

private:
    void radar_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // 校验消息有效性
        if (msg->header.frame_id.empty()) {
            RCLCPP_WARN(this->get_logger(), "雷达里程计frame_id为空，跳过");
            return;
        }

        geometry_msgs::msg::PointStamped radar_point;
        radar_point.header = msg->header;
        radar_point.point = msg->pose.pose.position;

        last_radar_original_ = radar_point;
        try_publish_distance();
    }

    void base_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
    {
        // 校验消息有效性
        if (msg->header.frame_id.empty()) {
            RCLCPP_WARN(this->get_logger(), "基地位置frame_id为空，跳过");
            return;
        }
        last_base_ = *msg;
        try_publish_distance();
    }

    void try_publish_distance()
    {
        // 检查是否获取到有效数据
        if (!last_radar_original_ || !last_base_) {
            RCLCPP_DEBUG(this->get_logger(), "等待雷达/基地位置数据...");
            return;
        }

        geometry_msgs::msg::PointStamped radar_transformed;
        geometry_msgs::msg::PointStamped base_transformed;

        try {
            // 核心修复：正确的TF变换调用方式
            tf_buffer_.transform(
                *last_radar_original_, 
                radar_transformed, 
                target_frame_, 
                tf2::durationFromSec(1.0)
            );
            tf_buffer_.transform(
                *last_base_, 
                base_transformed, 
                target_frame_, 
                tf2::durationFromSec(1.0)
            );
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "TF变换失败: %s", ex.what());
            return;
        }

        // 计算三维距离
        double dx = radar_transformed.point.x - base_transformed.point.x;
        double dy = radar_transformed.point.y - base_transformed.point.y;
        double dz = radar_transformed.point.z - base_transformed.point.z;
        double dist = std::sqrt(dx*dx + dy*dy + dz*dz);

        // 发布距离
        std_msgs::msg::Float64 dist_msg;
        dist_msg.data = dist;
        distance_pub_->publish(dist_msg);

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "雷达到基地距离: %.3f 米", dist);
    }

    // 订阅器/发布器
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr radar_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr base_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr distance_pub_;

    // TF相关（正确的类型声明）
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    // 配置和缓存
    std::string target_frame_;
    std::optional<geometry_msgs::msg::PointStamped> last_radar_original_;
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
