#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

class BaseTargetPublisher : public rclcpp::Node
{
public:
    BaseTargetPublisher() : Node("base_target_publisher")
    {
        // 声明参数：基地在 map 坐标系下的坐标
        this->declare_parameter<double>("base_x", 0.0);
        this->declare_parameter<double>("base_y", 0.0);
        this->declare_parameter<double>("base_z", 0.0);
        this->declare_parameter<std::string>("target_frame", "base_link");
        this->declare_parameter<double>("publish_rate", 10.0);  

        // 获取参数
        base_x_ = this->get_parameter("base_x").as_double();
        base_y_ = this->get_parameter("base_y").as_double();
        base_z_ = this->get_parameter("base_z").as_double();
        target_frame_ = this->get_parameter("target_frame").as_string();
        double rate = this->get_parameter("publish_rate").as_double();

        // 创建发布者：发布基地在机器人坐标系下的位置
        publisher_ = this->create_publisher<geometry_msgs::msg::PointStamped>("base_position", 10);

        // 初始化 tf2 缓冲区和监听器
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // 创建定时器，周期性计算并发布
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds((int)(1000.0 / rate)),
            std::bind(&BaseTargetPublisher::timer_callback, this));
    }

private:
    void timer_callback()
    {
        // 基地在 map 坐标系下的点
        geometry_msgs::msg::PointStamped base_in_map;
        base_in_map.header.frame_id = "map";
        base_in_map.header.stamp = this->now();
        base_in_map.point.x = base_x_;
        base_in_map.point.y = base_y_;
        base_in_map.point.z = base_z_;

        try
        {
            // 查找从 map 到 target_frame（通常是 base_link）的最新变换
            geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform(
                target_frame_, "map", tf2::TimePointZero);

            // 将点从 map 坐标系变换到 target_frame 坐标系
            geometry_msgs::msg::PointStamped base_in_target;
            tf2::doTransform(base_in_map, base_in_target, transform);

            // 发布结果
            publisher_->publish(base_in_target);
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "Could not transform: %s", ex.what());
        }
    }

    // 参数变量
    double base_x_, base_y_, base_z_;
    std::string target_frame_;

    // ROS 对象
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr publisher_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BaseTargetPublisher>());
    rclcpp::shutdown();
    return 0;
}