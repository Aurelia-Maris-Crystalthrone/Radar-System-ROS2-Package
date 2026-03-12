#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

class OdomToTf : public rclcpp::Node
{
public:
    OdomToTf() : Node("odom_to_tf")
    {
        // 声明参数，允许运行时调整话题名和坐标系
        this->declare_parameter<std::string>("odom_topic", "/Odometry");
        this->declare_parameter<std::string>("parent_frame", "aft_mapped");
        this->declare_parameter<std::string>("child_frame", "livox_frame");

        // 获取参数
        std::string odom_topic = this->get_parameter("odom_topic").as_string();
        parent_frame_ = this->get_parameter("parent_frame").as_string();
        child_frame_ = this->get_parameter("child_frame").as_string();

        // 创建订阅者，订阅里程计话题
        subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            odom_topic, 10, std::bind(&OdomToTf::odom_callback, this, std::placeholders::_1));

        // 初始化TF广播器
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        RCLCPP_INFO(this->get_logger(), "OdomToTf node started: listening to '%s'", odom_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "Publishing TF: '%s' -> '%s'", parent_frame_.c_str(), child_frame_.c_str());
    }

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        geometry_msgs::msg::TransformStamped transform;

        // 设置头部：使用里程计的时间戳，确保同步
        transform.header.stamp = msg->header.stamp;
        transform.header.frame_id = parent_frame_;
        transform.child_frame_id = child_frame_;

        // 从里程计位姿中提取平移和旋转
        transform.transform.translation.x = msg->pose.pose.position.x;
        transform.transform.translation.y = msg->pose.pose.position.y;
        transform.transform.translation.z = msg->pose.pose.position.z;
        transform.transform.rotation = msg->pose.pose.orientation;

        // 广播TF
        tf_broadcaster_->sendTransform(transform);
    }

    std::string parent_frame_;
    std::string child_frame_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdomToTf>());
    rclcpp::shutdown();
    return 0;
}