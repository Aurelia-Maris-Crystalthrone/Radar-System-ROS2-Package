#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/crop_box.h>

class PointCloudFilterNode : public rclcpp::Node {
public:
    PointCloudFilterNode() : Node("pointcloud_filter_node") {
        // 订阅者
        auto qos_profile = rclcpp::SensorDataQoS();
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/cloud_registered_body", 
            qos_profile,
            std::bind(&PointCloudFilterNode::cloud_callback, this, std::placeholders::_1));

        // 发布者通常建议也保持一致，或者使用默认
        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/pointclouds_self_filter", qos_profile);
    }

private:
    void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        // 1. 将 ROS 消息转换为 PCL 点云
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud);

        if (cloud->empty()) return;

        // 2. 配置 CropBox 过滤器
        pcl::CropBox<pcl::PointXYZ> crop_box;
        crop_box.setInputCloud(cloud);

        // 设置过滤空间边界 (Min: x, y, z; Max: x, y, z)
        // 假设过滤掉车体范围：x[-1, 1], y[-0.5, 0.5], z[-0.5, 1.0]
        Eigen::Vector4f min_pt(-0.1625f, -0.0f, -0.3f, 1.0f);
        Eigen::Vector4f max_pt(0.1625f, 0.325f, 0.4f, 1.0f);
        crop_box.setMin(min_pt);
        crop_box.setMax(max_pt);

        // 设置为 True 表示“过滤掉空间内”，为 False 表示“只保留空间内”
        crop_box.setNegative(true); 

        // 3. 执行过滤
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
        crop_box.filter(*cloud_filtered);

        // 4. 转回 ROS 消息并发布
        sensor_msgs::msg::PointCloud2 output_msg;
        pcl::toROSMsg(*cloud_filtered, output_msg);
        output_msg.header = msg->header; // 保持原始时间戳和坐标系
        output_msg.header.frame_id = "livox_frame";
        publisher_->publish(output_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointCloudFilterNode>());
    rclcpp::shutdown();
    return 0;
}