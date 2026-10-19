// 自身点云滤除节点。
//
// 用一个立方体 Box (PCL CropBox) 抠掉属于机器人本体的点云，防止在导航时把自身
// 结构误判成障碍物。
//
// 过滤范围、输入/输出话题全部做成 ROS 参数，调参不需要重新编译——原实现把六个
// 边界值硬编码在源文件里，现场要改一次就得编一次。
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/crop_box.h>
#include <Eigen/Core>

class PointCloudFilterNode : public rclcpp::Node
{
public:
  PointCloudFilterNode()
  : Node("pointcloud_filter_node")
  {
    // ---- 参数声明 ----
    input_topic_ = this->declare_parameter(
      "input_topic", std::string("/cloud_registered"));   // Point-LIO 默认输出
    output_topic_ = this->declare_parameter(
      "output_topic", std::string("/pointclouds_self_filter"));
    output_frame_ = this->declare_parameter("output_frame", std::string(""));

    crop_min_x_ = this->declare_parameter("crop_min_x", -0.1625);
    crop_min_y_ = this->declare_parameter("crop_min_y", 0.0);
    crop_min_z_ = this->declare_parameter("crop_min_z", -0.3);
    crop_max_x_ = this->declare_parameter("crop_max_x", 0.1625);
    crop_max_y_ = this->declare_parameter("crop_max_y", 0.325);
    crop_max_z_ = this->declare_parameter("crop_max_z", 0.4);

    // 点云数据量大且允许丢帧，用 SensorDataQoS（best_effort + keep_last）
    const auto qos_profile = rclcpp::SensorDataQoS();

    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_, qos_profile,
      std::bind(&PointCloudFilterNode::cloud_callback, this, std::placeholders::_1));

    publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      output_topic_, qos_profile);

    RCLCPP_INFO(this->get_logger(),
      "Self filter: %s -> %s, crop box [%.4f, %.4f, %.4f] ~ [%.4f, %.4f, %.4f]",
      input_topic_.c_str(), output_topic_.c_str(),
      crop_min_x_, crop_min_y_, crop_min_z_,
      crop_max_x_, crop_max_y_, crop_max_z_);
  }

private:
  void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *cloud);

    if (cloud->empty()) {
      return;
    }

    pcl::CropBox<pcl::PointXYZ> crop_box;
    crop_box.setInputCloud(cloud);
    // 第 4 位必须是 1，CropBox 内部使用齐次坐标
    crop_box.setMin(Eigen::Vector4f(
      static_cast<float>(crop_min_x_), static_cast<float>(crop_min_y_),
      static_cast<float>(crop_min_z_), 1.0f));
    crop_box.setMax(Eigen::Vector4f(
      static_cast<float>(crop_max_x_), static_cast<float>(crop_max_y_),
      static_cast<float>(crop_max_z_), 1.0f));
    // true = 抠掉盒子内部的点
    crop_box.setNegative(true);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    crop_box.filter(*cloud_filtered);

    sensor_msgs::msg::PointCloud2 output_msg;
    pcl::toROSMsg(*cloud_filtered, output_msg);
    output_msg.header = msg->header;
    // 未显式指定输出帧时沿用输入帧，避免把 camera_init 的点强行标成 livox_frame
    if (!output_frame_.empty()) {
      output_msg.header.frame_id = output_frame_;
    }
    publisher_->publish(output_msg);
  }

  // ---- 参数 ----
  std::string input_topic_, output_topic_, output_frame_;
  double crop_min_x_, crop_min_y_, crop_min_z_;
  double crop_max_x_, crop_max_y_, crop_max_z_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointCloudFilterNode>());
  rclcpp::shutdown();
  return 0;
}
