// TF 差值节点：以固定频率查询两个坐标系之间的变换，把平移部分作为 PointStamped 发出去。
//
// 默认 source_frame="odometry"、target_frame="enermy_base"，即把 odom 原点换算到
// 敌方基地坐标系下表达，交给下游 radar_to_serial_node 走串口给下位机。
//
// 帧名与频率改为 ROS 参数，不要再改源码重编——原先写死 `"odom"` 时，TF 树里真正的
// 里程计帧叫 `odometry`，两者不是同一个东西。
#include <chrono>
#include <string>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/time.h>

using namespace std::chrono_literals;

class TFDiffCalculator : public rclcpp::Node
{
public:
  TFDiffCalculator()
  : Node("tf_diff_calculator")
  {
    source_frame_ = this->declare_parameter("source_frame", std::string("odometry"));
    target_frame_ = this->declare_parameter("target_frame", std::string("enermy_base"));
    output_topic_ = this->declare_parameter(
      "output_topic", std::string("/radar/target_position"));
    const int64_t period_ms = this->declare_parameter("publish_period_ms", 100);  // 10 Hz
    const double timeout_sec = this->declare_parameter("tf_timeout_sec", 0.5);
    tf_timeout_ = tf2::durationFromSec(timeout_sec);

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    publisher_ = this->create_publisher<geometry_msgs::msg::PointStamped>(
      output_topic_, 10);

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(period_ms),
      std::bind(&TFDiffCalculator::calculate_and_publish, this));

    RCLCPP_INFO(this->get_logger(),
      "TF diff: %s -> %s, publish on %s every %ld ms",
      source_frame_.c_str(), target_frame_.c_str(), output_topic_.c_str(), period_ms);
  }

private:
  void calculate_and_publish()
  {
    try {
      // source 原点在 target 坐标系下的表达
      const geometry_msgs::msg::TransformStamped transform_stamped =
        tf_buffer_->lookupTransform(target_frame_, source_frame_, tf2::TimePointZero,
          tf_timeout_);

      geometry_msgs::msg::PointStamped point_stamped;
      point_stamped.header.stamp = this->get_clock()->now();
      point_stamped.header.frame_id = target_frame_;
      point_stamped.point.x = transform_stamped.transform.translation.x;
      point_stamped.point.y = transform_stamped.transform.translation.y;
      point_stamped.point.z = transform_stamped.transform.translation.z;

      publisher_->publish(point_stamped);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
        "Transform %s -> %s failed: %s",
        source_frame_.c_str(), target_frame_.c_str(), ex.what());
    }
  }

  std::string source_frame_, target_frame_, output_topic_;
  tf2::Duration tf_timeout_ = tf2::durationFromSec(0.5);
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TFDiffCalculator>());
  rclcpp::shutdown();
  return 0;
}
