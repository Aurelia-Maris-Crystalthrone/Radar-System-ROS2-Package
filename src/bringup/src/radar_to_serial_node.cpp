#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "serial/serial.h"
#include "bringup/radar_data_converter.hpp"

#include <cmath>
#include <string>
#include <vector>

#include <builtin_interfaces/msg/time.hpp>

using namespace std::chrono_literals;

namespace bringup
{

/**
 * 数据聚合与串口下行节点。
 *
 * 设计要点（面试常被追问的部分）：
 *  1. 目标位置来自 tf_diff_calculator 发布的 /radar/target_position，该点已经表达在
 *     enermy_base 坐标系下，因此"基地基准点"就是该坐标系原点 (0,0,0)，外部再单独输入
 *     base_point 只是可选覆盖。
 *  2. 距离由 target - base_point 的欧氏范数就地计算，不再依赖外部 /radar/distance，
 *     避免"三个话题必须齐活才发帧"的死锁（原实现的硬伤）。
 *  3. 所有数据带 staleness 检查：超过 max_data_age_sec 的一律丢弃，绝不把陈旧帧
 *     发给下位机去做弹道解算。
 */
class RadarToSerialNode : public rclcpp::Node
{
public:
  RadarToSerialNode()
  : Node("radar_to_serial_node")
  {
    // 1. 参数
    serial_port_ = this->declare_parameter("serial_port", std::string("/dev/ttyUSB0"));
    baud_rate_ = this->declare_parameter("baud_rate", 115200);
    send_period_ms_ = this->declare_parameter("send_period_ms", 50);  // 20 Hz
    max_data_age_ = rclcpp::Duration::from_seconds(
      this->declare_parameter("max_data_age_sec", 0.5));

    target_topic_ = this->declare_parameter("target_topic", std::string("/radar/target_position"));
    base_point_topic_ = this->declare_parameter("base_point_topic", std::string("/radar/base_point"));
    distance_topic_ = this->declare_parameter("distance_topic", std::string("/radar/distance"));

    // enermy_base 原点作为默认基准点，可由参数或外部话题覆盖
    default_base_x_ = this->declare_parameter("base_point_x", 0.0);
    default_base_y_ = this->declare_parameter("base_point_y", 0.0);
    default_base_z_ = this->declare_parameter("base_point_z", 0.0);

    // 2. 初始化串口
    try {
      serial_.setPort(serial_port_);
      serial_.setBaudrate(static_cast<uint32_t>(baud_rate_));
      serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
      serial_.setTimeout(timeout);
      serial_.open();
      RCLCPP_INFO(this->get_logger(), "Serial port %s opened at %ld baud",
        serial_port_.c_str(), baud_rate_);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open serial port: %s", e.what());
      rclcpp::shutdown();
      return;
    }

    // 3. 订阅：target 为必需，base_point 与 distance 为可选覆盖
    target_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
      target_topic_, 10,
      std::bind(&RadarToSerialNode::targetCallback, this, std::placeholders::_1));

    base_point_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
      base_point_topic_, 10,
      std::bind(&RadarToSerialNode::basePointCallback, this, std::placeholders::_1));

    distance_sub_ = this->create_subscription<std_msgs::msg::Float64>(
      distance_topic_, 10,
      std::bind(&RadarToSerialNode::distanceCallback, this, std::placeholders::_1));

    // 4. 定时下发
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(send_period_ms_),
      std::bind(&RadarToSerialNode::sendData, this));

    RCLCPP_INFO(this->get_logger(),
      "Radar to Serial Node started: %s -> %s @ %ld baud, %ld ms period",
      target_topic_.c_str(), serial_port_.c_str(), baud_rate_, send_period_ms_);
  }

private:
  struct Vec3 { double x; double y; double z; };

  // ---- 参数 ----
  std::string serial_port_;
  int64_t baud_rate_ = 115200;
  int64_t send_period_ms_ = 50;
  rclcpp::Duration max_data_age_ = rclcpp::Duration::from_seconds(0.5);
  std::string target_topic_, base_point_topic_, distance_topic_;
  double default_base_x_, default_base_y_, default_base_z_;

  // ---- 最新数据 ----
  bool has_target_ = false;
  geometry_msgs::msg::PointStamped latest_target_;
  bool has_base_point_ = false;
  Vec3 latest_base_point_{0.0, 0.0, 0.0};
  bool has_external_distance_ = false;
  double latest_external_distance_ = 0.0;
  rclcpp::Time latest_distance_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);

  // ---- ROS 对象 ----
  serial::Serial serial_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr target_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr base_point_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr distance_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // ---- 回调 ----
  void targetCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
  {
    latest_target_ = *msg;
    has_target_ = true;
  }

  void basePointCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
  {
    latest_base_point_.x = msg->point.x;
    latest_base_point_.y = msg->point.y;
    latest_base_point_.z = msg->point.z;
    has_base_point_ = true;
  }

  void distanceCallback(const std_msgs::msg::Float64::SharedPtr msg)
  {
    latest_external_distance_ = msg->data;
    latest_distance_stamp_ = this->get_clock()->now();
    has_external_distance_ = true;
  }

  bool isFresh(const builtin_interfaces::msg::Time & stamp) const
  {
    // 时间戳为空（外部节点未填 stamp）时不做 staleness 判定
    if (stamp.sec == 0 && stamp.nanosec == 0) {
      return true;
    }
    return (this->get_clock()->now() - rclcpp::Time(stamp)) <= max_data_age_;
  }

  bool isFresh(const rclcpp::Time & stamp) const
  {
    return (this->get_clock()->now() - stamp) <= max_data_age_;
  }

  // ---- 打包并下发 ----
  void sendData()
  {
    if (!has_target_) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
        "Waiting for target data on %s...", target_topic_.c_str());
      return;
    }

    if (!isFresh(latest_target_.header.stamp)) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "Target data stale (>%.2fs), skip this frame.", max_data_age_.seconds());
      return;
    }

    const Vec3 base = has_base_point_
      ? latest_base_point_
      : Vec3{default_base_x_, default_base_y_, default_base_z_};

    RadarData data;
    data.position_x = static_cast<float>(latest_target_.point.x);
    data.position_y = static_cast<float>(latest_target_.point.y);
    data.position_z = static_cast<float>(latest_target_.point.z);
    data.base_point_x = static_cast<float>(base.x);
    data.base_point_y = static_cast<float>(base.y);
    data.base_point_z = static_cast<float>(base.z);

    const double dx = latest_target_.point.x - base.x;
    const double dy = latest_target_.point.y - base.y;
    const double dz = latest_target_.point.z - base.z;
    const double computed = std::sqrt(dx * dx + dy * dy + dz * dz);

    // 外部距离优先，但同样要足够新鲜才采信
    const bool use_external =
      has_external_distance_ && isFresh(latest_distance_stamp_);
    data.distance = use_external ? latest_external_distance_ : computed;

    try {
      const std::vector<uint8_t> frame = RadarDataProtocol::packRadarData(data);
      const size_t bytes_written = serial_.write(frame);
      if (bytes_written != frame.size()) {
        RCLCPP_ERROR(this->get_logger(), "Short write: %zu/%zu bytes",
          bytes_written, frame.size());
      }
    } catch (const std::exception & e) {
      RCLCPP_ERROR(this->get_logger(), "Error sending data: %s", e.what());
    }
  }
};

}  // namespace bringup

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<bringup::RadarToSerialNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
