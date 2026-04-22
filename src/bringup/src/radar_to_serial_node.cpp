#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"  
#include "serial/serial.h"                      
#include "bringup/radar_data_converter.hpp"

using namespace std::chrono_literals;

namespace bringup
{

class RadarToSerialNode : public rclcpp::Node
{
public:
  RadarToSerialNode()
  : Node("radar_to_serial_node")
  {
    // 1. 从参数服务器获取配置
    this->declare_parameter("serial_port", "/dev/ttyUSB0");
    this->declare_parameter("baud_rate", 115200);

    std::string port = this->get_parameter("serial_port").as_string();
    int baud = this->get_parameter("baud_rate").as_int();

    // 2. 初始化串口
    try {
      serial_.setPort(port);
      serial_.setBaudrate(baud);
      serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
      serial_.setTimeout(timeout);
      serial_.open();
      RCLCPP_INFO(this->get_logger(), "Serial port %s opened successfully", port.c_str());
    } catch (const std::exception & e) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open serial port: %s", e.what());
      rclcpp::shutdown();
      return;
    }

    // 3. 创建订阅者
    distance_sub_ = this->create_subscription<std_msgs::msg::Float64>(
      "/radar/distance", 10,
      std::bind(&RadarToSerialNode::distanceCallback, this, std::placeholders::_1));

    radar_pos_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
      "/radar/position", 10,
      std::bind(&RadarToSerialNode::radarPositionCallback, this, std::placeholders::_1));

    base_point_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
      "/radar/base_point", 10,
      std::bind(&RadarToSerialNode::basePointCallback, this, std::placeholders::_1));

    // 4. 创建定时器，定期打包并发送数据 (20Hz)
    timer_ = this->create_wall_timer(
      50ms,
      std::bind(&RadarToSerialNode::sendData, this));

    RCLCPP_INFO(this->get_logger(), "Radar to Serial Node has been started.");
  }

private:
  // 存储最新的雷达数据
  RadarData latest_data_;
  bool distance_received_ = false;
  bool radar_pos_received_ = false;
  bool base_point_received_ = false;

  // 串口对象
  serial::Serial serial_;

  // 订阅者
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr distance_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr radar_pos_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr base_point_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // 回调函数
  void distanceCallback(const std_msgs::msg::Float64::SharedPtr msg)
  {
    latest_data_.distance = msg->data;
    distance_received_ = true;
  }

  void radarPositionCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
  {
    latest_data_.position_x = msg->point.x;
    latest_data_.position_y = msg->point.y;
    latest_data_.position_z = msg->point.z;
    radar_pos_received_ = true;
  }

  void basePointCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
  {
    latest_data_.base_point_x = msg->point.x;
    latest_data_.base_point_y = msg->point.y;
    latest_data_.base_point_z = msg->point.z;
    base_point_received_ = true;
  }

  // 打包并发送数据
  void sendData()
  {
    // 检查是否已接收到所有必要数据
    if (!distance_received_ || !radar_pos_received_ || !base_point_received_) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
        "Waiting for all radar data...");
      return;
    }

    try {
      // 将雷达数据打包为 uint8 帧
      std::vector<uint8_t> frame = RadarDataProtocol::packRadarData(latest_data_);

      // 通过串口发送
      size_t bytes_written = serial_.write(frame);
      if (bytes_written == frame.size()) {
        RCLCPP_DEBUG(this->get_logger(), "Sent radar data frame (%zu bytes)", bytes_written);
      } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to send complete frame");
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