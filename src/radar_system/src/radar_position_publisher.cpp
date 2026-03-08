#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <cmath>
#include <chrono>

using namespace std::chrono_literals;

class RadarPositionPublisher : public rclcpp::Node
{
public:
    RadarPositionPublisher() : Node("radar_position_publisher")
    {
        // 声明参数
        this->declare_parameter<double>("radius", 10.0);        // 圆周半径 (米)
        this->declare_parameter<double>("angular_velocity", 0.2); // 角速度 (rad/s)
        this->declare_parameter<double>("center_x", 0.0);       // 圆心x坐标
        this->declare_parameter<double>("center_y", 0.0);       // 圆心y坐标
        this->declare_parameter<double>("center_z", 2.0);       // 固定高度z
        this->declare_parameter<std::string>("frame_id", "map"); // 坐标系
        this->declare_parameter<double>("publish_rate", 10.0);  // 发布频率 (Hz)

        // 获取参数
        radius_ = this->get_parameter("radius").as_double();
        omega_ = this->get_parameter("angular_velocity").as_double();
        cx_ = this->get_parameter("center_x").as_double();
        cy_ = this->get_parameter("center_y").as_double();
        cz_ = this->get_parameter("center_z").as_double();
        frame_id_ = this->get_parameter("frame_id").as_string();
        double rate_hz = this->get_parameter("publish_rate").as_double();

        // 创建发布者
        publisher_ = this->create_publisher<geometry_msgs::msg::PointStamped>("/radar/position", 10);

        // 定时器
        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(1.0 / rate_hz),
            std::bind(&RadarPositionPublisher::timer_callback, this));

        // 记录起始时间用于计算角度
        start_time_ = this->now();

        RCLCPP_INFO(this->get_logger(), "雷达位置发布节点已启动，发布频率 %.1f Hz，坐标系 %s",
                    rate_hz, frame_id_.c_str());
        RCLCPP_INFO(this->get_logger(), "运动参数: 半径 %.2f m, 角速度 %.2f rad/s, 中心 (%.2f, %.2f, %.2f)",
                    radius_, omega_, cx_, cy_, cz_);
    }

private:
    void timer_callback()
    {
        // 计算当前时间相对于起始时间的角度
        rclcpp::Time now = this->now();
        double dt = (now - start_time_).seconds();
        double theta = omega_ * dt;

        // 圆周运动坐标
        double x = cx_ + radius_ * std::cos(theta);
        double y = cy_ + radius_ * std::sin(theta);  // 由于y轴向左为正，sin(theta)仍保持几何关系
        double z = cz_;

        // 构建消息
        geometry_msgs::msg::PointStamped msg;
        msg.header.stamp = now;
        msg.header.frame_id = frame_id_;
        msg.point.x = x;
        msg.point.y = y;
        msg.point.z = z;

        publisher_->publish(msg);

        // 调试输出（可降低频率或改为DEBUG级别）
        RCLCPP_DEBUG(this->get_logger(), "发布雷达位置: (%.2f, %.2f, %.2f)", x, y, z);
    }

    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Time start_time_;

    // 参数
    double radius_;
    double omega_;
    double cx_, cy_, cz_;
    std::string frame_id_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RadarPositionPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}