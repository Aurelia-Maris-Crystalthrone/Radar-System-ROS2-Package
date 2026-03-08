#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <chrono>

using namespace std::chrono_literals;

class LidarAgainst : public rclcpp::Node
{
public:
    LidarAgainst() : Node("base_point_publisher")
    {
        publisher_ = this->create_publisher<geometry_msgs::msg::PointStamped>("/radar/position", 10);

        timer_ = this->create_wall_timer(
            1s, std::bind(&LidarAgainst::timer_callback, this));

        base_x_ = 0.0;   
        base_y_ = 0.0;   
        base_z_ = 0.0;   
        frame_id_ = "map";
    }

private:
    void timer_callback()
    {
        auto msg = geometry_msgs::msg::PointStamped();
        // 填充头部信息
        msg.header.stamp = this->get_clock()->now();
        msg.header.frame_id = frame_id_;
        // 填充点坐标
        msg.point.x = base_x_;
        msg.point.y = base_y_;
        msg.point.z = base_z_;

        publisher_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "发布基地坐标: (%.2f, %.2f, %.2f) 在坐标系 %s 中",
                    base_x_, base_y_, base_z_, frame_id_.c_str());
    }

    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    double base_x_, base_y_, base_z_;
    std::string frame_id_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LidarAgainst>());
    rclcpp::shutdown();
    return 0;
}