#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <memory>
#include <chrono>

using namespace std::chrono_literals;

class TFDiffCalculator : public rclcpp::Node
{
public:
    TFDiffCalculator() : Node("tf_diff_calculator")
    {
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        publisher_ = this->create_publisher<geometry_msgs::msg::TransformStamped>("/radar/target_position", 10);

        // 0.1秒定时器，对应10Hz
        timer_ = this->create_wall_timer(100ms, std::bind(&TFDiffCalculator::calculate_and_publish, this));
    }

private:
    void calculate_and_publish()
    {
        std::string source_frame = "livox_frame";
        std::string target_frame = "enermy_base";

        try {
            geometry_msgs::msg::TransformStamped transform_stamped;
            transform_stamped = tf_buffer_->lookupTransform(target_frame, source_frame, tf2::TimePointZero);

            transform_stamped.header.stamp = this->get_clock()->now();
            transform_stamped.header.frame_id = source_frame;
            transform_stamped.child_frame_id = target_frame;

            publisher_->publish(transform_stamped);
            // 已移除所有日志输出
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "Transform error: %s", ex.what());
        }
    }

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TFDiffCalculator>());
    rclcpp::shutdown();
    return 0;
}