#include <chrono>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

using namespace std::chrono_literals;

class TFDiffCalculator: public rclcpp::Node {
public:
    TFDiffCalculator(): Node("tf_diff_calculator") {
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        publisher_ =
            this->create_publisher<geometry_msgs::msg::PointStamped>("/radar/target_position", 10);

        timer_ = this->create_wall_timer(
            100ms,
            std::bind(&TFDiffCalculator::calculate_and_publish, this)
        );
    }

private:
    void calculate_and_publish() {
        std::string source_frame = "odom";
        std::string target_frame = "enermy_base";

        try {
            // 使用 TransformStamped 接收变换
            geometry_msgs::msg::TransformStamped transform_stamped;
            // 使用 rclcpp::Time(0) 获取最新变换（兼容构造函数）
            transform_stamped =
                tf_buffer_->lookupTransform(target_frame, source_frame, rclcpp::Time(0));

            // 构造点消息，提取平移部分
            geometry_msgs::msg::PointStamped point_stamped;
            point_stamped.header.stamp = this->get_clock()->now();
            point_stamped.header.frame_id = target_frame; // 点位于目标坐标系
            point_stamped.point.x = transform_stamped.transform.translation.x;
            point_stamped.point.y = transform_stamped.transform.translation.y;
            point_stamped.point.z = transform_stamped.transform.translation.z;

            publisher_->publish(point_stamped);
        } catch (tf2::TransformException& ex) {
            RCLCPP_WARN(this->get_logger(), "Transform error: %s", ex.what());
        }
    }

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TFDiffCalculator>());
    rclcpp::shutdown();
    return 0;
}