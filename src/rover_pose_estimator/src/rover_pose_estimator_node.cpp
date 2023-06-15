#include "rclcpp/rclcpp.hpp"
#include "turtlesim/msg/pose.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include <functional>
#include <chrono>

using CurrentPositionMsg = turtlesim::msg::Pose;
using PositionEstimateMsg = geometry_msgs::msg::Pose;

using std::placeholders::_1;

class RoverPoseEstimator : public rclcpp::Node
{
public:
    RoverPoseEstimator();
private:
    rclcpp::Subscription<CurrentPositionMsg>::SharedPtr p_current_position_subscription_;
    rclcpp::Publisher<PositionEstimateMsg>::SharedPtr p_position_estimate_publisher_;
    rclcpp::TimerBase::SharedPtr p_timer_;

    PositionEstimateMsg p_current_position_estimate_;

    void on_new_position_reading_(CurrentPositionMsg::ConstSharedPtr);
    void on_timer_();
};

RoverPoseEstimator::RoverPoseEstimator()
:   Node("rover_pose_estimator")
{
    this->declare_parameter("position_input_topic", "position_input_topic");
    this->declare_parameter("position_output_topic", "position_output_topic");
    this->declare_parameter("broadcast_rate_ms", 1000);

    auto position_input_topic = this->get_parameter("position_input_topic").as_string();
    auto position_output_topic = this->get_parameter("position_output_topic").as_string();
    auto broadcast_rate_ms = this->get_parameter("broadcast_rate_ms").as_int();

    RCLCPP_INFO(this->get_logger(),
        "Reading position from '%s' "
        "Broadcasting estimate on '%s' "
        "With rate of %d ms.",
        position_input_topic.c_str(),
        position_output_topic.c_str(),
        broadcast_rate_ms
    );

    auto callback = std::bind(&RoverPoseEstimator::on_new_position_reading_, this, _1);
    p_current_position_subscription_ = this->create_subscription<CurrentPositionMsg>(position_input_topic, 10, callback);

    p_position_estimate_publisher_ = this->create_publisher<PositionEstimateMsg>(position_output_topic, 10);

    auto rate = std::chrono::milliseconds(broadcast_rate_ms);
    p_timer_ = this->create_wall_timer(rate, std::bind(&RoverPoseEstimator::on_timer_, this));

    p_current_position_estimate_ = PositionEstimateMsg();
    p_current_position_estimate_.position.x = 0;
    p_current_position_estimate_.position.y = 0;
}

void RoverPoseEstimator::on_new_position_reading_(CurrentPositionMsg::ConstSharedPtr msg)
{
    RCLCPP_DEBUG(this->get_logger(), "Received position update. <%03.3lf, %03.3lf>", msg->x, msg->y);
    
    p_current_position_estimate_.position.x = msg->x;
    p_current_position_estimate_.position.y = msg->y;
}

void RoverPoseEstimator::on_timer_()
{
    RCLCPP_DEBUG(this->get_logger(), "Broadcasting position estimate!");
    PositionEstimateMsg message = PositionEstimateMsg();
        
    message.position.x = p_current_position_estimate_.position.x;
    message.position.y = p_current_position_estimate_.position.y;

    p_position_estimate_publisher_->publish(message);
}

int main(int argc, const char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RoverPoseEstimator>());
    rclcpp::shutdown();
    return 0;
}