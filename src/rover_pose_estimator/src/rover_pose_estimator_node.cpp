#include "rclcpp/rclcpp.hpp"
#include "turtlesim/msg/pose.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "rover_messages/msg/rover_heading.hpp"
#include "marvelmind_ros2_msgs/msg/hedge_position.hpp"
#include <functional>
#include <chrono>

using CurrentPositionMsg = marvelmind_ros2_msgs::msg::HedgePosition;
using PositionEstimateMsg = geometry_msgs::msg::Pose;
using RelativeHeadingMsg = rover_messages::msg::RoverHeading;

using std::placeholders::_1;

class RoverPoseEstimator : public rclcpp::Node
{
public:
    RoverPoseEstimator();
private:
    rclcpp::Subscription<CurrentPositionMsg>::SharedPtr p_position_measurement_subscription_;
    rclcpp::Subscription<RelativeHeadingMsg>::SharedPtr p_heading_from_odometry_subscription_;
    rclcpp::Publisher<PositionEstimateMsg>::SharedPtr p_position_estimate_publisher_;
    rclcpp::TimerBase::SharedPtr p_timer_;

    PositionEstimateMsg p_current_position_estimate_;

    void on_new_position_reading_(CurrentPositionMsg::ConstSharedPtr);
    void on_new_relative_heading_from_odometry_measurement_(RelativeHeadingMsg::ConstSharedPtr);
    void on_timer_();

    void do_update_estimate_with_marvelmind_reading_(float x_m, float y_m);
    void do_update_estimate_with_odometry_measurement_(float x_m, float y_m);
};

RoverPoseEstimator::RoverPoseEstimator()
:   Node("rover_pose_estimator")
{
    // TODO: QoS Parameters

    this->declare_parameter("position_input_topic", "position_input_topic");
    this->declare_parameter("position_output_topic", "position_output_topic");
    this->declare_parameter("broadcast_rate_ms", 1000);
    this->declare_parameter("relative_heading_input_topic", "/relative_heading_estimate_from_odometry");

    auto position_input_topic = this->get_parameter("position_input_topic").as_string();
    auto position_output_topic = this->get_parameter("position_output_topic").as_string();
    auto broadcast_rate_ms = this->get_parameter("broadcast_rate_ms").as_int();
    auto relative_heading_input_topic = this->get_parameter("relative_heading_input_topic").as_string();

    RCLCPP_INFO(this->get_logger(),
        "Reading position from '%s' and odometry from '%s' "
        "Broadcasting estimate on '%s' "
        "With rate of %d ms.",
        position_input_topic.c_str(), relative_heading_input_topic.c_str(),
        position_output_topic.c_str(),
        broadcast_rate_ms
    );

    auto callback = std::bind(&RoverPoseEstimator::on_new_position_reading_, this, _1);
    p_position_measurement_subscription_ = this->create_subscription<CurrentPositionMsg>(position_input_topic, 10, callback);

    p_position_estimate_publisher_ = this->create_publisher<PositionEstimateMsg>(position_output_topic, 10);

    auto rate = std::chrono::milliseconds(broadcast_rate_ms);
    p_timer_ = this->create_wall_timer(rate, std::bind(&RoverPoseEstimator::on_timer_, this));

    auto relative_heading_callback = std::bind(
        &RoverPoseEstimator::on_new_relative_heading_from_odometry_measurement_,
        this,
        _1
    );
    p_heading_from_odometry_subscription_ = this->create_subscription<RelativeHeadingMsg>(
        relative_heading_input_topic,
        10,
        relative_heading_callback
    );

    p_current_position_estimate_ = PositionEstimateMsg();
    p_current_position_estimate_.position.x = 0;
    p_current_position_estimate_.position.y = 0;
}

void RoverPoseEstimator::on_new_position_reading_(CurrentPositionMsg::ConstSharedPtr msg)
{
    RCLCPP_DEBUG(this->get_logger(), "Received position update. <%03.3lf m, %03.3lf m>", msg->x_m, msg->y_m);
    
    p_current_position_estimate_.position.x = msg->x_m;
    p_current_position_estimate_.position.y = msg->y_m;
}

void RoverPoseEstimator::on_new_relative_heading_from_odometry_measurement_(RelativeHeadingMsg::ConstSharedPtr p_msg)
{
    RCLCPP_DEBUG(
        this->get_logger(),
        "Received relative heading estimate from odometry readings. <v, phi> = <%03.3f m/s, %03.3f DEG>",
        p_msg->speed_m_s, (360.0f / 6.28f) * p_msg->relative_direction_rad
    );

    // TODO: ---
}

void RoverPoseEstimator::on_timer_()
{
    RCLCPP_DEBUG(this->get_logger(), "Broadcasting position estimate!");
    PositionEstimateMsg message = PositionEstimateMsg();
        
    message.position.x = p_current_position_estimate_.position.x;
    message.position.y = p_current_position_estimate_.position.y;

    p_position_estimate_publisher_->publish(message);
}

void RoverPoseEstimator::do_update_estimate_with_marvelmind_reading_(float x_m, float y_m)
{
    // For now just take measurement without estimation 
    p_current_position_estimate_.position.x = x_m;
    p_current_position_estimate_.position.y = y_m;
}

void RoverPoseEstimator::do_update_estimate_with_odometry_measurement_(float x_m, float y_m)
{
    // TODO: Do nothing for now
    ;
}

int main(int argc, const char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RoverPoseEstimator>());
    rclcpp::shutdown();
    return 0;
}