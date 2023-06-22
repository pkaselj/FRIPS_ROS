#include "rclcpp/rclcpp.hpp"
#include "turtlesim/msg/pose.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "rover_messages/msg/rover_heading.hpp"
#include "marvelmind_ros2_msgs/msg/hedge_position_angle.hpp"
#include <functional>
#include <chrono>

// IMPORTANT TODO: Add transforms between world, rover-world and rover systems
// IMPORTANT TODO: Check RAD vs DEG

using PositionMsg = marvelmind_ros2_msgs::msg::HedgePositionAngle;
using RelativeHeadingMsg = rover_messages::msg::RoverHeading;

using std::placeholders::_1;

class RoverPoseEstimator : public rclcpp::Node
{
public:
    RoverPoseEstimator();
private:
    rclcpp::Subscription<PositionMsg>::SharedPtr p_position_measurement_subscription_;
    rclcpp::Subscription<RelativeHeadingMsg>::SharedPtr p_heading_from_odometry_subscription_;
    rclcpp::Subscription<PositionMsg>::SharedPtr p_rover_world_origin_subscriber_;
    rclcpp::Publisher<PositionMsg>::SharedPtr p_position_estimate_publisher_;
    rclcpp::TimerBase::SharedPtr p_timer_;

    // Aposteriori (Relative to Rover WCS)
    PositionMsg current_position_estimate_;

    // Apriori from marvelmind (Relative to Rover WCS)
    PositionMsg current_position_reading_;
    // Apriori from odometry (position delta) (Relative to Rover-Local CS)
    PositionMsg current_position_delta_measurement_;

    // Rover World Coordinate System origin (Relative to Marvelmind WCS)
    PositionMsg rwcs_origin_relative_to_mwcs;

    void on_new_position_reading_(PositionMsg::ConstSharedPtr);
    void on_new_relative_heading_from_odometry_measurement_(RelativeHeadingMsg::ConstSharedPtr);
    void on_rover_world_origin_callback_(PositionMsg::ConstSharedPtr);
    void on_timer_();

    void do_update_estimate_with_marvelmind_reading_(const PositionMsg&);
    void do_update_estimate_with_odometry_measurement_(float x_m, float y_m);

    PositionMsg transform_mwcs_to_rwcs_position_(const PositionMsg& mwcs_position);
};

RoverPoseEstimator::RoverPoseEstimator()
:   Node("rover_pose_estimator")
{
    // TODO: QoS Parameters

    this->declare_parameter("position_input_topic", "position_input_topic");
    this->declare_parameter("position_output_topic", "position_output_topic");
    this->declare_parameter("broadcast_rate_ms", 1000);
    this->declare_parameter("relative_heading_input_topic", "/relative_heading_estimate_from_odometry");
    this->declare_parameter("rover_world_origin_topic", "rover_world_origin_topic");

    auto position_input_topic = this->get_parameter("position_input_topic").as_string();
    auto position_output_topic = this->get_parameter("position_output_topic").as_string();
    auto broadcast_rate_ms = this->get_parameter("broadcast_rate_ms").as_int();
    auto relative_heading_input_topic = this->get_parameter("relative_heading_input_topic").as_string();
    auto rover_world_origin_topic = this->get_parameter("rover_world_origin_topic").as_string();

    RCLCPP_INFO(this->get_logger(),
        "Reading position from '%s' and odometry from '%s' "
        "Broadcasting estimate on '%s' "
        "With rate of %d ms.",
        position_input_topic.c_str(), relative_heading_input_topic.c_str(),
        position_output_topic.c_str(),
        broadcast_rate_ms
    );

    auto callback = std::bind(&RoverPoseEstimator::on_new_position_reading_, this, _1);
    p_position_measurement_subscription_ = this->create_subscription<PositionMsg>(position_input_topic, 10, callback);

    p_position_estimate_publisher_ = this->create_publisher<PositionMsg>(position_output_topic, 10);

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

    auto rover_world_origin_callback = std::bind(&RoverPoseEstimator::on_rover_world_origin_callback_, this, _1);
    p_rover_world_origin_subscriber_ = this->create_subscription<PositionMsg>(
        rover_world_origin_topic,
        10,
        rover_world_origin_callback
    );

    current_position_estimate_ = PositionMsg();
    current_position_estimate_.x_m = 0;
    current_position_estimate_.y_m = 0;
    current_position_estimate_.angle = 0;

    current_position_reading_ = PositionMsg();
    current_position_reading_.x_m = 0;
    current_position_reading_.y_m = 0;
    current_position_reading_.angle = 0;

    current_position_delta_measurement_ = PositionMsg();
    current_position_delta_measurement_.x_m = 0;
    current_position_delta_measurement_.y_m = 0;
    current_position_delta_measurement_.angle = 0;

    rwcs_origin_relative_to_mwcs = PositionMsg();
    rwcs_origin_relative_to_mwcs.x_m = 0;
    rwcs_origin_relative_to_mwcs.y_m = 0;
    rwcs_origin_relative_to_mwcs.angle = 0;
}

void RoverPoseEstimator::on_new_position_reading_(PositionMsg::ConstSharedPtr p_mwcs_pos)
{
    RCLCPP_DEBUG(
        this->get_logger(),
        "Received position update. <x, y, phi> = <%03.3lf m, %03.3lf m, %03.3f DEG? >",
        p_mwcs_pos->x_m, p_mwcs_pos->y_m, p_mwcs_pos->angle
    );

    // Get RWCS coordinate from MWCS reading
    auto rwcs_pos = transform_mwcs_to_rwcs_position_(*p_mwcs_pos);

    // Save current RWCS position reading
    current_position_reading_ = rwcs_pos;

    // Update estimate
    do_update_estimate_with_marvelmind_reading_(rwcs_pos);
}

void RoverPoseEstimator::on_new_relative_heading_from_odometry_measurement_(RelativeHeadingMsg::ConstSharedPtr p_msg)
{
    RCLCPP_DEBUG(
        this->get_logger(),
        "Received relative heading estimate from odometry readings. <v, phi> = <%03.3f m/s, %03.3f DEG>",
        p_msg->speed_m_s, (360.0f / 6.28f) * p_msg->relative_direction_rad
    );

    current_position_delta_measurement_.x_m = p_msg->speed_m_s * cosf(p_msg->relative_direction_rad);
    current_position_delta_measurement_.y_m = p_msg->speed_m_s * sinf(p_msg->relative_direction_rad);
    
    // TODO angle measurement (transform relative heading to world heading / rover-world heading)
    current_position_delta_measurement_.angle = 0;
}

void RoverPoseEstimator::on_timer_()
{
    RCLCPP_DEBUG(
        this->get_logger(),
        "Broadcasting position estimate: <x, y, phi> = <%03.3lf m, %03.3lf m, %03.3f DEG? >",
        current_position_estimate_.x_m, current_position_estimate_.y_m, current_position_estimate_.angle    
    );

    p_position_estimate_publisher_->publish(current_position_estimate_);
}

void RoverPoseEstimator::do_update_estimate_with_marvelmind_reading_(const PositionMsg& rwcs_pos)
{
    // For now just take measurement without estimation
    current_position_estimate_ = rwcs_pos;
}

void RoverPoseEstimator::do_update_estimate_with_odometry_measurement_(float x_m, float y_m)
{
    // IMPORTANT TODO: Do nothing for now (KF, EKF)
    ;
}

PositionMsg RoverPoseEstimator::transform_mwcs_to_rwcs_position_(const PositionMsg &mwcs_position)
{
    auto rwcs_position = PositionMsg();
    
    auto delta_x = rwcs_origin_relative_to_mwcs.x_m;
    auto delta_y = rwcs_origin_relative_to_mwcs.y_m;
    auto delta_angle_deg = rwcs_origin_relative_to_mwcs.angle;
    auto delta_angle_rad = delta_angle_deg * (6.28f / 360.0f);

    // Derived from combined translation (by delta_x, delta_y) and rotation (delta_angle_deg) 
    // see 'https://nu-msr.github.io/navigation_site/lectures/rigid2d.html#outline-container-org9bbf411'
    // coordinate system 'i' corresponds to MWCS and 'j' to RWCS

    // x_j = (x_i - delta_x) * cos(theta) + (y_i - delta_y) * sin(theta)
    rwcs_position.x_m = (mwcs_position.x_m - delta_x) * cos(delta_angle_rad) + (mwcs_position.y_m - delta_y) * sin(delta_angle_rad);

    // y_j = (y_i - delta_y) * cos(theta) - (x_i - delta_x) * sin(theta)
    rwcs_position.y_m = (mwcs_position.y_m - delta_y) * cos(delta_angle_rad) - (mwcs_position.x_m - delta_x) * sin(delta_angle_rad);
    
    // TODO: Double check if angles add up (or subtract in this case)
    rwcs_position.angle = mwcs_position.angle - delta_angle_deg;

    return rwcs_position;
}

void RoverPoseEstimator::on_rover_world_origin_callback_(PositionMsg::ConstSharedPtr p_msg)
{
  RCLCPP_INFO(
    this->get_logger(),
    "Received request to relocate rover world origin to "
    "<x, y, angle> = <%03.3f m, %03.3f m, %03.3f DEG> "
    "relative to marvelmind world coordinate system",
    p_msg->x_m, p_msg->y_m, p_msg->angle
  );

  rwcs_origin_relative_to_mwcs = *p_msg;
}

int main(int argc, const char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RoverPoseEstimator>());
    rclcpp::shutdown();
    return 0;
}