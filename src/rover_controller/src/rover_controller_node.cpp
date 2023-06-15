#include <cstdio>
#include "rclcpp/rclcpp.hpp"
#include "rover_messages/msg/rover_heading.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include <chrono>
#include <memory>
#include <functional>

#define EPSILON 0.001f

using CommandMsg = rover_messages::msg::RoverHeading;
using PositionMsg = geometry_msgs::msg::Pose;

using std::placeholders::_1;

class RoverController : public rclcpp::Node
{
  public:
  RoverController();

  private:
  rclcpp::Publisher<CommandMsg>::SharedPtr p_command_publisher_;
  rclcpp::TimerBase::SharedPtr p_timer_;
  rclcpp::Subscription<PositionMsg>::SharedPtr p_position_subscriber_;
  rclcpp::Subscription<PositionMsg>::SharedPtr p_setpoint_subscriber_;

  PositionMsg p_current_position_;
  PositionMsg p_setpoint_position_;

  double max_speed_magnitude_;

  void timer_callback_();
  void position_receive_callback_(PositionMsg::ConstSharedPtr);
  void setpoint_receive_callback_(PositionMsg::ConstSharedPtr);
  void publish_stop_command_();
  void publish_heading_command_();
};

RoverController::RoverController()
  : Node("rover_controller")
{
  RCLCPP_INFO(this->get_logger(), "START Ctor()");

  this->declare_parameter("command_topic", "rover_commands");
  this->declare_parameter("position_topic", "position_topic");
  this->declare_parameter("setpoint_topic", "setpoint_topic");
  this->declare_parameter("publish_rate_ms", 1000);
  this->declare_parameter("max_speed_magnitude", 1.0f);

  auto command_topic = this->get_parameter("command_topic").as_string();
  auto position_topic = this->get_parameter("position_topic").as_string();
  auto setpoint_topic = this->get_parameter("setpoint_topic").as_string();
  auto rate_ms = this->get_parameter("publish_rate_ms").as_int();
  max_speed_magnitude_ = this->get_parameter("max_speed_magnitude").as_double();

  RCLCPP_INFO(this->get_logger(),
    "Publishing commands to topic: "
    "'%s' every '%d' ms." 
    "Reading position from '%s'."
    "Max speed limited to: %03.3lf",
    command_topic.c_str(),
    rate_ms,
    position_topic.c_str(),
    max_speed_magnitude_
  );

  p_command_publisher_ = this->create_publisher<CommandMsg>(command_topic, 10);

  auto timer_callback = std::bind(&RoverController::timer_callback_, this);
  auto timer_rate = std::chrono::milliseconds(rate_ms);
  p_timer_ = this->create_wall_timer(timer_rate, timer_callback);

  auto position_callback = std::bind(&RoverController::position_receive_callback_, this, _1);
  p_position_subscriber_ = this->create_subscription<PositionMsg>(position_topic, 10, position_callback);

  auto setpoint_callback = std::bind(&RoverController::setpoint_receive_callback_, this, _1);
  p_setpoint_subscriber_ = this->create_subscription<PositionMsg>(setpoint_topic, 10, setpoint_callback);

  p_setpoint_position_ = PositionMsg();
  p_setpoint_position_.position.x = 5;
  p_setpoint_position_.position.y = 5;

  RCLCPP_INFO(this->get_logger(), "END Ctor()");
}

static bool is_target_at_destination_(double target_x, double target_y, double dest_x, double dest_y)
{
  return (fabs(dest_x - target_x) < EPSILON) && (fabs(dest_y - target_y) < EPSILON);
}

void RoverController::timer_callback_()
{
  if (
    is_target_at_destination_(
      p_current_position_.position.x,  /* TARGET.X */
      p_current_position_.position.y,  /* TARGET.Y */
      p_setpoint_position_.position.x,   /* DESTINATION.X */
      p_setpoint_position_.position.y    /* DESTINATION.Y */
      )
  )
  {
    publish_stop_command_();
    RCLCPP_DEBUG(this->get_logger(), "Target is at destination.");
    return; // Stay in place
  }
  
  publish_heading_command_();
}

void RoverController::position_receive_callback_(PositionMsg::ConstSharedPtr new_position)
{
  RCLCPP_DEBUG(this->get_logger(), "Received position update.");

  p_current_position_.position.x = new_position->position.x;
  p_current_position_.position.y = new_position->position.y;
}

void RoverController::setpoint_receive_callback_(PositionMsg::ConstSharedPtr new_setpoint)
{
  RCLCPP_DEBUG(this->get_logger(), "Received setpoint update.");

  p_setpoint_position_.position.x = new_setpoint->position.x;
  p_setpoint_position_.position.y = new_setpoint->position.y;
}

void RoverController::publish_stop_command_()
{
  auto message = CommandMsg();

  message.durations_ms = 0;
  message.relative_direction_rad = 0;
  message.speed_rps = 0;

  p_command_publisher_->publish(message);
}

void RoverController::publish_heading_command_()
{
  auto message = CommandMsg();

  auto delta_x = p_setpoint_position_.position.x - p_current_position_.position.x;
  auto delta_y = p_setpoint_position_.position.y - p_current_position_.position.y;

  auto distance = sqrt(delta_x * delta_x + delta_y * delta_y);

  auto angle_of_attack = atan2(delta_y, delta_x);

  message.relative_direction_rad = angle_of_attack;
  message.speed_rps = max_speed_magnitude_;
  message.durations_ms = 1000 * (distance / message.speed_rps);

  this->p_command_publisher_->publish(message);

  RCLCPP_DEBUG(
    this->get_logger(),
    "Setpoint <%03.3lf, %03.3lf> "
    "Target <%03.3lf, %03.3lf> "
    "Rover speed set to <%03.3lf RPS, %03.3lf rad> for %04d ms",
    p_setpoint_position_.position.x, p_setpoint_position_.position.y,
    p_current_position_.position.x, p_current_position_.position.y,
    message.speed_rps, message.relative_direction_rad, message.durations_ms
  );
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RoverController>());
  rclcpp::shutdown();

  return 0;
}
