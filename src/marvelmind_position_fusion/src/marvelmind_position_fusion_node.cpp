#include <cstdio>
#include <functional>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "marvelmind_ros2_msgs/msg/hedge_position_angle.hpp"

using std::placeholders::_1;

using PositionMsg = marvelmind_ros2_msgs::msg::HedgePositionAngle;

class MarvelmindPositionFusionNode : public rclcpp::Node
{
public:
  MarvelmindPositionFusionNode();

private:
  rclcpp::Subscription<PositionMsg>::SharedPtr p_left_beacon_position_subscription_;
  rclcpp::Subscription<PositionMsg>::SharedPtr p_right_beacon_position_subscription_;
  rclcpp::Publisher<PositionMsg>::SharedPtr p_fused_position_publisher_;
  rclcpp::TimerBase::SharedPtr p_fused_position_broadcast_timer_;

  PositionMsg fused_position_;
  float fusion_alpha_;

  float center_point_distance_from_left_node_percentage_;
  float pair_distance_m_;
  bool f_are_beacon_paired_;

  int left_beacon_address;
  int right_beacon_address;

  int position_buffer_size_;

  // TODO Send status update when buffer is full
  std::vector<PositionMsg> vec_left_beacon_buffer_;
  std::vector<PositionMsg> vec_right_beacon_buffer_;
  PositionMsg left_beacon_filtered_position_;
  PositionMsg right_beacon_filtered_position_;
  std::vector<PositionMsg>::iterator left_beacon_position_insert_iterator;
  std::vector<PositionMsg>::iterator right_beacon_position_insert_iterator;

  void on_left_position_receive_(PositionMsg::ConstSharedPtr p_msg);
  void on_right_position_receive_(PositionMsg::ConstSharedPtr p_msg);

  void feed_new_center_position_measurement_to_filter_(PositionMsg::ConstSharedPtr p_msg);

  void on_position_broadcast_timer_trigger_();
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin( std::make_shared<MarvelmindPositionFusionNode>() );
  rclcpp::shutdown();
  return 0;
}

MarvelmindPositionFusionNode::MarvelmindPositionFusionNode()
: rclcpp::Node("marvelmind_position_fusion_node")
{
  auto default_position_buffer_size = 10; 

  this->declare_parameter("position_measurement_left_topic", "/position_measurement_left");
  this->declare_parameter("position_measurement_left_qos", 10);
  this->declare_parameter("position_measurement_right_topic", "/position_measurement_right");
  this->declare_parameter("position_measurement_right_qos", 10);
  this->declare_parameter("fused_position_topic", "/fused_position");
  this->declare_parameter("fused_position_qos", 10);
  this->declare_parameter("fused_position_broadcast_rate_hz", 1.0f);
  this->declare_parameter("center_distance_from_left_node_percent", 0.5f);
  this->declare_parameter("pair_distance_m", 0.0f);
  this->declare_parameter("are_beacons_paired", false);
  this->declare_parameter("fusion_exponential_averaging_alpha", 0.8f);
  this->declare_parameter("position_buffer_size", 10);
  this->declare_parameter("left_beacon_address", 7);
  this->declare_parameter("right_beacon_address", 8);

  left_beacon_address = this->get_parameter("left_beacon_address").as_int();
  right_beacon_address = this->get_parameter("right_beacon_address").as_int();


  auto set_position_buffer_size = this->get_parameter("position_buffer_size").as_int();
  if (set_position_buffer_size > 0)
  {
    position_buffer_size_ = set_position_buffer_size;
  }
  else
  {
    RCLCPP_WARN(
      this->get_logger(),
      "Position buffer size value invalid: %d. Using default value: %d",
      position_buffer_size_,
      default_position_buffer_size
    );

    position_buffer_size_ = default_position_buffer_size;
  }
  

  vec_left_beacon_buffer_ = std::vector<PositionMsg>(position_buffer_size_);
  vec_right_beacon_buffer_ = std::vector<PositionMsg>(position_buffer_size_);
  
  
  f_are_beacon_paired_ = this->get_parameter("are_beacons_paired").as_bool();
  RCLCPP_INFO(
    this->get_logger(),
    "Beacons are paired: %s",
    (f_are_beacon_paired_) ? "TRUE" : "FALSE"
  );
  

  fusion_alpha_ = this->get_parameter("fusion_exponential_averaging_alpha").as_double();
  if (fusion_alpha_ < 0.0f || fusion_alpha_ > 1.0f)
  {
    RCLCPP_FATAL(
      this->get_logger(),
      "Exponential averaging alpha parameter value invalid. "
      "Expected value in range [0.0f, 1.0f], got: %01.3f",
      fusion_alpha_
    );

    exit(-1);
  }

  center_point_distance_from_left_node_percentage_ = this->get_parameter(
    "center_distance_from_left_node_percent").as_double();

  if (center_point_distance_from_left_node_percentage_ < 0.0f
      || center_point_distance_from_left_node_percentage_ > 1.0f
    )
  {
    RCLCPP_FATAL(
      this->get_logger(),
      "Center point distance from left node must be expressed as "
      "percentage of left-right pair distance i.e. in range of <0.0f, 1.0f>. "
      "Current value: %01.3f",
      center_point_distance_from_left_node_percentage_
    );

    exit(-1);
  }

  pair_distance_m_ = this->get_parameter("pair_distance_m").as_double();
  if (pair_distance_m_ < 0.0f)
  {
    RCLCPP_FATAL(
      this->get_logger(),
      "Distance between nodes cannot be less than 0.0f. Current distance: %03.3f m.",
      pair_distance_m_
    );
    
    exit(-1);
  }
  

  auto position_measurement_left_topic = this->get_parameter("position_measurement_left_topic").as_string();
  auto position_measurement_left_qos = this->get_parameter("position_measurement_left_qos").as_int();
  auto position_measurement_left_callback = std::bind(
    &MarvelmindPositionFusionNode::on_left_position_receive_,
    this,
    _1
  );

  p_left_beacon_position_subscription_ = this->create_subscription<PositionMsg>(
    position_measurement_left_topic,
    position_measurement_left_qos,
    position_measurement_left_callback
  );



  auto position_measurement_right_topic = this->get_parameter("position_measurement_right_topic").as_string();
  auto position_measurement_right_qos = this->get_parameter("position_measurement_right_qos").as_int();
  auto position_measurement_right_callback = std::bind(
    &MarvelmindPositionFusionNode::on_right_position_receive_,
    this,
    _1
  );

  p_right_beacon_position_subscription_ = this->create_subscription<PositionMsg>(
    position_measurement_right_topic,
    position_measurement_right_qos,
    position_measurement_right_callback
  );



  // TODO: Maybe make it double
  auto fused_position_broadcast_rate_hz = this->get_parameter("fused_position_broadcast_rate_hz").as_double();
  auto fused_position_broadcast_timer_period = rclcpp::Rate(fused_position_broadcast_rate_hz).period();
  auto fused_position_timer_callback = std::bind(
    &MarvelmindPositionFusionNode::on_position_broadcast_timer_trigger_,
    this
  );

  p_fused_position_broadcast_timer_ = this->create_wall_timer(
    fused_position_broadcast_timer_period,
    fused_position_timer_callback
  );


  auto fused_position_topic = this->get_parameter("fused_position_topic").as_string();
  auto fused_position_qos = this->get_parameter("fused_position_qos").as_int();

  p_fused_position_publisher_ = this->create_publisher<PositionMsg>(
    fused_position_topic,
    fused_position_qos
  );

  fused_position_ = PositionMsg();
  fused_position_.x_m = 0;
  fused_position_.y_m = 0;
  fused_position_.angle = 0;

  // Fill position buffers with initial position
  std::fill_n(vec_left_beacon_buffer_.begin(), position_buffer_size_, fused_position_);
  std::fill_n(vec_right_beacon_buffer_.begin(), position_buffer_size_, fused_position_);

  left_beacon_filtered_position_ = fused_position_;
  right_beacon_filtered_position_ = fused_position_;

  left_beacon_position_insert_iterator = vec_left_beacon_buffer_.begin();
  right_beacon_position_insert_iterator = vec_right_beacon_buffer_.begin();
  
}

void MarvelmindPositionFusionNode::on_left_position_receive_(PositionMsg::ConstSharedPtr p_msg)
{
  if (p_msg->address == right_beacon_address)
  {
    RCLCPP_DEBUG(
      this->get_logger(),
      "Left beacon read location from right beacon. Rerouting callback..."
    );
    on_right_position_receive_(p_msg);
    return;
  }
  else if (p_msg->address != left_beacon_address)
  {
    RCLCPP_DEBUG(
      this->get_logger(),
      "Expected position of beacon with address %d got %d. Discarding reading.",
      left_beacon_address, p_msg->address
    );

    return;
  }
  

  RCLCPP_DEBUG(
    this->get_logger(),
    "Received position from left beacon: <x, y, heading> = <%03.3f m, %03.3f m, %03.3f DEG>",
    p_msg->x_m, p_msg->y_m, p_msg->angle
  );

  if (f_are_beacon_paired_)
  {
    // Beacons are paired:
    // Both left and right beacons give center position:
    feed_new_center_position_measurement_to_filter_(p_msg);
  }
  else
  {
    // Beacons are unpaired:
    // Each beacon streams only its position.

    // Write last measurement and update iterator
    *(left_beacon_position_insert_iterator++) = *p_msg;
    if (left_beacon_position_insert_iterator == vec_left_beacon_buffer_.end())
    {
      left_beacon_position_insert_iterator = vec_left_beacon_buffer_.begin();
    }
    
    // Update filtered position of left beacon:
    left_beacon_filtered_position_.x_m = 0;
    left_beacon_filtered_position_.y_m = 0;

    const auto averaging_weight = (1.0f / position_buffer_size_);

    auto callback = [&](PositionMsg measurement){
      left_beacon_filtered_position_.x_m += averaging_weight * measurement.x_m;
      left_beacon_filtered_position_.y_m += averaging_weight * measurement.y_m;
    };

    std::for_each(
      vec_left_beacon_buffer_.begin(),
      vec_left_beacon_buffer_.end(),
      callback
    );

    // If beacons are not paired, then this node will calculate 
    // center position in timer callback. No need to feed it.
  }
  

}

void MarvelmindPositionFusionNode::on_right_position_receive_(PositionMsg::ConstSharedPtr p_msg)
{
  if (p_msg->address == left_beacon_address)
  {
    RCLCPP_DEBUG(
      this->get_logger(),
      "Right beacon read location from left beacon. Rerouting callback..."
    );
    on_left_position_receive_(p_msg);
    return;
  }
  else if (p_msg->address != right_beacon_address)
  {
    RCLCPP_DEBUG(
      this->get_logger(),
      "Expected position of beacon with address %d got %d. Discarding reading.",
      right_beacon_address, p_msg->address
    );

    return;
  }
  
  RCLCPP_DEBUG(
    this->get_logger(),
    "Received position from right beacon: <x, y, heading> = <%03.3f m, %03.3f m, %03.3f DEG>",
    p_msg->x_m, p_msg->y_m, p_msg->angle
  );

  if (f_are_beacon_paired_)
  {
    // Beacons are paired:
    // Both left and right beacons give center position:
    feed_new_center_position_measurement_to_filter_(p_msg);
  }
  else
  {
    // Beacons are unpaired:
    // Each beacon streams only its position.

    // Write last measurement and update iterator
    *(right_beacon_position_insert_iterator++) = *p_msg;
    if (right_beacon_position_insert_iterator == vec_right_beacon_buffer_.end())
    {
      right_beacon_position_insert_iterator = vec_right_beacon_buffer_.begin();
    }
    
    // Update filtered position of right beacon:
    right_beacon_filtered_position_.x_m = 0;
    right_beacon_filtered_position_.y_m = 0;

    const auto averaging_weight = (1.0f / position_buffer_size_);

    auto callback = [&](PositionMsg measurement){
      right_beacon_filtered_position_.x_m += averaging_weight * measurement.x_m;
      right_beacon_filtered_position_.y_m += averaging_weight * measurement.y_m;
    };

    std::for_each(
      vec_right_beacon_buffer_.begin(),
      vec_right_beacon_buffer_.end(),
      callback
    );

    // If beacons are not paired, then this node will calculate 
    // center position in timer callback. No need to feed it.
  }
}

void MarvelmindPositionFusionNode::feed_new_center_position_measurement_to_filter_(PositionMsg::ConstSharedPtr p_msg)
{
  fused_position_.x_m = (fusion_alpha_ * fused_position_.x_m) + (1.0f - fusion_alpha_) * (p_msg->x_m);
  fused_position_.y_m = (fusion_alpha_ * fused_position_.y_m) + (1.0f - fusion_alpha_) * (p_msg->y_m);
  fused_position_.angle = (fusion_alpha_ * fused_position_.angle) + (1.0f - fusion_alpha_) * (p_msg->angle);
}

void MarvelmindPositionFusionNode::on_position_broadcast_timer_trigger_()
{
  if (!f_are_beacon_paired_)
  {
    // If beacons are paired then each (left/right)
    // beacon streams pair's center position which is
    // immediately fed into an EMA filter.

    // If beacons are not paired, each beacon saves its
    // own positions in a buffer and prefilter position using
    // averaging so that orientation evolves with high degree of stability
    // We also have to calculate center position manually:
    // Note: Center position is located on a line between LEFT and RIGHT
    // beacon at position p*L from LEFT beacon where
    //  `p` [center_distance_from_left_node_percent] is distance of
    // center position from LEFT beacon expressed in fractions of 
    // pair base length `L` which is distance between two beacons
    // expressed in meters

    const auto p = center_point_distance_from_left_node_percentage_;
    const auto L = pair_distance_m_;

    const auto d_y = left_beacon_filtered_position_.y_m - right_beacon_filtered_position_.y_m;
    const auto d_x = left_beacon_filtered_position_.x_m - right_beacon_filtered_position_.x_m;
	
    // This is angle of line that goes through beacons.
    auto angle_deg = atan2(d_y, d_x) * (360.0f / 6.28f);
    // Subtract 90 DEG to get heading angle
    auto heading_deg = angle_deg - 90;
    auto center_x = (1 - p) * d_x + right_beacon_filtered_position_.x_m;
    auto center_y = (1 - p) * d_y + right_beacon_filtered_position_.y_m;

    fused_position_.x_m = center_x;
    fused_position_.y_m = center_y;
    fused_position_.angle = heading_deg;
  }
  
  RCLCPP_DEBUG(
    this->get_logger(),
    "Fused position timer triggered. Broadcasting fused position: "
    "<x, y, heading> = <%03.3f m, %03.3f m, %03.3f DEG>",
    fused_position_.x_m, fused_position_.y_m, fused_position_.angle
  );

  p_fused_position_publisher_->publish(fused_position_);
}
