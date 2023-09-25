// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

// std
#include <functional>
#include <memory>
#include <string>
#include <utility>

// romea
#include "romea_robot_to_world_localisation_rtls_plugin/robot_to_world_rtls_localisation_plugin.hpp"
#include "romea_rtls_transceiver_utils/rtls_transceiver_data_conversions.hpp"
#include "romea_rtls_utils/rtls_parameters.hpp"

#include "romea_common_utils/params/algorithm_parameters.hpp"
#include "romea_common_utils/qos.hpp"

namespace romea
{

//-----------------------------------------------------------------------------
R2WRTLSLocalisationPlugin::R2WRTLSLocalisationPlugin(const rclcpp::NodeOptions & options)
: node_(std::make_shared<rclcpp::Node>("robot_to_world_rtls_localisation_plugin", options)),
  plugin_(nullptr),
  scheduler_(nullptr),
  range_observation_(),
  pose_observation_(),
  odom_sub_(nullptr),
  rtls_communication_hub_(nullptr),
  range_pub_(nullptr),
  pose_pub_(nullptr),
  diagnostic_pub_(nullptr)
{
  declare_parameters_();
  init_plugin_();
  init_communication_hub_();
  init_range_publisher_();
  init_pose_publisher_();
  init_diagnostic_publisher_();
  init_scheduler_();
  init_odom_subscriber_();
}

//-----------------------------------------------------------------------------
rclcpp::node_interfaces::NodeBaseInterface::SharedPtr
R2WRTLSLocalisationPlugin::get_node_base_interface() const
{
  return node_->get_node_base_interface();
}

//-----------------------------------------------------------------------------
void R2WRTLSLocalisationPlugin::declare_parameters_()
{
  declare_map_frame_id(node_);
  declare_poll_rate(node_);
  declare_range_std(node_);
  declare_minimal_range(node_);
  declare_maximal_range(node_);
  declare_initiators_ids(node_);
  declare_initiators_names(node_);
  declare_initiators_positions(node_);
  declare_responders_ids(node_);
  declare_responders_names(node_);
  declare_responders_positions(node_);
  declare_enable_scheduler(node_);
}

//-----------------------------------------------------------------------------
void R2WRTLSLocalisationPlugin::init_odom_subscriber_()
{
  using namespace std::placeholders;
  auto cb = std::bind(&R2WRTLSLocalisationPlugin::process_odom_, this, _1);
  odom_sub_ = node_->create_subscription<OdometryMsg>("filtered_odom", best_effort(1), cb);
}

//-----------------------------------------------------------------------------
void R2WRTLSLocalisationPlugin::init_communication_hub_()
{
  using namespace std::placeholders;
  auto cb = std::bind(&R2WRTLSLocalisationPlugin::process_range_, this, _1, _2, _3);
  rtls_communication_hub_ = std::make_unique<RTLSCommunicationHub>(node_, cb);
}

//-----------------------------------------------------------------------------
void R2WRTLSLocalisationPlugin::init_range_publisher_()
{
  range_pub_ = node_->create_publisher<ObservationRangeStampedMsg>(
    "range", sensor_data_qos());
}

//-----------------------------------------------------------------------------
void R2WRTLSLocalisationPlugin::init_pose_publisher_()
{
  pose_pub_ = make_stamped_data_publisher<ObservationPose, ObservationPose2DStampedMsg>(
    node_, "pose", get_map_frame_id(node_), sensor_data_qos(), true);
}

//-----------------------------------------------------------------------------
void R2WRTLSLocalisationPlugin::init_diagnostic_publisher_()
{
  diagnostic_pub_ = make_diagnostic_publisher<DiagnosticReport>(node_, node_->get_name(), 1.0);
}

//-----------------------------------------------------------------------------
void R2WRTLSLocalisationPlugin::init_scheduler_()
{
  if (get_enable_scheduler(node_)) {
    using namespace std::placeholders;
    auto cb = std::bind(&R2WRTLSLocalisationPlugin::process_ranging_request_, this, _1, _2, _3);

    scheduler_ = std::make_unique<Scheduler>(
      get_poll_rate(node_),
      get_maximal_range(node_),
      get_initiators_names(node_),
      get_initiators_positions(node_),
      get_responders_names(node_),
      get_responders_positions(node_),
      cb);

    scheduler_->start();
  }
}

//-----------------------------------------------------------------------------
void R2WRTLSLocalisationPlugin::init_plugin_()
{
  plugin_ = std::make_unique<Plugin>(
    get_range_std(node_),
    get_minimal_range(node_),
    get_maximal_range(node_),
    20,   // rxPowerRejectionThreshold
    get_initiators_positions(node_),
    get_responders_positions(node_));
}


//-----------------------------------------------------------------------------
void R2WRTLSLocalisationPlugin::process_odom_(OdometryMsg::ConstSharedPtr msg)
{
  Eigen::Vector3d robot_position;
  robot_position.x() = msg->pose.pose.position.x;
  robot_position.y() = msg->pose.pose.position.y;
  robot_position.z() = msg->pose.pose.position.z;
  scheduler_->updateRobotPosition(robot_position);
}

//-----------------------------------------------------------------------------
void R2WRTLSLocalisationPlugin::process_ranging_request_(
  const size_t & initiator_index,
  const size_t & responder_index,
  const Duration & timeout)
{
  rtls_communication_hub_->send_ranging_request(
    initiator_index, responder_index, durationToSecond(timeout));

  if (initiator_index == 0 && responder_index == 0) {
    plugin_->selectRespondersRanges(scheduler_->getSelectedRespondersIndexes());
    diagnostic_pub_->publish(node_->get_clock()->now(), scheduler_->getReport());
  }
}

//-----------------------------------------------------------------------------
void R2WRTLSLocalisationPlugin::process_range_(
  const size_t & initiator_index,
  const size_t & responder_index,
  const RangeMsg & range)
{
  RangingResult ranging_result = to_romea(range);

  if (plugin_->processRangingResult(
      initiator_index, responder_index,
      ranging_result, range_observation_))
  {
    publish_range_(range.stamp, "toto");
  }

  if (plugin_->computePose(pose_observation_)) {
    pose_pub_->publish(range.stamp, pose_observation_);
  }

  scheduler_->feedback(initiator_index, responder_index, ranging_result);
}

//-----------------------------------------------------------------------------
void R2WRTLSLocalisationPlugin::publish_range_(
  const rclcpp::Time & stamp,
  const std::string & frame_id)
{
  auto range_msg = std::make_unique<ObservationRangeStampedMsg>();
  to_ros_msg(stamp, frame_id, range_observation_, *range_msg);
  range_pub_->publish(std::move(range_msg));
}

}  // namespace romea

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(romea::R2WRTLSLocalisationPlugin)
