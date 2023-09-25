// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef ROMEA_ROBOT_TO_WORLD_LOCALISATION_RTLS_PLUGIN__ROBOT_TO_WORLD_RTLS_LOCALISATION_PLUGIN_HPP_
#define ROMEA_ROBOT_TO_WORLD_LOCALISATION_RTLS_PLUGIN__ROBOT_TO_WORLD_RTLS_LOCALISATION_PLUGIN_HPP_


// std
#include <string>
#include <memory>

// ros
#include "nav_msgs/msg/odometry.hpp"

// romea
#include "romea_core_localisation_rtls/R2WLocalisationRTLSPlugin.hpp"
#include "romea_localisation_utils/conversions/observation_range_conversions.hpp"
#include "romea_localisation_utils/conversions/observation_pose_conversions.hpp"

#include "romea_common_utils/conversions/diagnostic_conversions.hpp"
#include "romea_common_utils/publishers/diagnostic_publisher.hpp"
#include "romea_common_utils/publishers/stamped_data_publisher.hpp"

#include "romea_core_rtls/coordination/RTLSGeoreferencedCoordinatorScheduler.hpp"
#include "romea_rtls_utils/rtls_communication_hub.hpp"

// local
#include "romea_robot_to_world_localisation_rtls_plugin/visibility_control.h"

namespace romea
{

class R2WRTLSLocalisationPlugin
{
public:
  using Plugin = R2WLocalisationRTLSPlugin;
  using Scheduler = RTLSGeoreferencedCoordinatorScheduler;
  using RangingResult = RTLSTransceiverRangingResult;

  using OdometryMsg = nav_msgs::msg::Odometry;
  using RangeMsg = romea_rtls_transceiver_msgs::msg::Range;
  using PayloadMsg = romea_rtls_transceiver_msgs::msg::Payload;
  using ObservationRangeStampedMsg = romea_localisation_msgs::msg::ObservationRangeStamped;
  using ObservationPose2DStampedMsg = romea_localisation_msgs::msg::ObservationPose2DStamped;

public:
  ROMEA_ROBOT_TO_WOLRD_LOCALISATION_RTLS_PLUGIN_PUBLIC
  explicit R2WRTLSLocalisationPlugin(const rclcpp::NodeOptions & options);

  ROMEA_ROBOT_TO_WOLRD_LOCALISATION_RTLS_PLUGIN_PUBLIC
  virtual ~R2WRTLSLocalisationPlugin() = default;

  ROMEA_ROBOT_TO_WOLRD_LOCALISATION_RTLS_PLUGIN_PUBLIC
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr
  get_node_base_interface() const;

protected:
  void declare_parameters_();

  void init_plugin_();

  void init_scheduler_();

  void init_communication_hub_();

  void init_odom_subscriber_();

  void init_range_publisher_();

  void init_pose_publisher_();

  void init_diagnostic_publisher_();

  void process_ranging_request_(
    const size_t & initiatorIndex,
    const size_t & responderIndex,
    const Duration & timeout);

  void process_range_(
    const size_t & initiatorIndex,
    const size_t & responderIndex,
    const RangeMsg & range);

  void process_odom_(OdometryMsg::ConstSharedPtr msg);

  void publish_range_(const rclcpp::Time & stamp, const std::string & frame_id);

protected:
  rclcpp::Node::SharedPtr node_;

  std::unique_ptr<Plugin> plugin_;
  std::unique_ptr<Scheduler> scheduler_;
  romea::ObservationRange range_observation_;
  romea::ObservationPose pose_observation_;

  rclcpp::Subscription<OdometryMsg>::SharedPtr odom_sub_;
  std::unique_ptr<RTLSCommunicationHub> rtls_communication_hub_;
  rclcpp::Publisher<ObservationRangeStampedMsg>::SharedPtr range_pub_;
  std::shared_ptr<StampedPublisherBase<ObservationPose>> pose_pub_;
  std::shared_ptr<StampedPublisherBase<DiagnosticReport>> diagnostic_pub_;
  // rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace romea

#endif  // ROMEA_ROBOT_TO_WORLD_LOCALISATION_RTLS_PLUGIN__ROBOT_TO_WORLD_RTLS_LOCALISATION_PLUGIN_HPP_
