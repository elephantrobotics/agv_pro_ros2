// Copyright (c) 2019 Intel Corporation
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

#include "nav2_rviz_plugins/charger_tool.hpp"

#include <memory>
#include <string>

#include "rviz_common/display_context.hpp"
#include "rviz_common/load_resource.hpp"
#include "rviz_common/properties/string_property.hpp"
#include "rviz_common/properties/qos_profile_property.hpp"

namespace nav2_rviz_plugins
{

ChargerTool::ChargerTool()
: rviz_default_plugins::tools::PoseTool(), qos_profile_(5)
{
  shortcut_key_ = 'c';
  topic_property_ = new rviz_common::properties::StringProperty(
    "Topic", "charger_position_update",
    "The topic on which to publish goals.",
    getPropertyContainer(), SLOT(updateTopic()), this);

  qos_profile_property_ = new rviz_common::properties::QosProfileProperty(
    topic_property_, qos_profile_);
}

ChargerTool::~ChargerTool()
{
}

void ChargerTool::onInitialize()
{
  PoseTool::onInitialize();
  setName("Charger Update");
  setIcon(rviz_common::loadPixmap("package://rviz_default_plugins/icons/classes/SetGoal.png"));
  updateTopic();
}

void ChargerTool::updateTopic()
{
  rclcpp::Node::SharedPtr raw_node =
    context_->getRosNodeAbstraction().lock()->get_raw_node();
  // TODO(anhosi, wjwwood): replace with abstraction for publishers once available
  publisher_ = raw_node->
    template create_publisher<geometry_msgs::msg::PoseStamped>(
    topic_property_->getStdString(), qos_profile_);
  clock_ = raw_node->get_clock();
}

void
ChargerTool::onPoseSet(double x, double y, double theta)
{
  std::string fixed_frame = context_->getFixedFrame().toStdString();

  geometry_msgs::msg::PoseStamped goal;
  goal.header.stamp = clock_->now();
  goal.header.frame_id = fixed_frame;

  goal.pose.position.x = x;
  goal.pose.position.y = y;
  goal.pose.position.z = 0.0;

  goal.pose.orientation = orientationAroundZAxis(theta);

  logPose("goal", goal.pose.position, goal.pose.orientation, theta, fixed_frame);

  publisher_->publish(goal);
}

}  // namespace nav2_rviz_plugins

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(nav2_rviz_plugins::ChargerTool, rviz_common::Tool)
