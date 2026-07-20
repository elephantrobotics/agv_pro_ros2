#include "agv_pro_multi/robot_footprint_layer.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/exceptions.h"
#include "tf2/time.h"

namespace agv_pro_multi
{

namespace
{
double yawFromQuat(const geometry_msgs::msg::Quaternion & q)
{
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}
}  // namespace

void RobotFootprintLayer::onInitialize()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"RobotFootprintLayer: failed to lock node"};
  }
  clock_ = node->get_clock();
  logger_ = node->get_logger();

  declareParameter("enabled", rclcpp::ParameterValue(true));
  declareParameter("robot_id", rclcpp::ParameterValue(std::string("")));
  declareParameter(
    "fleet", rclcpp::ParameterValue(std::vector<std::string>({"robot1", "robot2", "robot3"})));
  declareParameter("base_suffix", rclcpp::ParameterValue(std::string("/base_footprint")));
  declareParameter("tf_timeout", rclcpp::ParameterValue(0.5));
  declareParameter("margin", rclcpp::ParameterValue(0.05));

  std::string robot_id;
  std::vector<std::string> fleet;
  node->get_parameter(name_ + "." + "enabled", enabled_);
  node->get_parameter(name_ + "." + "robot_id", robot_id);
  node->get_parameter(name_ + "." + "fleet", fleet);
  node->get_parameter(name_ + "." + "base_suffix", base_suffix_);
  node->get_parameter(name_ + "." + "tf_timeout", tf_timeout_);
  node->get_parameter(name_ + "." + "margin", margin_);

  global_frame_ = layered_costmap_->getGlobalFrameID();

  other_robots_.clear();
  for (const auto & name : fleet) {
    if (name != robot_id) {
      other_robots_.push_back(name);
    }
  }

  if (other_robots_.empty()) {
    RCLCPP_WARN(
      logger_, "%s: fleet lists no teammate besides robot_id '%s'; layer stays idle",
      name_.c_str(), robot_id.c_str());
  }

  current_ = true;
}

void RobotFootprintLayer::refreshHalfExtents()
{
  const auto & footprint = getFootprint();
  if (footprint.empty()) {
    return;
  }
  double half_length = 0.0;
  double half_width = 0.0;
  for (const auto & point : footprint) {
    half_length = std::max(half_length, std::abs(point.x));
    half_width = std::max(half_width, std::abs(point.y));
  }
  half_length_ = half_length + margin_;
  half_width_ = half_width + margin_;
}

bool RobotFootprintLayer::lookupRobot(const std::string & robot, RobotPose & pose)
{
  geometry_msgs::msg::TransformStamped transform;
  try {
    transform = tf_->lookupTransform(global_frame_, robot + base_suffix_, tf2::TimePointZero);
  } catch (const tf2::TransformException &) {
    return false;
  }

  const double age =
    (clock_->now().nanoseconds() - rclcpp::Time(transform.header.stamp).nanoseconds()) * 1e-9;
  if (age > tf_timeout_) {
    return false;
  }

  pose.x = transform.transform.translation.x;
  pose.y = transform.transform.translation.y;
  pose.yaw = yawFromQuat(transform.transform.rotation);
  return true;
}

std::vector<geometry_msgs::msg::Point> RobotFootprintLayer::cornersAt(const RobotPose & pose) const
{
  const double cy = std::cos(pose.yaw);
  const double sy = std::sin(pose.yaw);
  const double dx[4] = {half_length_, half_length_, -half_length_, -half_length_};
  const double dy[4] = {half_width_, -half_width_, -half_width_, half_width_};

  std::vector<geometry_msgs::msg::Point> corners(4);
  for (int i = 0; i < 4; ++i) {
    corners[i].x = pose.x + dx[i] * cy - dy[i] * sy;
    corners[i].y = pose.y + dx[i] * sy + dy[i] * cy;
    corners[i].z = 0.0;
  }
  return corners;
}

void RobotFootprintLayer::stampPose(
  nav2_costmap_2d::Costmap2D & master_grid, const RobotPose & pose,
  int min_i, int min_j, int max_i, int max_j) const
{
  const double radius = std::hypot(half_length_, half_width_);

  unsigned int lo_i;
  unsigned int lo_j;
  unsigned int hi_i;
  unsigned int hi_j;
  const double origin_x = master_grid.getOriginX();
  const double origin_y = master_grid.getOriginY();
  const double resolution = master_grid.getResolution();
  const double size_x = master_grid.getSizeInCellsX();
  const double size_y = master_grid.getSizeInCellsY();

  const double lo_wx = std::max(pose.x - radius, origin_x);
  const double lo_wy = std::max(pose.y - radius, origin_y);
  const double hi_wx = std::min(pose.x + radius, origin_x + size_x * resolution);
  const double hi_wy = std::min(pose.y + radius, origin_y + size_y * resolution);
  if (lo_wx >= hi_wx || lo_wy >= hi_wy) {
    return;
  }
  if (!master_grid.worldToMap(lo_wx, lo_wy, lo_i, lo_j)) {
    return;
  }
  if (!master_grid.worldToMap(hi_wx - resolution / 2.0, hi_wy - resolution / 2.0, hi_i, hi_j)) {
    return;
  }

  const int start_i = std::max(static_cast<int>(lo_i), min_i);
  const int start_j = std::max(static_cast<int>(lo_j), min_j);
  const int end_i = std::min(static_cast<int>(hi_i), max_i - 1);
  const int end_j = std::min(static_cast<int>(hi_j), max_j - 1);

  const double cy = std::cos(pose.yaw);
  const double sy = std::sin(pose.yaw);

  for (int j = start_j; j <= end_j; ++j) {
    for (int i = start_i; i <= end_i; ++i) {
      double wx;
      double wy;
      master_grid.mapToWorld(static_cast<unsigned int>(i), static_cast<unsigned int>(j), wx, wy);
      const double dx = wx - pose.x;
      const double dy = wy - pose.y;
      const double local_x = dx * cy + dy * sy;
      const double local_y = -dx * sy + dy * cy;
      if (std::abs(local_x) <= half_length_ && std::abs(local_y) <= half_width_) {
        master_grid.setCost(
          static_cast<unsigned int>(i), static_cast<unsigned int>(j),
          nav2_costmap_2d::LETHAL_OBSTACLE);
      }
    }
  }
}

void RobotFootprintLayer::updateBounds(
  double, double, double, double * min_x, double * min_y, double * max_x, double * max_y)
{
  poses_.clear();
  if (!enabled_) {
    has_last_bounds_ = false;
    return;
  }

  refreshHalfExtents();

  double cur_min_x = 1e30;
  double cur_min_y = 1e30;
  double cur_max_x = -1e30;
  double cur_max_y = -1e30;

  for (const auto & other : other_robots_) {
    RobotPose pose;
    if (!lookupRobot(other, pose)) {
      continue;
    }
    for (const auto & corner : cornersAt(pose)) {
      cur_min_x = std::min(cur_min_x, corner.x);
      cur_min_y = std::min(cur_min_y, corner.y);
      cur_max_x = std::max(cur_max_x, corner.x);
      cur_max_y = std::max(cur_max_y, corner.y);
    }
    poses_.push_back(pose);
  }

  const bool have_current = !poses_.empty();
  if (have_current) {
    *min_x = std::min(*min_x, cur_min_x);
    *min_y = std::min(*min_y, cur_min_y);
    *max_x = std::max(*max_x, cur_max_x);
    *max_y = std::max(*max_y, cur_max_y);
  }

  if (has_last_bounds_) {
    *min_x = std::min(*min_x, last_min_x_);
    *min_y = std::min(*min_y, last_min_y_);
    *max_x = std::max(*max_x, last_max_x_);
    *max_y = std::max(*max_y, last_max_y_);
  }

  has_last_bounds_ = have_current;
  if (have_current) {
    last_min_x_ = cur_min_x;
    last_min_y_ = cur_min_y;
    last_max_x_ = cur_max_x;
    last_max_y_ = cur_max_y;
  }
}

void RobotFootprintLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid, int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_) {
    return;
  }
  for (const auto & pose : poses_) {
    stampPose(master_grid, pose, min_i, min_j, max_i, max_j);
  }
}

void RobotFootprintLayer::reset()
{
  poses_.clear();
  has_last_bounds_ = false;
}

}  // namespace agv_pro_multi

PLUGINLIB_EXPORT_CLASS(agv_pro_multi::RobotFootprintLayer, nav2_costmap_2d::Layer)
