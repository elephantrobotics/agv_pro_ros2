#ifndef AGV_PRO_MULTI__ROBOT_FOOTPRINT_LAYER_HPP_
#define AGV_PRO_MULTI__ROBOT_FOOTPRINT_LAYER_HPP_

#include <string>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "rclcpp/rclcpp.hpp"

namespace agv_pro_multi
{

class RobotFootprintLayer : public nav2_costmap_2d::Layer
{
public:
  void onInitialize() override;

  void updateBounds(
    double robot_x, double robot_y, double robot_yaw,
    double * min_x, double * min_y, double * max_x, double * max_y) override;

  void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j) override;

  void reset() override;

  bool isClearable() override {return false;}

private:
  struct RobotPose
  {
    double x;
    double y;
    double yaw;
  };

  bool lookupRobot(const std::string & robot, RobotPose & pose);
  std::vector<geometry_msgs::msg::Point> cornersAt(const RobotPose & pose) const;
  void stampPose(
    nav2_costmap_2d::Costmap2D & master_grid, const RobotPose & pose,
    int min_i, int min_j, int max_i, int max_j) const;
  void refreshHalfExtents();

  std::vector<std::string> other_robots_;
  std::vector<RobotPose> poses_;

  std::string base_suffix_;
  std::string global_frame_;
  double tf_timeout_{0.5};
  double margin_{0.05};
  double half_length_{0.265};
  double half_width_{0.18};

  bool has_last_bounds_{false};
  double last_min_x_{0.0};
  double last_min_y_{0.0};
  double last_max_x_{0.0};
  double last_max_y_{0.0};

  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Logger logger_{rclcpp::get_logger("RobotFootprintLayer")};
};

}  // namespace agv_pro_multi

#endif  // AGV_PRO_MULTI__ROBOT_FOOTPRINT_LAYER_HPP_
