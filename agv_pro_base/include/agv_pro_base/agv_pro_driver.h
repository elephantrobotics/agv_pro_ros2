#ifndef AGV_PRO_DRIVER_H
#define AGV_PRO_DRIVER_H

#include <algorithm> 
#include "serial_driver/serial_driver.hpp"

#include "rclcpp/rclcpp.hpp"

#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float32.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#define RECEIVE_DATA_SIZE 14         //The length of the data sent by the esp32

extern std::array<double, 36> odom_pose_covariance;
extern std::array<double, 36> odom_twist_covariance;

class AGV_PRO : public rclcpp::Node
{
public:
  AGV_PRO(std::string node_name);
  ~AGV_PRO();
private:
  void Control();
  void print_hex(const std::string& label, const std::vector<uint8_t>& data, std::optional<size_t> override_size = std::nullopt);
  void send_serial_frame(const std::vector<uint8_t>& frame, bool debug);
  void is_power_on();
  void set_auto_report();
  bool readData();
  void publisherOdom(double dt);
  void publisherVoltage();
  void cmdCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  std::vector<uint8_t> build_serial_frame(uint8_t cmd_id, const std::vector<uint8_t>& payload);
  std::vector<uint8_t> read_serial_response(const std::vector<uint8_t>& expected_header, size_t payload_size, double timeout_sec);

  std::string frame_id_of_odometry_;
  std::string child_frame_id_of_odometry_;
  std::string frame_id_of_imu_;
  std::string name_space_;
  std::string device_name_;
  
  double x= 0.0;
  double y= 0.0;
  double theta= 0.0;

  double vx= 0.0;
  double vy= 0.0;
  double vtheta= 0.0;

  double linearX = 0.0;
  double linearY = 0.0;
  double angularZ = 0.0;

  int is_poweron_status = 0;
  int poweron_status = 0;

  uint8_t motor_status = 0;
  uint8_t motor_error = 0;
  uint8_t enable_status = 0;
  
  float battery_voltage = 0.0f;

  rclcpp::Time currentTime, lastTime;
  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_voltage;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub;

  std::unique_ptr<tf2_ros::TransformBroadcaster> odomBroadcaster;
  std::shared_ptr<drivers::serial_driver::SerialDriver> serial_driver_;
  std::shared_ptr<drivers::common::IoContext> io_context_;

};

#endif