#ifndef AGV_PRO_DRIVER_H
#define AGV_PRO_DRIVER_H

#include <algorithm> 
#include <iostream>
#include <boost/asio.hpp>

#include "rclcpp/rclcpp.hpp"

#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float32.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <agv_pro_msgs/srv/set_digital_output.hpp>
#include <agv_pro_msgs/srv/get_digital_input.hpp>

#define SEND_DATA_SIZE 14                               // Total bytes in a command frame to ESP32(version>=V1.0.8)
#define RECEIVE_FRAME_SIZE 31                           // Total bytes in a frame from ESP32(version>=V1.0.8)
#define RECEIVE_PAYLOAD_SIZE (RECEIVE_FRAME_SIZE - 3)   // Payload length (excluding header)

#define POWER_ON 0x10
#define GET_POWER_STATE 0x12
#define SET_AUTO_REPORT_STATE 0x23
#define SET_OUTPUT_IO 0x40
#define GET_INPUT_IO 0x41

extern std::array<double, 36> odom_pose_covariance;
extern std::array<double, 36> odom_twist_covariance;

class AGV_PRO : public rclcpp::Node
{
public:
  /**
   * @brief Constructor
   */
  AGV_PRO(std::string node_name);

  /**
   * @brief Destructor
   */
  ~AGV_PRO();

private:
  /**
   * @brief Main control loop for the AGV.
   */
  void Control();

  /**
   * @brief Print a vector of bytes in hexadecimal format to the ROS logger.
   *
   * @param[in] label A label to prepend to the printed data.
   * @param[in] data The byte vector to print.
   * @param[in] override_size Optional size to display instead of the full data length.
   */
  void print_hex(const std::string& label,
    const std::vector<uint8_t>& data,
    std::optional<size_t> override_size = std::nullopt);

  /**
   * @brief Send a serial frame to the AGV and optionally print it in hex.
   *
   * @param[in] frame The byte vector representing the serial frame to send.
   * @param[in] debug If true, prints the transmitted frame using print_hex().
   */
  void send_serial_frame(const std::vector<uint8_t>& frame, bool debug);

  /**
   * @brief Query and print the current power status of the AGV.
   * @return true if AGV is successfully power on; false otherwise.
   */
  bool is_power_on();

  /**
   * @brief Enable or disable AGV auto-reporting
   * @param[in] enable 0 = disable, 1 = enable
   */
  void set_auto_report(bool enable);

  /**
   * @brief Clear the serial port input and output buffers
   * @param[in] fd File descriptor of the serial port
   */
  void clearSerialBuffer(int fd);

  /**
   * @brief Disable the DTR (Data Terminal Ready) and RTS (Request To Send) lines of the serial port
   * @param[in] fd File descriptor of the serial port
   */
  void disableDTR_RTS(int fd);

  /**
   * @brief Read sensor and motor data from the AGV via serial port.
   * @return true if data is successfully read and verified; false otherwise.
   */
  bool readData();

  /**
   * @brief Odometry publisher
   * @param[in] dt Time difference (in seconds) since the last odometry update.
   */
  void publisherOdom(double dt);

  /**
   * @brief Voltage publisher
   */
  void publisherVoltage();

  /**
   * @brief ImuSensor publisher
   */
  void publisherImuSensor();

  /**
   * @brief Callback for velocity command updates
   * @param[in] msg The Twist message containing desired linear and angular velocities
   */
  void cmdCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  
  /**
   * @brief Build a standard AGV serial frame with header, payload, and CRC.
   *
   * The frame has a fixed size of RECEIVE_DATA_SIZE, starts with 0xFE 0xFE 0x0B,
   * includes a command ID and up to 8 payload bytes, and ends with a 16-bit CRC.
   *
   * @param[in] cmd_id The command ID for the serial frame.
   * @param[in] payload The payload bytes to include (up to 8 bytes).
   * @return A vector containing the complete serial frame ready to transmit.
   */
  std::vector<uint8_t> build_serial_frame(uint8_t cmd_id, const std::vector<uint8_t>& payload);

  /**
   * @brief Read a serial response from the AGV device, waiting for a specific header.
   *
   * This function reads bytes from the serial port until the expected header
   * sequence is detected or the timeout expires. After detecting the header,
   * it reads the remaining payload bytes along with a 2-byte CRC.
   *
   * @param[in] expected_header The byte sequence to identify the start of a valid frame.
   * @param[in] payload_size The expected number of payload bytes following the header.
   * @param[in] timeout_sec Maximum time (in seconds) to wait for the header.
   * @return A vector containing the complete frame (header + payload + CRC).
   *         Returns an empty vector if a timeout occurs or the full payload is not received.
   */
  std::vector<uint8_t> read_serial_response(
    const std::vector<uint8_t>& expected_header,
    size_t payload_size,
    double timeout_sec);

  /**
   * @brief Compute the CRC-16-IBM checksum for a byte array.
   *
   * This function calculates the CRC using the standard IBM polynomial 0xA001.
   *
   * @param[in] data Pointer to the byte array.
   * @param[in] length Number of bytes to include in the CRC calculation.
   * @return The computed 16-bit CRC value.
   */
  uint16_t crc16_ibm(const uint8_t* data, size_t length);

  /**
   * @brief Handle the SetDigitalOutput service request.
   *
   * This service sets the state (HIGH/LOW) of a specific digital output pin on the AGV device.
   * The request contains the pin number and desired state, which are sent to the hardware
   * via the serial interface. The response reports whether the operation succeeded.
   *
   * @param[in] request The service request, containing:
   *   - pin: The digital output pin number.
   *   - state: Desired output state (true = HIGH, false = LOW).
   * @param[out] response The service response, containing:
   *   - success: True if the operation succeeded.
   *   - message: Optional status or error description.
   */
  void handleSetDigitalOutput(
    const std::shared_ptr<agv_pro_msgs::srv::SetDigitalOutput::Request> request,
    std::shared_ptr<agv_pro_msgs::srv::SetDigitalOutput::Response> response);

  /**
   * @brief Handle the GetDigitalInput service request.
   *
   * This service reads the state (HIGH/LOW) of a specific digital input pin on the AGV device.
   * The request specifies the pin number, and the node queries the hardware via the serial
   * interface to retrieve its current state.
   *
   * @param[in] request The service request, containing:
   *   - pin: The digital input pin number to read.
   * @param[out] response The service response, containing:
   *   - state: Current pin state (true = HIGH, false = LOW).
   *   - success: True if the read operation succeeded.
   *   - message: Optional status or error description.
   */
  void handleGetDigitalInput(
    const std::shared_ptr<agv_pro_msgs::srv::GetDigitalInput::Request> request,
    std::shared_ptr<agv_pro_msgs::srv::GetDigitalInput::Response> response);

  boost::asio::io_service io_;
  std::unique_ptr<boost::asio::serial_port> serial_port_;

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

  double ax= 0.0;
  double ay= 0.0;
  double az= 0.0;

  double wx= 0.0;
  double wy= 0.0;
  double wz= 0.0;

  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;

  int is_poweron_status = 0;
  int poweron_status = 0;

  uint8_t motor_status = 0;
  uint8_t motor_error = 0;
  uint8_t enable_status = 0;
  
  float battery_voltage = 0.0f;

  std::array<double, 36> odom_pose_covariance = {
    {1e-9, 0, 0, 0, 0, 0,
    0, 1e-3, 1e-9, 0, 0, 0,
    0, 0, 1e6, 0, 0, 0,
    0, 0, 0, 1e6, 0, 0,
    0, 0, 0, 0, 1e6, 0,
    0, 0, 0, 0, 0, 1e-9} };

  std::array<double, 36> odom_twist_covariance = {
    {1e-9, 0, 0, 0, 0, 0,
    0, 1e-3, 1e-9, 0, 0, 0,
    0, 0, 1e6, 0, 0, 0,
    0, 0, 0, 1e6, 0, 0,
    0, 0, 0, 0, 1e6, 0,
    0, 0, 0, 0, 0, 1e-9} };

  rclcpp::Time currentTime, lastTime;
  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_voltage;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub;
  rclcpp::Service<agv_pro_msgs::srv::SetDigitalOutput>::SharedPtr set_output_service;
  rclcpp::Service<agv_pro_msgs::srv::GetDigitalInput>::SharedPtr get_input_service;

  sensor_msgs::msg::Imu imu_data;
  std::unique_ptr<tf2_ros::TransformBroadcaster> odomBroadcaster;
};

#endif