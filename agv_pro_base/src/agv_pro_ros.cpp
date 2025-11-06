#include "agv_pro_base/agv_pro_driver.h"

uint16_t AGV_PRO::crc16_ibm(const uint8_t* data, size_t length) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < length; ++i) {
    crc ^= static_cast<uint16_t>(data[i]);
    for (int j = 0; j < 8; ++j) {
      if (crc & 0x0001)
        crc = (crc >> 1) ^ 0xA001;
      else
        crc = crc >> 1;
    }
  }
  return crc;
}

std::vector<uint8_t> AGV_PRO::build_serial_frame(uint8_t cmd_id, const std::vector<uint8_t>& payload)
{
  std::vector<uint8_t> frame(SEND_DATA_SIZE, 0x00);
  frame[0] = 0xFE;
  frame[1] = 0xFE;
  frame[2] = 0x0B;
  frame[3] = cmd_id;

  for (size_t i = 0; i < payload.size() && i < 8; ++i) {
    frame[4 + i] = payload[i];
  }

  uint16_t crc = crc16_ibm(frame.data(), 12);
  frame[12] = (crc >> 8) & 0xff;
  frame[13] = crc & 0xff;

  return frame;
}

void AGV_PRO::print_hex(const std::string& label, const std::vector<uint8_t>& data, std::optional<size_t> override_size) {
  std::stringstream ss;
  for (auto b : data) {
    ss << std::hex << std::uppercase << std::setfill('0') << std::setw(2)
       << static_cast<int>(b) << " ";
  }
  size_t len = override_size.value_or(data.size());
  RCLCPP_INFO(this->get_logger(), "%s (%zu bytes): [%s]", label.c_str(), len, ss.str().c_str());
}

void AGV_PRO::send_serial_frame(const std::vector<uint8_t>& frame, bool debug)
{
  try {
    size_t bytes_transmit_size = boost::asio::write(*serial_port_, boost::asio::buffer(frame));
    if (debug) {
      print_hex("Sent", frame, bytes_transmit_size);
    }
  } catch (const std::exception &ex) {
    RCLCPP_ERROR(this->get_logger(), "Error Transmiting from serial port: %s", ex.what());
  }
}

std::vector<uint8_t> AGV_PRO::read_serial_response(
  const std::vector<uint8_t>& expected_header,
  size_t payload_size,
  double timeout_sec)
{
  std::vector<uint8_t> sliding_buf;
  uint8_t byte = 0;

  rclcpp::Time start_time = this->now();
  rclcpp::Duration timeout = rclcpp::Duration::from_seconds(timeout_sec);

  while ((this->now() - start_time) < timeout) {
    boost::asio::mutable_buffers_1 buf(&byte, 1);
    boost::system::error_code ec;
    size_t n = serial_port_->read_some(buf, ec);
    if (ec) {
        RCLCPP_WARN(this->get_logger(), "Serial read error: %s", ec.message().c_str());
        return {};
    }
    if (n == 1) {
      sliding_buf.push_back(byte);
      if (sliding_buf.size() > expected_header.size()) {
          sliding_buf.erase(sliding_buf.begin());
      }
      if (sliding_buf == expected_header) {
          break;
      }
    }
  }
  
  if (sliding_buf != expected_header) {
    RCLCPP_WARN(this->get_logger(), "Timeout waiting for header");
    return {};
  }

  size_t remain_len = payload_size + 2;
  std::vector<uint8_t> remain_buf(remain_len);
  size_t total_read = 0;

  while (total_read < remain_len && (this->now() - start_time) < timeout) {
    boost::asio::mutable_buffers_1 buf(&remain_buf[total_read], remain_len - total_read);
    boost::system::error_code ec;
    size_t n = serial_port_->read_some(buf, ec);
    if (ec) {
        RCLCPP_WARN(this->get_logger(), "Serial read error: %s", ec.message().c_str());
        return {};
    }
    total_read += n;
  }

  if (total_read != remain_len) {
    RCLCPP_WARN(this->get_logger(), "Timeout or incomplete data payload");
    return {};
  }

  std::vector<uint8_t> full_buf = expected_header;
  full_buf.insert(full_buf.end(), remain_buf.begin(), remain_buf.end());

  return full_buf;
}

bool AGV_PRO::is_power_on(){
  auto power_query_frame = build_serial_frame(GET_POWER_STATE, {});
  send_serial_frame(power_query_frame,true);

  const std::vector<uint8_t> expected_header = {0xFE, 0xFE, 0x0B, 0x12};
  auto power_query_response = read_serial_response(expected_header, 8, 12.0);

  print_hex("recv_buf", power_query_response);
  
  if (power_query_response.size() != 14) return false;

  uint16_t received_crc = (power_query_response[12] << 8) | power_query_response[13];
  uint16_t computed_crc = crc16_ibm(power_query_response.data(), 12);
  if (received_crc != computed_crc) {
    RCLCPP_WARN(this->get_logger(), "CRC mismatch: received=0x%04X, expected=0x%04X", received_crc, computed_crc);
    return false;
  }

  int is_poweron_status = static_cast<int8_t>(power_query_response[4]);
  RCLCPP_INFO(this->get_logger(), "is_poweron_status: %d", is_poweron_status);

  if (is_poweron_status == 0){
    auto status_query_frame = build_serial_frame(POWER_ON, {});
    send_serial_frame(status_query_frame,true);

    rclcpp::sleep_for(std::chrono::milliseconds(1000));// Sleep for 1000 milliseconds to allow the device enough time to process the previous command

    const std::vector<uint8_t> expected_header = {0xFE, 0xFE, 0x0B, 0x10};
    auto status_query_response = read_serial_response(expected_header, 8, 5.0);// Read the serial response with the specified expected header, payload size, and timeout of 5 seconds
    print_hex("recv_buf", status_query_response);
  
    if (status_query_response.size() != 14) return false;

    uint16_t received_crc = (status_query_response[12] << 8) | status_query_response[13];
    uint16_t computed_crc = crc16_ibm(status_query_response.data(), 12);
    if (received_crc != computed_crc) {
      RCLCPP_WARN(this->get_logger(), "CRC mismatch: received=0x%04X, expected=0x%04X", received_crc, computed_crc);
      return false;
    }

    int poweron_status = static_cast<int8_t>(status_query_response[4]);
    std::string status_msg;

    switch (poweron_status) {
      case 1:
        status_msg = "Motor is operating normally.";
        RCLCPP_INFO(this->get_logger(), "power_status: %d, %s", poweron_status, status_msg.c_str());
        return true;
      case 2:
        status_msg = "Emergency stop button is not released.";
        RCLCPP_ERROR(this->get_logger(), "power_status: %d, %s", poweron_status, status_msg.c_str());
        return false;
      case 3:
        status_msg = "Battery voltage is below 19.5V.";
        RCLCPP_ERROR(this->get_logger(), "power_status: %d, %s", poweron_status, status_msg.c_str());
        return false;
      case 4:
        status_msg = "CAN initialization error.";
        RCLCPP_ERROR(this->get_logger(), "power_status: %d, %s", poweron_status, status_msg.c_str());
        return false;
      case 5:
        status_msg = "Motor initialization error.";
        RCLCPP_ERROR(this->get_logger(), "power_status: %d, %s", poweron_status, status_msg.c_str());
        return false;
      default:
        RCLCPP_WARN(this->get_logger(), "power_status: %d, Unknown power status code", poweron_status);
        return false;
    }
  }
  else{
    RCLCPP_INFO(this->get_logger(), "Motor is operating normally.");
    return true;
  }
}

void AGV_PRO::set_auto_report(bool enable){
  auto frame = build_serial_frame(0x23, {static_cast<uint8_t>(enable)});
  send_serial_frame(frame,true);
}

void AGV_PRO::clearSerialBuffer(int fd) {
  if (tcflush(fd, TCIOFLUSH) < 0) {
    RCLCPP_WARN(this->get_logger(), "Failed to flush serial buffer: %s", std::strerror(errno));
  } else {
    RCLCPP_INFO(this->get_logger(), "Serial buffer flushed.");
  }
}

void AGV_PRO::disableDTR_RTS(int fd) {
  int status;
  if (::ioctl(fd, TIOCMGET, &status) == 0) {
    status &= ~(TIOCM_DTR | TIOCM_RTS);
    if (::ioctl(fd, TIOCMSET, &status) != 0) {
      RCLCPP_WARN(this->get_logger(), "Failed to clear DTR and RTS: %s", std::strerror(errno));
    } else {
      RCLCPP_INFO(this->get_logger(), "DTR and RTS lines disabled successfully.");
    }
  } else {
    RCLCPP_WARN(this->get_logger(), "Failed to read modem status: %s", std::strerror(errno));
  }
}

void AGV_PRO::cmdCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  linearX = std::clamp(msg->linear.x, -1.5, 1.5);
  linearY = std::clamp(msg->linear.y, -1.0, 1.0);
  angularZ = std::clamp(msg->angular.z, -1.0, 1.0);

  int16_t x_send = static_cast<int16_t>(linearX * 100);
  int16_t y_send = static_cast<int16_t>(linearY * 100);
  int16_t rot_send = static_cast<int16_t>(angularZ * 100);

  uint8_t buf[14] = { 0xfe,0xfe,0x0b,0x21 };

  buf[4] = (x_send >> 8) & 0xff;
  buf[5] = x_send & 0xff;
  buf[6] = (y_send >> 8) & 0xff;
  buf[7] = y_send & 0xff;
  buf[8] = (rot_send >> 8) & 0xff;
  buf[9] = rot_send & 0xff;
  buf[10] = 0x00;
  buf[11] = 0x00;

  uint16_t crc = crc16_ibm(buf, 12);
  buf[12] = (crc >> 8) & 0xff;
  buf[13] = crc & 0xff;

  std::vector<uint8_t> data_vec(buf, buf + sizeof(buf));

  try
  {
    boost::asio::write(*serial_port_,boost::asio::buffer(data_vec));
    // print_hex("Sent", data_vec);//debug
  }
  catch(const std::exception &ex)
  {
    RCLCPP_ERROR(this->get_logger(), "Error Transmiting from serial port:%s",ex.what());
  }
}

void AGV_PRO::handleSetDigitalOutput(
  const std::shared_ptr<agv_pro_msgs::srv::SetDigitalOutput::Request> request,
  std::shared_ptr<agv_pro_msgs::srv::SetDigitalOutput::Response> response)
{
  uint8_t output_number = request->pin;
  uint8_t output_state = request->state;

  if (output_number < 1 || output_number > 6){
    RCLCPP_ERROR(this->get_logger(), "Invalid output pin number: %u", output_number);
    response->success = false;
    response->message = "Invalid output pin number";
    return;
  }

  auto frame = build_serial_frame(SET_OUTPUT_IO, {output_number, output_state});
  send_serial_frame(frame, true);

  const std::vector<uint8_t> expected_header = {0xFE, 0xFE, 0x0B, SET_OUTPUT_IO};
  auto response_frame = read_serial_response(expected_header, 8, 5.0);

  // print_hex("recv_buf", response_frame); //debug

  uint8_t status = response_frame[4];
  if (status == 0x01) {
    RCLCPP_INFO(this->get_logger(), "SetDigitalOutput succeeded");
    response->success = true;
    response->message = "Success";
  } else {
    RCLCPP_ERROR(this->get_logger(), "SetDigitalOutput failed with status: 0x%02X", status);
    response->success = false;
    response->message = "Failed with status code";
  }
}

void AGV_PRO::handleGetDigitalInput(
  const std::shared_ptr<agv_pro_msgs::srv::GetDigitalInput::Request> request,
  std::shared_ptr<agv_pro_msgs::srv::GetDigitalInput::Response> response)
{
  uint8_t input_number = request->pin;

  if (input_number < 1 || input_number > 6){
    RCLCPP_ERROR(this->get_logger(), "Invalid input pin number: %u", input_number);
    response->success = false;
    response->message = "Invalid input pin number";
    return;
  }

  auto frame = build_serial_frame(GET_INPUT_IO, {input_number});
  send_serial_frame(frame, true);

  const std::vector<uint8_t> expected_header = {0xFE, 0xFE, 0x0B, GET_INPUT_IO};
  auto response_frame = read_serial_response(expected_header, 8, 5.0);
  // print_hex("recv_buf", response_frame); //debug

  uint8_t status = response_frame[5];
  if (status == 0xff) {
    RCLCPP_ERROR(this->get_logger(), "GetDigitalInput failed with status: 0x%02X", status);
    response->success = false;
  } else {
    RCLCPP_INFO(this->get_logger(), "GetDigitalInput succeeded, state: %u", status);
    response->state = static_cast<int32_t>(status);
    response->success = true;
    response->message = "Success";
  }
}

bool AGV_PRO::readData()
{
  std::vector<uint8_t> buf_length(1);
  std::vector<uint8_t> data_buf(RECEIVE_PAYLOAD_SIZE);

  uint8_t byte = 0;
  boost::system::error_code ec;
  
  while (true)
  {
    size_t ret = boost::asio::read(*serial_port_, boost::asio::buffer(&byte, 1), ec);
    if (ec) {
      RCLCPP_ERROR(this->get_logger(), "Serial read error: %s", ec.message().c_str());
      return false;
    }
    if (ret != 1 || byte != 0xfe) {
      continue;
    }

    ret = boost::asio::read(*serial_port_, boost::asio::buffer(&byte, 1), ec);
    if (ec) {
      RCLCPP_ERROR(this->get_logger(), "Serial read error: %s", ec.message().c_str());
      return false;
    }
    if (ret == 1 && byte == 0xfe) {
      break; 
    }
  }

  size_t ret = boost::asio::read(*serial_port_, boost::asio::buffer(buf_length), ec);
  if (ec) {
      RCLCPP_ERROR(this->get_logger(), "Serial read error: %s", ec.message().c_str());
      return false;
  }

  if (buf_length[0] != RECEIVE_PAYLOAD_SIZE) {
    //RCLCPP_ERROR(this->get_logger(), "The received length is incorrect:%u", buf_length[0]);
    return false;
  }

  ret = boost::asio::read(*serial_port_, boost::asio::buffer(data_buf), ec);
  if (ec || ret != data_buf.size())
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to receive full payload");
    return false;
  }

  std::vector<uint8_t> recv_buf;
  recv_buf.push_back(0xFE);
  recv_buf.push_back(0xFE);
  recv_buf.push_back(0x1C);
  recv_buf.insert(recv_buf.end(), data_buf.begin(), data_buf.end());
  
  // print_hex("recv_buf", recv_buf); //debug

  if (recv_buf[3] != 0x25) {
    //RCLCPP_WARN(this->get_logger(), "Command error:0x%02X", recv_buf[2]); //debug
    return false;
  }

  uint16_t received_crc = recv_buf[RECEIVE_FRAME_SIZE-1] | (recv_buf[RECEIVE_FRAME_SIZE-2] << 8);
  uint16_t computed_crc = crc16_ibm(recv_buf.data(), RECEIVE_FRAME_SIZE-2);

  if (received_crc != computed_crc) {
    RCLCPP_WARN(this->get_logger(), "CRC error: received 0x%04X, calculated 0x%04X", received_crc, computed_crc);
    return false;
  }

  vx = static_cast<double>(static_cast<int8_t>(recv_buf[4])) * 0.01;
  vy = static_cast<double>(static_cast<int8_t>(recv_buf[5])) * 0.01;
  vtheta = static_cast<double>(static_cast<int8_t>(recv_buf[6])) * 0.01;

  motor_status = recv_buf[7];
  motor_error  = recv_buf[8];
  battery_voltage = static_cast<float>(recv_buf[9]) / 10.0f;
  enable_status = recv_buf[10];

  imu_data.linear_acceleration.x = static_cast<double>(static_cast<int16_t>((recv_buf[11] << 8) | recv_buf[12])) * 0.01;
  imu_data.linear_acceleration.y = static_cast<double>(static_cast<int16_t>((recv_buf[13] << 8) | recv_buf[14])) * 0.01;
  imu_data.linear_acceleration.z = static_cast<double>(static_cast<int16_t>((recv_buf[15] << 8) | recv_buf[16])) * 0.01;

  imu_data.angular_velocity.x = static_cast<double>(static_cast<int16_t>((recv_buf[17] << 8) | recv_buf[18])) * 0.01;
  imu_data.angular_velocity.y = static_cast<double>(static_cast<int16_t>((recv_buf[19] << 8) | recv_buf[20])) * 0.01;
  imu_data.angular_velocity.z = static_cast<double>(static_cast<int16_t>((recv_buf[21] << 8) | recv_buf[22])) * 0.01;

  roll  = static_cast<double>(static_cast<int16_t>((recv_buf[23] << 8) | recv_buf[24])) * 0.01;
  pitch = static_cast<double>(static_cast<int16_t>((recv_buf[25] << 8) | recv_buf[26])) * 0.01;
  yaw   = static_cast<double>(static_cast<int16_t>((recv_buf[27] << 8) | recv_buf[28])) * 0.01;

  // RCLCPP_INFO(this->get_logger(),
  // "IMU Data - Accel[x: %.2f, y: %.2f, z: %.2f], "
  // "Gyro[x: %.2f, y: %.2f, z: %.2f], "
  // "RPY[roll: %.2f, pitch: %.2f, yaw: %.2f]",
  // imu_data.linear_acceleration.x,
  // imu_data.linear_acceleration.y,
  // imu_data.linear_acceleration.z,
  // imu_data.angular_velocity.x,
  // imu_data.angular_velocity.y,
  // imu_data.angular_velocity.z,
  // roll, pitch, yaw);

  return true;
}

void AGV_PRO::publisherVoltage()
{
  std_msgs::msg::Float32 voltage_msg,voltage_backup_msg;
  voltage_msg.data = battery_voltage;
  pub_voltage->publish(voltage_msg);
}

void AGV_PRO::publisherImuSensor()
{
  sensor_msgs::msg::Imu ImuSensor;

  ImuSensor.header.stamp = this->get_clock()->now();
  ImuSensor.header.frame_id = "imu_link";

  tf2::Quaternion qua;
  qua.setRPY(0, 0, yaw * M_PI / 180.0);

  ImuSensor.orientation.x = qua[0];
  ImuSensor.orientation.y = qua[1];
  ImuSensor.orientation.z = qua[2];
  ImuSensor.orientation.w = qua[3];

  ImuSensor.angular_velocity.x = imu_data.angular_velocity.x;
  ImuSensor.angular_velocity.y = imu_data.angular_velocity.y;
  ImuSensor.angular_velocity.z = imu_data.angular_velocity.z;

  ImuSensor.linear_acceleration.x = imu_data.linear_acceleration.x;
  ImuSensor.linear_acceleration.y = imu_data.linear_acceleration.y;
  ImuSensor.linear_acceleration.z = imu_data.linear_acceleration.z;

  ImuSensor.orientation_covariance[0] = 1e6;
  ImuSensor.orientation_covariance[4] = 1e6;
  ImuSensor.orientation_covariance[8] = 1e-6;

  ImuSensor.angular_velocity_covariance[0] = 1e6;
  ImuSensor.angular_velocity_covariance[4] = 1e6;
  ImuSensor.angular_velocity_covariance[8] = 1e-6;

  pub_imu->publish(ImuSensor);
}

void AGV_PRO::publisherOdom(double dt)
{
  currentTime = this->get_clock()->now();

  double delta_x = (vx * cos(theta) - vy * sin(theta)) * dt;
  double delta_y = (vx * sin(theta) + vy * cos(theta)) * dt;
  double delta_th = vtheta * dt;

  x += delta_x;
  y += delta_y;
  theta += delta_th;

  geometry_msgs::msg::TransformStamped odom_trans;
  odom_trans.header.stamp = currentTime;
  odom_trans.header.frame_id = frame_id_of_odometry_;
  odom_trans.child_frame_id = child_frame_id_of_odometry_;

  tf2::Quaternion quat;
  quat.setRPY(0.0, 0.0, theta);
  geometry_msgs::msg::Quaternion odom_quat = tf2::toMsg(quat);

  odom_trans.transform.translation.x = x; 
  odom_trans.transform.translation.y = y; 
  odom_trans.transform.translation.z = 0.0;
  odom_trans.transform.rotation = odom_quat;

  odomBroadcaster->sendTransform(odom_trans);

  nav_msgs::msg::Odometry odom;
  odom.header.stamp = currentTime;
  odom.header.frame_id = frame_id_of_odometry_;
  odom.child_frame_id = child_frame_id_of_odometry_;

  odom.pose.pose.position.x = x;
  odom.pose.pose.position.y = y;
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation = odom_quat;
  odom.pose.covariance = this->odom_pose_covariance;

  odom.twist.twist.linear.x = vx;
  odom.twist.twist.linear.y = vy;
  odom.twist.twist.angular.z = vtheta;
  odom.twist.covariance = this->odom_twist_covariance;

  pub_odom->publish(odom);
}

void AGV_PRO::Control()
{
  if (true == readData())
  {
    currentTime = this->get_clock()->now();
    double dt = 0.0;
    if (lastTime.nanoseconds() != 0) {
      dt = (currentTime - lastTime).seconds();
    }

    lastTime = currentTime;
    publisherOdom(dt);
    // RCLCPP_INFO(this->get_logger(), "dt:%f", dt);
    publisherVoltage();
    publisherImuSensor();
  }
}

AGV_PRO::AGV_PRO(std::string node_name):rclcpp::Node(node_name)
{
  this->declare_parameter<std::string>("port_name","/dev/agvpro_controller");
  this->declare_parameter<std::string>("odometry.frame_id", "odom");
  this->declare_parameter<std::string>("odometry.child_frame_id", "base_footprint");
  this->declare_parameter<std::string>("imu.frame_id", "imu_link");
  this->declare_parameter<std::string>("namespace", "");

  this->get_parameter_or<std::string>("port_name",device_name_,std::string("/dev/agvpro_controller"));
  this->get_parameter_or<std::string>("odometry.frame_id",frame_id_of_odometry_,std::string("odom"));
  this->get_parameter_or<std::string>("odometry.child_frame_id",child_frame_id_of_odometry_,std::string("base_footprint"));
  this->get_parameter_or<std::string>("imu.frame_id",frame_id_of_imu_,std::string("imu_link"));        
  this->get_parameter_or<std::string>("namespace",name_space_,std::string(""));

  if (name_space_ != "") {
    frame_id_of_odometry_ = name_space_ + "/" + frame_id_of_odometry_;
    child_frame_id_of_odometry_ = name_space_ + "/" + child_frame_id_of_odometry_;
    frame_id_of_imu_ = name_space_ + "/" + frame_id_of_imu_;
  }

  odomBroadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(this);
  pub_imu =  this->create_publisher<sensor_msgs::msg::Imu>("imu", 20);
  pub_odom = this->create_publisher<nav_msgs::msg::Odometry>("odom", 50);
  pub_voltage = create_publisher<std_msgs::msg::Float32>("voltage", 10);
  cmd_sub = this->create_subscription<geometry_msgs::msg::Twist>(
    "/cmd_vel", 10, std::bind(&AGV_PRO::cmdCallback, this, std::placeholders::_1));

  set_output_service = this->create_service<agv_pro_msgs::srv::SetDigitalOutput>(
    "set_digital_output",
    std::bind(&AGV_PRO::handleSetDigitalOutput, this, std::placeholders::_1, std::placeholders::_2)
  );

  get_input_service = this->create_service<agv_pro_msgs::srv::GetDigitalInput>(
    "get_digital_input",
    std::bind(&AGV_PRO::handleGetDigitalInput, this, std::placeholders::_1, std::placeholders::_2)
  );

  lastTime = this->get_clock()->now();
      
  try{
    serial_port_ = std::make_unique<boost::asio::serial_port>(io_);

    serial_port_->open(device_name_);
    serial_port_->set_option(boost::asio::serial_port_base::baud_rate(1000000));
    serial_port_->set_option(boost::asio::serial_port_base::character_size(8));
    serial_port_->set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
    serial_port_->set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
    serial_port_->set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));

    int fd = serial_port_->native_handle();
    this->clearSerialBuffer(fd);
    this->disableDTR_RTS(fd);

    rclcpp::sleep_for(std::chrono::milliseconds(3000));//esp32 Restart time

    RCLCPP_INFO(this->get_logger(), "Serial port initialized successfully");
    RCLCPP_INFO(this->get_logger(), "Using device: %s", device_name_.c_str());

    boost::asio::serial_port_base::baud_rate baud_option;
    serial_port_->get_option(baud_option);
    unsigned int current_baud = baud_option.value();
    RCLCPP_INFO(this->get_logger(), "Baud_rate: %u", current_baud);
  }
  catch (const std::exception &ex){
    RCLCPP_ERROR(this->get_logger(), "Failed to initialize serial port: %s", ex.what());
    return;
  }

  if (this->is_power_on()) {
    this->set_auto_report(1);

    control_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(20),
    std::bind(&AGV_PRO::Control, this)
    );
    RCLCPP_INFO(this->get_logger(), "Control timer started");
  }
  else {
    RCLCPP_WARN(this->get_logger(), "Control timer not started.");
  }
}

AGV_PRO::~AGV_PRO()
{
  if (serial_port_ && serial_port_->is_open()) {
    this->set_auto_report(0);
    serial_port_->cancel();
    serial_port_->close();
  } 
}