#pragma once

#include <hardware_interface/system_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <chrono>

std::string rx_buffer_;
bool readLine(std::string & line);


rclcpp::Time last_cmd_time_;
bool first_write_ = true;

namespace my_bot_hardware
{

class BotSystem : public hardware_interface::SystemInterface
{
public:
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State &) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State &) override;

  hardware_interface::return_type read(
    const rclcpp::Time &, const rclcpp::Duration &) override;

  hardware_interface::return_type write(
    const rclcpp::Time &, const rclcpp::Duration &) override;

private:
  std::string port_name_ = "/dev/ttyUSB0";
  int baudrate_ = 115200;
  int serial_fd_ = -1;

  double pos_left_rad_ = 0.0;
  double pos_right_rad_ = 0.0;
  double vel_left_rad_s_ = 0.0;
  double vel_right_rad_s_ = 0.0;
  double cmd_left_rad_s_ = 0.0;
  double cmd_right_rad_s_ = 0.0;

  bool openSerial();
  void closeSerial();
  bool readRPM(double &l, double &r);
  void sendRPM(double l, double r);
};

}
