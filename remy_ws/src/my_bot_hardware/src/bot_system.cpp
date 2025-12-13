#include "my_bot_hardware/bot_system.hpp"
#include <pluginlib/class_list_macros.hpp>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <sstream>
#include <cmath>
#include <cstdio>

PLUGINLIB_EXPORT_CLASS(
  my_bot_hardware::BotSystem,
  hardware_interface::SystemInterface)

namespace my_bot_hardware
{

hardware_interface::CallbackReturn BotSystem::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
    return CallbackReturn::ERROR;

  for (const auto & p : info.hardware_parameters) {
    if (p.first == "port") port_name_ = p.second;
    if (p.first == "baudrate") baudrate_ = std::stoi(p.second);
  }
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
BotSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> interfaces;
  interfaces.emplace_back("rev1", "position", &pos_left_rad_);
  interfaces.emplace_back("rev1", "velocity", &vel_left_rad_s_);
  interfaces.emplace_back("rev2", "position", &pos_right_rad_);
  interfaces.emplace_back("rev2", "velocity", &vel_right_rad_s_);
  return interfaces;
}

std::vector<hardware_interface::CommandInterface>
BotSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> interfaces;
  interfaces.emplace_back("rev1", "velocity", &cmd_left_rad_s_);
  interfaces.emplace_back("rev2", "velocity", &cmd_right_rad_s_);
  return interfaces;
}

hardware_interface::CallbackReturn BotSystem::on_activate(
  const rclcpp_lifecycle::State &)
{
  return openSerial() ? CallbackReturn::SUCCESS : CallbackReturn::ERROR;
}

hardware_interface::CallbackReturn BotSystem::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  closeSerial();
  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type BotSystem::read(
  const rclcpp::Time &, const rclcpp::Duration & period)
{
  double rpmL, rpmR;
  if (readRPM(rpmL, rpmR)) {
    vel_left_rad_s_ = rpmL * 2.0 * M_PI / 60.0;
    vel_right_rad_s_ = rpmR * 2.0 * M_PI / 60.0;
    pos_left_rad_ += vel_left_rad_s_ * period.seconds();
    pos_right_rad_ += vel_right_rad_s_ * period.seconds();
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type BotSystem::write(
  const rclcpp::Time & time,
  const rclcpp::Duration &)
{
  // Inicialização
  if (first_write_) {
    last_cmd_time_ = time;
    first_write_ = false;
  }

  // Calcula tempo desde último comando
  double dt = (time - last_cmd_time_).seconds();

  // Converte rad/s → RPM
  double rpm_left  = cmd_left_rad_s_  * 60.0 / (2.0 * M_PI);
  double rpm_right = cmd_right_rad_s_ * 60.0 / (2.0 * M_PI);

  // 🚨 FAILSAFE: sem comando recente → PARA
  if (dt > 0.5) {  // 500 ms
    rpm_left  = 0.0;
    rpm_right = 0.0;
  } else {
    // comando válido → atualiza timestamp
    last_cmd_time_ = time;
  }

  sendRPM(rpm_left, rpm_right);
  return hardware_interface::return_type::OK;
}

/* ===== Serial ===== */

bool BotSystem::openSerial()
{
  serial_fd_ = ::open(port_name_.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
  if (serial_fd_ < 0) {
    perror("openSerial: open");
    return false;
  }

  termios tty{};
  if (tcgetattr(serial_fd_, &tty) != 0) {
    perror("openSerial: tcgetattr");
    ::close(serial_fd_);
    serial_fd_ = -1;
    return false;
  }

  // ---- RAW mode ----
  cfmakeraw(&tty);

  // ---- Baudrate ----
  speed_t spd = B115200;
  switch (baudrate_) {
    case 9600:   spd = B9600; break;
    case 19200:  spd = B19200; break;
    case 38400:  spd = B38400; break;
    case 57600:  spd = B57600; break;
    case 115200: spd = B115200; break;
    case 230400: spd = B230400; break;
    default:     spd = B115200; break;
  }
  cfsetispeed(&tty, spd);
  cfsetospeed(&tty, spd);

  // ---- 8N1, sem flow control ----
  tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;   // 8 bits
  tty.c_cflag |= (CLOCAL | CREAD);              // enable receiver
  tty.c_cflag &= ~(PARENB | PARODD);            // no parity
  tty.c_cflag &= ~CSTOPB;                       // 1 stop bit
  tty.c_cflag &= ~CRTSCTS;                      // no HW flow control

  // ---- Timeouts (read não bloqueia eternamente) ----
  tty.c_cc[VMIN]  = 0;
  tty.c_cc[VTIME] = 1; // 0.1s

  // Opcional: evita reset “por hangup”
  tty.c_cflag &= ~HUPCL;

  if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) {
    perror("openSerial: tcsetattr");
    ::close(serial_fd_);
    serial_fd_ = -1;
    return false;
  }

  tcflush(serial_fd_, TCIOFLUSH);

  // Dá tempo da ESP bootar (muito importante!)
  usleep(800000); // 0.8s

  return true;
}


void BotSystem::closeSerial()
{
  if (serial_fd_ >= 0) ::close(serial_fd_);
}

bool BotSystem::readRPM(double &l, double &r)
{
  char buf[64];
  int n = ::read(serial_fd_, buf, sizeof(buf));
  if (n <= 0) return false;

  // Acumula dados no buffer
  rx_buffer_.append(buf, n);

  // Procura fim de linha
  size_t pos = rx_buffer_.find('\n');
  if (pos == std::string::npos)
    return false;  // ainda não chegou linha completa

  // Extrai UMA linha
  std::string line = rx_buffer_.substr(0, pos);
  rx_buffer_.erase(0, pos + 1);

  // Espera exatamente o formato correto
  if (line.rfind("RPM,", 0) != 0)
    return false;

  // Parse seguro
  if (std::sscanf(line.c_str(), "RPM,%lf,%lf", &l, &r) != 2)
    return false;

  return true;
}


void BotSystem::sendRPM(double l, double r)
{
  std::ostringstream ss;
  ss << "VEL," << l << "," << r << "\n";
  ::write(serial_fd_, ss.str().c_str(), ss.str().size());
}

}  // namespace my_bot_hardware
