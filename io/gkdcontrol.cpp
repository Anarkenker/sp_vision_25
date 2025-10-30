#include "gkdcontrol.hpp"

#include <thread>

#include "io/gkdcontrol/send_control.hpp"
#include "tools/math_tools.hpp"
#include "tools/yaml.hpp"

namespace
{
constexpr int kDefaultQuaternionUdpPort = 11453;
constexpr int kDefaultBulletSpeedUdpPort = 11454;
constexpr int kDefaultSendUdpPort = 11451;
constexpr double kAngleChangeEpsilon = 1e-5;
}  // namespace

namespace io
{
GKDControl::GKDControl(const std::string & config_path)
: mode(GKDMode::idle),
  shoot_mode(GKDShootMode::left_shoot),
  bullet_speed(0.0),
  ft_angle(0.0),
  queue_(5000)
{
  target_ip_ = read_yaml(config_path);

  initialize_udp_transmission();
  initialize_udp_reception();

  tools::logger()->info("[GKDControl] Waiting for IMU samples...");
  queue_.pop(data_ahead_);
  queue_.pop(data_behind_);
  tools::logger()->info("[GKDControl] Ready.");
}

Eigen::Quaterniond GKDControl::imu_at(std::chrono::steady_clock::time_point timestamp)
{
  if (data_behind_.timestamp < timestamp) data_ahead_ = data_behind_;

  while (true) {
    queue_.pop(data_behind_);
    if (data_behind_.timestamp > timestamp) break;
    data_ahead_ = data_behind_;
  }

  Eigen::Quaterniond q_a = data_ahead_.q.normalized();
  Eigen::Quaterniond q_b = data_behind_.q.normalized();
  auto t_a = data_ahead_.timestamp;
  auto t_b = data_behind_.timestamp;
  std::chrono::duration<double> t_ab = t_b - t_a;
  std::chrono::duration<double> t_ac = timestamp - t_a;

  double k = t_ac / t_ab;
  return q_a.slerp(k, q_b).normalized();
}

void GKDControl::send(Command command) const
{
  send_control(command.yaw, command.pitch, command.shoot);
}

void GKDControl::initialize_udp_reception()
{
  std::thread(&IO::Server_socket_interface::task, &socket_interface_).detach();

  std::thread([this]() {
    ReceiveGimbalInfo last_pkg{};
    bool has_last = false;

    while (true) {
      ReceiveGimbalInfo current = socket_interface_.pkg;

      if (current.header == 0) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        continue;
      }

      bool changed = !has_last ||
                     std::abs(current.yaw - last_pkg.yaw) > kAngleChangeEpsilon ||
                     std::abs(current.pitch - last_pkg.pitch) > kAngleChangeEpsilon;

      if (changed) {
        Eigen::Vector3d euler(current.yaw, current.pitch, 0.0);
        Eigen::Quaterniond q(tools::rotation_matrix(euler));
        queue_.push({q.normalized(), std::chrono::steady_clock::now()});

        last_pkg = current;
        has_last = true;
      }

      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }).detach();
}

void GKDControl::initialize_udp_transmission()
{
  init_send(target_ip_);

  if (send_udp_port_ != kDefaultSendUdpPort) {
    tools::logger()->warn(
      "[GKDControl] Config send_udp_port {} ignored; current sender uses {}.", send_udp_port_,
      kDefaultSendUdpPort);
  }
}

std::string GKDControl::read_yaml(const std::string & config_path)
{
  auto yaml = tools::load(config_path);

  quaternion_udp_port_ = kDefaultQuaternionUdpPort;
  bullet_speed_udp_port_ = kDefaultBulletSpeedUdpPort;
  send_udp_port_ = kDefaultSendUdpPort;
  std::string target_ip = "127.0.0.1";

  if (!yaml["gkdcontrol"]) {
    tools::logger()->warn(
      "[GKDControl] Missing 'gkdcontrol' section in {}, fallback to defaults.", config_path);
    return target_ip;
  }

  auto node = yaml["gkdcontrol"];

  if (node["target_ip"]) target_ip = node["target_ip"].as<std::string>();
  if (node["quaternion_udp_port"]) quaternion_udp_port_ = node["quaternion_udp_port"].as<int>();
  if (node["bullet_speed_udp_port"]) bullet_speed_udp_port_ = node["bullet_speed_udp_port"].as<int>();
  if (node["send_udp_port"]) send_udp_port_ = node["send_udp_port"].as<int>();

  if (quaternion_udp_port_ != kDefaultQuaternionUdpPort) {
    tools::logger()->warn(
      "[GKDControl] Config quaternion_udp_port {} ignored; socket interface binds to {}.",
      quaternion_udp_port_, kDefaultQuaternionUdpPort);
  }

  if (bullet_speed_udp_port_ != kDefaultBulletSpeedUdpPort) {
    tools::logger()->warn(
      "[GKDControl] Config bullet_speed_udp_port {} currently unused; default {} expected.",
      bullet_speed_udp_port_, kDefaultBulletSpeedUdpPort);
  }

  return target_ip;
}

}  // namespace io

