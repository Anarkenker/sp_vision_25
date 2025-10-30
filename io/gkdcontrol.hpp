#ifndef IO__GKDCONTROL_HPP
#define IO__GKDCONTROL_HPP

#include <Eigen/Geometry>
#include <chrono>
#include <cmath>
#include <functional>
#include <string>
#include <vector>

#include "io/command.hpp"
#include "io/gkdcontrol/socket_interface.hpp"
#include "tools/logger.hpp"
#include "tools/thread_safe_queue.hpp"

namespace io
{
enum GKDMode
{
  idle,
  auto_aim,
  small_buff,
  big_buff,
  outpost
};
const std::vector<std::string> MODES = {"idle", "auto_aim", "small_buff", "big_buff", "outpost"};

// 哨兵专有
enum GKDShootMode
{
  left_shoot,
  right_shoot,
  both_shoot
};
const std::vector<std::string> SHOOTMODES = {"left_shoot", "right_shoot", "both_shoot"};

class GKDControl
{
public:
  double bullet_speed;
  GKDMode mode;
  GKDShootMode shoot_mode;
  double ft_angle;  //无人机专有

  GKDControl(const std::string & config_path);

  Eigen::Quaterniond imu_at(std::chrono::steady_clock::time_point timestamp);

  void send(Command command) const;

private:
  struct IMUData
  {
    Eigen::Quaterniond q;
    std::chrono::steady_clock::time_point timestamp;
  };

  tools::ThreadSafeQueue<IMUData> queue_;  // 必须在socket_之前初始化
  IMUData data_ahead_;
  IMUData data_behind_;

  IO::Server_socket_interface socket_interface_;

  int quaternion_udp_port_, bullet_speed_udp_port_, send_udp_port_;
  std::string target_ip_;

  void initialize_udp_reception();
  void initialize_udp_transmission();

  std::string read_yaml(const std::string & config_path);
};

}  // namespace io

#endif  // IO__GKDCONTROL_HPP