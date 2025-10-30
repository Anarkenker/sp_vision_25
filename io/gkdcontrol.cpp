#include "gkdcontrol.hpp"

#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#include <thread>

#include "tools/math_tools.hpp"
#include "tools/yaml.hpp"

namespace io
{
GKDControl::GKDControl(const std::string & config_path)
: mode(GKDMode::idle),
  shoot_mode(GKDShootMode::left_shoot),
  bullet_speed(0),
  queue_(5000)
{
  std::string ip_port_config = read_yaml(config_path);
  
  size_t pos1 = ip_port_config.find(':');
  size_t pos2 = ip_port_config.find(':', pos1 + 1);
  size_t pos3 = ip_port_config.find(':', pos2 + 1);
  
  target_ip_ = ip_port_config.substr(0, pos1);
  quaternion_udp_port_ = std::stoi(ip_port_config.substr(pos1 + 1, pos2 - pos1 - 1));
  bullet_speed_udp_port_ = std::stoi(ip_port_config.substr(pos2 + 1, pos3 - pos2 - 1));
  send_udp_port_ = std::stoi(ip_port_config.substr(pos3 + 1));

  socket_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
  if (socket_fd_ < 0) {
    tools::logger()->error("[GKDControl] Cannot create socket");
    throw std::runtime_error("Cannot create socket for GKDControl");
  }

  std::thread receive_thread([this]() {
    sockaddr_in server_addr;
    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = INADDR_ANY;
    server_addr.sin_port = htons(quaternion_udp_port_);
    
    if (bind(socket_fd_, (sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
      tools::logger()->error("[GKDControl] Cannot bind socket");
      return;
    }
    
    char buffer[256];
    sockaddr_in client_addr;
    socklen_t client_len = sizeof(client_addr);
    
    tools::logger()->info("[GKDControl] Waiting for IMU data...");
    
    while (true) {
      ssize_t n = recvfrom(socket_fd_, buffer, sizeof(buffer), MSG_WAITALL, 
                           (sockaddr*)&client_addr, &client_len);
      if (n > 0) {
        callback_imu_data(buffer, n);
      }
    }
  });
  
  receive_thread.detach();  // 分离线程，让其在后台运行

  // 等待初始数据
  tools::logger()->info("[GKDControl] Waiting for initial IMU data...");
  queue_.pop(data_ahead_);
  queue_.pop(data_behind_);
  tools::logger()->info("[GKDControl] GKDControl initialized.");
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
  auto t_c = timestamp;
  std::chrono::duration<double> t_ab = t_b - t_a;
  std::chrono::duration<double> t_ac = t_c - t_a;

  // 四元数插值
  auto k = t_ac / t_ab;
  Eigen::Quaterniond q_c = q_a.slerp(k, q_b).normalized();

  return q_c;
}

void GKDControl::send(Command command) const
{
  // 创建UDP套接字用于发送
  int send_socket = socket(AF_INET, SOCK_DGRAM, 0);
  if (send_socket < 0) {
    tools::logger()->warn("[GKDControl] Cannot create send socket");
    return;
  }

  // 准备发送数据结构
  struct GKDCommand {
    uint8_t header = 0xAA;
    float yaw = 0;
    float pitch = 0;
    uint8_t control = 0;
    uint8_t shoot = 0;
    float distance = 0;
  } cmd;

  cmd.yaw = command.yaw;
  cmd.pitch = command.pitch;
  cmd.control = command.control ? 1 : 0;
  cmd.shoot = command.shoot ? 1 : 0;
  cmd.distance = command.horizon_distance;

  // 设置目标地址
  sockaddr_in dest_addr;
  memset(&dest_addr, 0, sizeof(dest_addr));
  dest_addr.sin_family = AF_INET;
  dest_addr.sin_port = htons(send_udp_port_);
  inet_pton(AF_INET, target_ip_.c_str(), &dest_addr.sin_addr);

  // 发送命令
  ssize_t sent = sendto(send_socket, &cmd, sizeof(cmd), 0, 
                        (sockaddr*)&dest_addr, sizeof(dest_addr));

  if (sent < 0) {
    tools::logger()->warn("[GKDControl] Failed to send command via UDP");
  }

  close(send_socket);
}


void GKDControl::callback_imu_data(const char* data, size_t length)
{
  // 假设IMU数据格式为: header(1 byte) + qw(4 bytes) + qx(4 bytes) + qy(4 bytes) + qz(4 bytes)
  if (length >= sizeof(uint8_t) + 4 * sizeof(float)) {
    uint8_t header = *(uint8_t*)data;
    
    if (header == 0x01) { // 假设IMU数据的头部标识
      float qw = *((float*)(data + 1));
      float qx = *((float*)(data + 5));
      float qy = *((float*)(data + 9));
      float qz = *((float*)(data + 13));

      // 验证四元数长度是否接近单位长度
      float norm = qw*qw + qx*qx + qy*qy + qz*qz;
      if (std::abs(norm - 1.0f) < 1e-2f) {
        Eigen::Quaterniond q(qw, qx, qy, qz);
        auto timestamp = std::chrono::steady_clock::now();
        queue_.push({q, timestamp});
      } else {
        tools::logger()->warn("[GKDControl] Invalid quaternion received: ({}, {}, {}, {}), norm = {}", 
                             qw, qx, qy, qz, norm);
      }
    }
  }
}

std::string GKDControl::read_yaml(const std::string & config_path)
{
  auto yaml = tools::load(config_path);

  if (!yaml["gkd_udp_target"]) {
    throw std::runtime_error("Missing 'gkd_udp_target' in YAML configuration. Expected format: 'ip:quat_port:bullet_port:send_port'");
  }

  return yaml["gkd_udp_target"].as<std::string>();
}

}  // namespace io