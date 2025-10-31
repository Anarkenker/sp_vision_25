#include <fmt/core.h>

#include <chrono>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>

#include "io/camera.hpp"
#include "io/gkdcontrol.hpp"
#include "tasks/auto_aim/aimer.hpp"
#include "tasks/auto_aim/multithread/commandgener.hpp"
#include "tasks/auto_aim/shooter.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"
#include "tools/recorder.hpp"

using namespace std::chrono;

const std::string keys =
  "{help h usage ? |      | 输出命令行参数说明}"
  "{visualize v    | true | 是否显示调试画面}"
  "{plot p         | true | 是否发送Plotter数据}"
  "{record r       | false | 是否录制识别结果}"
  "{@config-path   | configs/standard3.yaml | 位置参数，yaml配置文件路径 }";

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  auto config_path = cli.get<std::string>(0);
  if (cli.has("help") || config_path.empty()) {
    cli.printMessage();
    return 0;
  }
  const bool enable_visualize = cli.get<bool>("visualize");
  const bool enable_plot = cli.get<bool>("plot");
  const bool enable_record = cli.get<bool>("record");

  tools::Exiter exiter;
  tools::Plotter plotter;
  tools::Recorder recorder;

  io::GKDControl gkdcontrol(config_path);
  io::Camera camera(config_path);

  auto_aim::YOLO detector(config_path, false);
  auto_aim::Solver solver(config_path);
  auto_aim::Tracker tracker(config_path, solver);
  auto_aim::Aimer aimer(config_path);
  auto_aim::Shooter shooter(config_path);

  cv::Mat img;
  Eigen::Quaterniond q;
  std::chrono::steady_clock::time_point t;

  auto mode = io::GKDMode::idle;
  auto last_mode = io::GKDMode::idle;

  while (!exiter.exit()) {
    camera.read(img, t);
    q = gkdcontrol.imu_at(t - 1ms);
    mode = gkdcontrol.mode;

    if (last_mode != mode) {
      tools::logger()->info(
        "[GKDControl] Switch to {}", io::GKD_MODE_NAMES[static_cast<int>(mode)]);
      last_mode = mode;
    }

    solver.set_R_gimbal2world(q);

    Eigen::Vector3d ypr = tools::eulers(solver.R_gimbal2world(), 2, 1, 0);

    if (enable_record) recorder.record(img, q, t);

    auto armors = detector.detect(img);
    std::list<auto_aim::Armor> debug_armors;
    if (enable_visualize || enable_plot) debug_armors = armors;
    auto targets = tracker.track(armors, t);

    auto command = aimer.aim(targets, t, gkdcontrol.bullet_speed);

    gkdcontrol.send(command);

    if (enable_plot || enable_visualize) {
      nlohmann::json data;
      data["armor_num"] = debug_armors.size();
      data["tracker_state"] = tracker.state();
      data["bullet_speed"] = gkdcontrol.bullet_speed;
      data["gimbal_yaw"] = ypr[0] * 57.3;
      data["gimbal_pitch"] = ypr[1] * 57.3;
      if (command.control) {
        data["cmd_yaw"] = command.yaw * 57.3;
        data["cmd_pitch"] = command.pitch * 57.3;
        data["cmd_shoot"] = command.shoot ? 1 : 0;
      }

      if (!debug_armors.empty()) {
        auto armor = debug_armors.front();
        solver.solve(armor);
        data["armor_x"] = armor.xyz_in_world[0];
        data["armor_y"] = armor.xyz_in_world[1];
        data["armor_z"] = armor.xyz_in_world[2];
        data["armor_yaw"] = armor.ypr_in_world[0] * 57.3;
        data["armor_pitch"] = armor.ypr_in_world[1] * 57.3;
        data["armor_name"] = auto_aim::ARMOR_NAMES[armor.name];
      }

      if (!targets.empty()) {
        const auto & target = targets.front();
        Eigen::VectorXd x = target.ekf_x();
        data["target_x"] = x[0];
        data["target_vx"] = x[1];
        data["target_y"] = x[2];
        data["target_vy"] = x[3];
        data["target_z"] = x[4];
        data["target_vz"] = x[5];
        data["target_yaw"] = x[6] * 57.3;
        data["target_w"] = x[7];
        data["target_radius"] = x[8];
        data["target_length"] = x[9];
        data["target_height"] = x[10];
        data["target_last_id"] = target.last_id;
      }

      if (enable_plot) plotter.plot(data);

      if (enable_visualize) {
        cv::Mat display = img.clone();
        tools::draw_text(
          display, fmt::format("[{}]", tracker.state()), {10, 30}, {255, 255, 255}, 0.8, 2);

        for (auto & armor : debug_armors) {
          const cv::Scalar color =
            (armor.color == auto_aim::Color::blue) ? cv::Scalar(255, 0, 0) : cv::Scalar(0, 0, 255);
          tools::draw_points(display, armor.points, color, 2);
          cv::rectangle(display, armor.box, color, 2);
          tools::draw_text(
            display,
            fmt::format("{} {}", auto_aim::COLORS[armor.color], auto_aim::ARMOR_NAMES[armor.name]),
            armor.box.tl() + cv::Point(0, -5), color, 0.6, 2);
        }

        if (!targets.empty()) {
          const auto & target = targets.front();
          const auto aim_point = aimer.debug_aim_point;
          for (const Eigen::Vector4d & xyza : target.armor_xyza_list()) {
            auto image_points = solver.reproject_armor(
              xyza.head<3>(), xyza[3], target.armor_type, target.name);
            tools::draw_points(display, image_points, {0, 255, 0}, 2);
          }
          if (aim_point.valid) {
            auto aim_points = solver.reproject_armor(
              aim_point.xyza.head<3>(), aim_point.xyza[3], target.armor_type, target.name);
            tools::draw_points(display, aim_points, {0, 0, 255}, 2);
          }
        }

        cv::imshow("gkdinfantry", display);
        auto key = cv::waitKey(1);
        if (key == 'q' || key == 'Q' || key == 27) break;
      }
    }
  }

  return 0;
}
