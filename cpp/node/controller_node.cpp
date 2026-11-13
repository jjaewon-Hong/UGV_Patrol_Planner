// L4 PatrolControllerNode in C++ (rclcpp): the ROS 2 interface around MissionManager (U5).
// Same topics, message types, QoS and status JSON as ugv_patrol_planner/nodes/controller_node.py,
// so it is a drop-in replacement (launch argument controller:=cpp).
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cmath>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <memory>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>

#include "config.hpp"
#include "ugv_core/mission.hpp"

using geometry_msgs::msg::Point;
using geometry_msgs::msg::PoseArray;
using std_msgs::msg::Float32;
using StringMsg = std_msgs::msg::String;

namespace {
ugv::Cell to_cell(const Point& p) {  // x = row, y = col
  return {static_cast<int>(std::lround(p.x)), static_cast<int>(std::lround(p.y))};
}
Point to_point(ugv::Cell c) {
  Point p;
  p.x = c.r;
  p.y = c.c;
  return p;
}
}  // namespace

class PatrolControllerNode : public rclcpp::Node {
 public:
  PatrolControllerNode() : Node("patrol_controller") {
    const std::string share = ament_index_cpp::get_package_share_directory("ugv_patrol_planner");
    auto resolve = [&](const std::string& p) { return !p.empty() && p[0] == '/' ? p : share + "/" + p; };
    const std::string config = declare_parameter("config", share + "/config/params.yaml");
    (void)declare_parameter("scenario", share + "/scenarios/default.yaml");  // used by the simulators only
    cfg_ = ugv::load_config(resolve(config));
    terrain_ = std::make_unique<ugv::TerrainMap>(ugv::load_terrain(cfg_, share));
    planner_ = std::make_unique<ugv::AStarPlanner>(*terrain_);
    energy_ = std::make_unique<ugv::ReturnEnergyMap>(*terrain_, cfg_.base_cell, cfg_.energy);
    mission_ = std::make_unique<ugv::MissionManager>(*terrain_, *planner_, sched_, *energy_);

    // Latched (transient local): late subscribers still get the current path and status.
    const auto latched = rclcpp::QoS(1).reliable().transient_local();
    pub_next_ = create_publisher<Point>("/controller/next_cell", 10);
    pub_path_ = create_publisher<nav_msgs::msg::Path>("/controller/path", latched);
    pub_status_ = create_publisher<StringMsg>("/mission/status", latched);
    pub_alarm_ = create_publisher<StringMsg>("/mission/alarm", 10);
    subs_.push_back(create_subscription<StringMsg>("/operator/command", 10,
                                                   [this](const StringMsg& m) { on_command(m); }));
    subs_.push_back(create_subscription<Point>("/ugv/pose", 10, [this](const Point& m) { on_pose(m); }));
    subs_.push_back(create_subscription<PoseArray>("/perception/obstacles", 10,
                                                   [this](const PoseArray& m) { on_obstacles(m); }));
    subs_.push_back(create_subscription<Float32>("/battery/level", 10, [this](const Float32& m) {
      publish(mission_->on_battery(m.data));
    }));
    publish({mission_->status()});
    RCLCPP_INFO(get_logger(), "[C++] map %dx%d loaded, base (%d, %d)", terrain_->size(), terrain_->size(),
                cfg_.base_cell.r, cfg_.base_cell.c);
  }

 private:
  double now_s() { return now().seconds(); }

  void on_command(const StringMsg& msg) {
    std::string error;
    const auto cmd = ugv::parse_command(msg.data, &error);
    if (!cmd) {  // malformed text: alarm, keep running (NFR-04)
      publish({ugv::Alarm{error}});
      return;
    }
    publish(mission_->on_command(*cmd, now_s()));
  }

  void on_pose(const Point& msg) {
    const ugv::Cell c = to_cell(msg);
    if (!terrain_->in_bounds(c)) {
      publish({ugv::Alarm{"지도 밖 위치: " + ugv::format_cell(c)}});
      return;
    }
    publish(mission_->on_pose(c, now_s()));
  }

  void on_obstacles(const PoseArray& msg) {
    std::vector<ugv::Cell> cells;
    std::string outside;
    for (const auto& p : msg.poses) {
      cells.push_back(to_cell(p.position));
      if (!terrain_->in_bounds(cells.back())) outside += (outside.empty() ? "" : ", ") + ugv::format_cell(cells.back());
    }
    if (!outside.empty()) publish({ugv::Alarm{"지도 밖 장애물 무시: [" + outside + "]"}});
    const auto outs = mission_->on_obstacles(cells, now_s());
    for (const auto& o : outs) {
      const auto* s = std::get_if<ugv::Status>(&o);
      if (s && s->state == ugv::MissionState::Replanning) {  // NFR-01 monitoring
        const double ms = planner_->last_plan_ms(), limit = cfg_.replan_time_limit_s * 1000;
        if (ms > limit)
          RCLCPP_WARN(get_logger(), "re-planned in %.2f ms (limit %.0f ms)", ms, limit);
        else
          RCLCPP_INFO(get_logger(), "re-planned in %.2f ms (limit %.0f ms)", ms, limit);
        break;
      }
    }
    publish(outs);
  }

  void publish(const ugv::Outputs& outputs) {
    for (const auto& o : outputs) {
      if (const auto* n = std::get_if<ugv::NextCell>(&o)) {
        pub_next_->publish(to_point(n->cell));
      } else if (const auto* p = std::get_if<ugv::PathUpdate>(&o)) {
        nav_msgs::msg::Path msg;
        msg.header.frame_id = "grid";
        msg.header.stamp = now();
        for (const auto& c : p->path) {
          geometry_msgs::msg::PoseStamped ps;
          ps.header.frame_id = "grid";
          ps.pose.position = to_point(c);
          msg.poses.push_back(ps);
        }
        pub_path_->publish(msg);
        const auto& t = mission_->target();
        RCLCPP_INFO(get_logger(), "경로 갱신: %s -> %s (%s, %zu칸)", ugv::format_cell(p->path.front()).c_str(),
                    ugv::format_cell(p->path.back()).c_str(), t ? t->pid.c_str() : "기지", p->path.size() - 1);
      } else if (const auto* s = std::get_if<ugv::Status>(&o)) {
        StringMsg msg;
        msg.data = ugv::status_json(*s);
        pub_status_->publish(msg);
        RCLCPP_INFO(get_logger(), "임무 상태: %s%s", ugv::state_name(s->state),
                    s->note.empty() ? "" : (" (" + s->note + ")").c_str());
      } else if (const auto* a = std::get_if<ugv::Alarm>(&o)) {
        StringMsg msg;
        msg.data = a->text;
        pub_alarm_->publish(msg);
        RCLCPP_WARN(get_logger(), "경보: %s", a->text.c_str());
      }
    }
  }

  ugv::Config cfg_;
  std::unique_ptr<ugv::TerrainMap> terrain_;
  std::unique_ptr<ugv::AStarPlanner> planner_;
  ugv::PatrolScheduler sched_;
  std::unique_ptr<ugv::ReturnEnergyMap> energy_;
  std::unique_ptr<ugv::MissionManager> mission_;
  rclcpp::Publisher<Point>::SharedPtr pub_next_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_;
  rclcpp::Publisher<StringMsg>::SharedPtr pub_status_, pub_alarm_;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subs_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<PatrolControllerNode>());
  } catch (const std::exception& e) {
    RCLCPP_FATAL(rclcpp::get_logger("patrol_controller"), "%s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
