// armdoc_fast: the live /joint_states checks of armdoc in C++ (see include/armdoc_cpp/core.hpp for what is covered).
// Read-only: subscribes to <ns>/joint_states and <ns>/robot_description, prints findings every `period` seconds.
// Parameters: ns, period, once (seconds; print one report, exit 1 on ERROR), grace, stale, noise_rad, lag_max,
// arm_joints (string array; default: learned from <ns>/controller_manager/list_controllers, like the Python tool).
#include <tinyxml2.h>

#include <chrono>
#include <cstring>
#include <memory>

#include "armdoc_cpp/core.hpp"
#include "rclcpp/rclcpp.hpp"
#include "controller_manager_msgs/srv/list_controllers.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

static void parse_limits(const std::string & urdf, armdoc::State & s) {
  tinyxml2::XMLDocument doc;
  if (doc.Parse(urdf.c_str()) != tinyxml2::XML_SUCCESS) return;
  auto * robot = doc.FirstChildElement("robot");
  if (!robot) return;
  for (auto * j = robot->FirstChildElement("joint"); j; j = j->NextSiblingElement("joint")) {
    const char * type = j->Attribute("type");
    auto * lim = j->FirstChildElement("limit");
    if (!lim || !type || !std::strcmp(type, "fixed") || !std::strcmp(type, "floating") || !std::strcmp(type, "planar")) continue;
    auto g = [&](const char * k) -> std::optional<double> {
      const char * v = lim->Attribute(k);
      if (!v || !*v) return std::nullopt;
      return std::atof(v);
    };
    s.limits[j->Attribute("name")] = armdoc::Limit{g("lower"), g("upper"), g("velocity"), g("effort"), !std::strcmp(type, "continuous")};
  }
}

class ArmDocFast : public rclcpp::Node {
 public:
  ArmDocFast() : Node("armdoc_fast") {
    ns_ = declare_parameter<std::string>("ns", "");
    const double period = declare_parameter<double>("period", 2.0);
    once_ = declare_parameter<double>("once", 0.0);
    cfg_.grace = declare_parameter<double>("grace", cfg_.grace);
    cfg_.stale = declare_parameter<double>("stale", cfg_.stale);
    cfg_.noise_rad = declare_parameter<double>("noise_rad", cfg_.noise_rad);
    cfg_.lag_max = declare_parameter<double>("lag_max", cfg_.lag_max);
    state_.t0 = now_s();
    auto fixed = declare_parameter<std::vector<std::string>>("arm_joints", std::vector<std::string>{});
    if (!fixed.empty()) { state_.arm_joints = std::set<std::string>(fixed.begin(), fixed.end()); fixed_arm_ = true; }
    cm_client_ = create_client<controller_manager_msgs::srv::ListControllers>(ns_ + "/controller_manager/list_controllers");
    poll_timer_ = create_wall_timer(std::chrono::seconds(2), [this]() { poll_controllers(); });
    sub_js_ = create_subscription<sensor_msgs::msg::JointState>(ns_ + "/joint_states", rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::JointState::ConstSharedPtr m) {
          const double now = now_s();
          const double stamp = rclcpp::Time(m->header.stamp).seconds();
          // Like the Python tool: a gap over 1e4 s means the stamps use another clock (sim time vs wall): no verdict.
          std::optional<double> lag;
          if (stamp > 0) {
            const double d = now - stamp;
            if (std::fabs(d) < 1e4) lag = d;
          }
          armdoc::feed_joints(state_, now, m->name, {m->position.begin(), m->position.end()},
                              {m->velocity.begin(), m->velocity.end()}, {m->effort.begin(), m->effort.end()}, lag);
        });
    sub_urdf_ = create_subscription<std_msgs::msg::String>(ns_ + "/robot_description", rclcpp::QoS(1).transient_local().reliable(),
        [this](std_msgs::msg::String::ConstSharedPtr m) { parse_limits(m->data, state_); });
    timer_ = create_wall_timer(std::chrono::duration<double>(once_ > 0 ? once_ : period), [this]() { report(); });
  }

 private:
  double now_s() { return this->now().seconds(); }
  void poll_controllers() {
    if (fixed_arm_ || pending_ || !cm_client_->service_is_ready()) return;
    pending_ = true;
    cm_client_->async_send_request(std::make_shared<controller_manager_msgs::srv::ListControllers::Request>(),
        [this](rclcpp::Client<controller_manager_msgs::srv::ListControllers>::SharedFuture f) {
          pending_ = false;
          std::set<std::string> claimed;
          for (auto & c : f.get()->controller) {
            if (!armdoc::is_arm_controller(c.name, c.type)) continue;
            for (auto & i : c.claimed_interfaces) claimed.insert(i.substr(0, i.rfind('/')));
          }
          state_.arm_joints = armdoc::update_arm_joints(state_.arm_joints, claimed);
        });
  }
  void report() {
    auto f = armdoc::evaluate(state_, now_s(), cfg_);
    bool error = false;
    if (f.empty()) std::printf("OK    arm looks healthy\n");
    for (auto & x : f) {
      std::printf("%-5s [%s] %s\n", x.level.c_str(), x.code.c_str(), x.msg.c_str());
      error |= x.level == armdoc::ERROR;
    }
    std::fflush(stdout);
    if (once_ > 0) {
      rclcpp::shutdown();
      std::exit(error ? 1 : 0);
    }
  }
  std::string ns_;
  double once_ = 0;
  armdoc::Config cfg_;
  armdoc::State state_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_js_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_urdf_;
  rclcpp::TimerBase::SharedPtr timer_, poll_timer_;
  rclcpp::Client<controller_manager_msgs::srv::ListControllers>::SharedPtr cm_client_;
  bool fixed_arm_ = false, pending_ = false;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ArmDocFast>());
  rclcpp::shutdown();
  return 0;
}
