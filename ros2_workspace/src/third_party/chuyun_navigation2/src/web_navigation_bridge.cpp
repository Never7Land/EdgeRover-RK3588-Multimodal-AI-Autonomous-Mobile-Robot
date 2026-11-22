// JSON-lines adapter for the web/voice point-navigation API. stdout is protocol only.
// ROS callbacks run in one persistent executor; stdin blocks when there is no request.
#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <poll.h>
#include <unistd.h>

#include <nlohmann/json.hpp>
#include <action_msgs/msg/goal_status_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

using Json = nlohmann::json;
using Clock = std::chrono::steady_clock;
using Navigate = nav2_msgs::action::NavigateToPose;

class WebNavigationBridge : public rclcpp::Node
{
public:
  WebNavigationBridge(const Json & topics, const rclcpp::NodeOptions & options)
  : Node("ros2_web_navigation_bridge", options), topics_(topics)
  {
    goal_topic_ = topic("goal_pose", "/goal_pose");
    status_topic_ = topic("navigation_status", "/lite_navigation/status");
    goal_ = create_publisher<geometry_msgs::msg::PoseStamped>(goal_topic_, 5);
    lite_cancel_ = create_publisher<std_msgs::msg::Empty>(
      topic("cancel_navigation", "/lite_navigation/cancel"), 5);
    nav_cancel_ = create_publisher<std_msgs::msg::Empty>("/navigation/cancel", 5);
    subscribe_lite_status();
    pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      topic("localized_pose", "/amcl_pose"), 5, [this](geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg) {
        if (msg->header.frame_id != "map") {return;}
        const auto & p = msg->pose.pose.position;
        const auto & q = msg->pose.pose.orientation;
        const double yaw = yaw_of(q);
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(yaw)) {return;}
        std::lock_guard<std::mutex> lock(mutex_);
        pose_ = {{"x", p.x}, {"y", p.y}, {"yaw", yaw}};
        pose_at_ = Clock::now();
      });
    distance_sub_ = create_subscription<std_msgs::msg::Float32>(
      topic("navigate_feedback", "/lite_navigation/feedback"), 5, [this](std_msgs::msg::Float32::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        distance_ = std::isfinite(msg->data) ? Json(msg->data) : Json(nullptr);
      });
    nav_client_ = rclcpp_action::create_client<Navigate>(this, "/navigate_to_pose");
    buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    // /tf is remapped to the existing /web/tf snapshots by our local NodeOptions.
    // A depth-one best-effort reader never applies backpressure to localization.
    listener_ = std::make_unique<tf2_ros::TransformListener>(
      *buffer_, this, false, rclcpp::QoS(1).best_effort());
    nav_status_sub_ = create_subscription<action_msgs::msg::GoalStatusArray>(
      "/navigate_to_pose/_action/status", rclcpp::QoS(1).transient_local(),
      [this](action_msgs::msg::GoalStatusArray::ConstSharedPtr msg) {
        if (msg->status_list.empty()) {return;}
        const auto & newest = *std::max_element(msg->status_list.begin(), msg->status_list.end(),
          [](const auto & a, const auto & b) {
            return rclcpp::Time(a.goal_info.stamp) < rclcpp::Time(b.goal_info.stamp);
          });
        std::lock_guard<std::mutex> lock(mutex_);
        switch (newest.status) {
          case 1: nav_status_ = "PLANNING"; break;
          case 2: nav_status_ = "TRACKING"; break;
          case 3: nav_status_ = "CANCELING"; break;
          case 4: nav_status_ = "ARRIVED"; nav_distance_ = 0.0; break;
          case 5: nav_status_ = "CANCELED"; nav_distance_ = nullptr; break;
          case 6: nav_status_ = "PLAN_FAILED"; nav_distance_ = nullptr; break;
          default: nav_status_ = "UNKNOWN";
        }
      });
    nav_feedback_sub_ = create_subscription<Navigate::Impl::FeedbackMessage>(
      "/navigate_to_pose/_action/feedback", 5, [this](Navigate::Impl::FeedbackMessage::ConstSharedPtr msg) {
        const auto value = msg->feedback.distance_remaining;
        std::lock_guard<std::mutex> lock(mutex_);
        nav_distance_ = std::isfinite(value) ? Json(value) : Json(nullptr);
      });
  }

  Json execute(const std::string & method, const Json & params)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const std::string engine = params.value("engine", "lite");
    if (engine != "lite" && engine != "nav2") {throw std::runtime_error("未知导航方式");}
    const Json session = params.value("sessionId", Json(0));
    if (engine_ != engine || session_ != session) {
      engine_ = engine;
      session_ = session;
      pose_ = distance_ = nav_distance_ = nullptr;
      status_ = "UNKNOWN";
      nav_status_ = "LOCALIZED";
      buffer_->clear();
      status_sub_.reset();
      subscribe_lite_status();
    }
    const auto current = state();
    if (method == "state") {return current;}
    if (method == "cancel") {
      if (!current.at("connected").get<bool>()) {throw std::runtime_error("导航节点未连接");}
      (engine_ == "nav2" ? nav_cancel_ : lite_cancel_)->publish(std_msgs::msg::Empty());
      return {{"submitted", true}};
    }
    if (method != "go") {throw std::runtime_error("未知导航操作");}
    if (!current.at("ready").get<bool>()) {
      throw std::runtime_error("导航或定位尚未就绪，请在网页设置初始位置并等待定位更新");
    }
    if (params.value("frame_id", "") != "map") {throw std::runtime_error("目标坐标无效");}
    for (const char * key : {"x", "y", "yaw"}) {
      if (!params.contains(key) || !params[key].is_number() ||
        !std::isfinite(params[key].get<double>())) {throw std::runtime_error("目标坐标无效");}
    }
    geometry_msgs::msg::PoseStamped msg;
    msg.header.frame_id = "map";
    msg.header.stamp = now();
    msg.pose.position.x = params["x"].get<double>();
    msg.pose.position.y = params["y"].get<double>();
    const double yaw = params["yaw"].get<double>();
    msg.pose.orientation.z = std::sin(yaw / 2);
    msg.pose.orientation.w = std::cos(yaw / 2);
    distance_ = nav_distance_ = nullptr;
    status_ = nav_status_ = "GOAL_SUBMITTED";
    goal_->publish(msg);
    return {{"submitted", true}, {"arrived", false}};
  }

private:
  std::string topic(const char * key, const char * fallback) const
  {return topics_.value(key, std::string(fallback));}

  static double yaw_of(const geometry_msgs::msg::Quaternion & q)
  {return std::atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));}

  void subscribe_lite_status()
  {
    status_sub_ = create_subscription<std_msgs::msg::String>(
      status_topic_, rclcpp::QoS(1).transient_local(), [this](std_msgs::msg::String::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        status_ = msg->data;
      });
  }

  // Called with mutex_ held, on demand; no background status polling.
  Json state()
  {
    if (engine_ == "nav2") {
      const bool connected = nav_client_->action_server_is_ready();
      Json pose = nullptr;
      if (connected) {
        try {
          const auto t = buffer_->lookupTransform("map", "base_footprint", tf2::TimePointZero);
          const double age = (now() - rclcpp::Time(t.header.stamp)).seconds();
          const auto & p = t.transform.translation;
          const double yaw = yaw_of(t.transform.rotation);
          if (age >= -1.0 && age < 2.0 && std::isfinite(p.x) &&
            std::isfinite(p.y) && std::isfinite(yaw)) {
            pose = {{"x", p.x}, {"y", p.y}, {"yaw", yaw}};
          }
        } catch (const tf2::TransformException &) {}
      }
      return {{"connected", connected}, {"ready", connected && !pose.is_null()},
        {"status", connected ? (pose.is_null() ? "IDLE" : nav_status_) : "OFFLINE"},
        {"pose", pose}, {"distance_remaining", connected ? nav_distance_ : Json(nullptr)}};
    }
    bool connected = false;
    for (const auto & info : get_subscriptions_info_by_topic(goal_topic_)) {
      if (info.node_name() == "lite_navigation") {connected = true; break;}
    }
    const bool fresh = !pose_.is_null() &&
      std::chrono::duration<double>(Clock::now() - pose_at_).count() < 2.0;
    return {{"connected", connected},
      {"ready", connected && fresh && status_ != "UNKNOWN" && status_ != "IDLE"},
      {"status", connected ? status_ : "OFFLINE"},
      {"pose", connected && fresh ? pose_ : Json(nullptr)},
      {"distance_remaining", connected ? distance_ : Json(nullptr)}};
  }

  Json topics_, session_ = nullptr, pose_ = nullptr, distance_ = nullptr, nav_distance_ = nullptr;
  std::string goal_topic_, status_topic_, engine_ = "lite", status_ = "UNKNOWN", nav_status_ = "LOCALIZED";
  Clock::time_point pose_at_{};
  std::mutex mutex_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr lite_cancel_, nav_cancel_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr distance_sub_;
  rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr nav_status_sub_;
  rclcpp::Subscription<Navigate::Impl::FeedbackMessage>::SharedPtr nav_feedback_sub_;
  rclcpp_action::Client<Navigate>::SharedPtr nav_client_;
  std::unique_ptr<tf2_ros::Buffer> buffer_;
  std::unique_ptr<tf2_ros::TransformListener> listener_;
};

int main(int argc, char ** argv)
{
  // ROS logging must never enter the JSON-lines stdout protocol.
  setenv("RCUTILS_LOGGING_USE_STDOUT", "0", 1);
  rclcpp::init(argc, argv);
  try {
    std::ifstream config("ros_config.json");
    const Json topics = Json::parse(config).value("topics", Json::object());
    rclcpp::NodeOptions options;
    options.start_parameter_services(false);
    options.arguments({"--ros-args", "-r", "/tf:=" + topics.value("tf", std::string("/tf"))});
    auto node = std::make_shared<WebNavigationBridge>(topics, options);
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread spin([&executor]() {executor.spin();});
    std::string pending;
    while (rclcpp::ok()) {
      pollfd input{STDIN_FILENO, POLLIN, 0};
      // Check shutdown periodically, but do not poll ROS state or request queues.
      const int ready = poll(&input, 1, 1000);
      if (ready <= 0) {continue;}
      char bytes[4096];
      const auto count = read(STDIN_FILENO, bytes, sizeof(bytes));
      if (count <= 0) {break;}
      pending.append(bytes, static_cast<size_t>(count));
      if (pending.size() > 65536) {std::cerr << "Request exceeds 64 KiB" << std::endl; break;}
      size_t end;
      while ((end = pending.find('\n')) != std::string::npos) {
        const auto line = pending.substr(0, end);
        pending.erase(0, end + 1);
        Json response = {{"id", nullptr}};
        try {
          const auto request = Json::parse(line);
          response["id"] = request.value("id", Json(nullptr));
          response["result"] = node->execute(
            request.value("method", ""), request.value("params", Json::object()));
        } catch (const std::exception & error) {
          response["error"] = error.what();
        }
        std::cout << response.dump(-1, ' ', false, Json::error_handler_t::replace) << std::endl;
      }
    }
    executor.cancel();
    spin.join();
    executor.remove_node(node);
    node.reset();
    rclcpp::shutdown();
    return 0;
  } catch (const std::exception & error) {
    std::cerr << error.what() << std::endl;
    rclcpp::shutdown();
    return 1;
  }
}
