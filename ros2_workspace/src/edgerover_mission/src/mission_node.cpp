// 寻物-接近-描述：SEARCH 原地转找目标 → APPROACH 视觉伺服接近 → DESCRIBE 问 VLM → IDLE
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>   // snprintf，用于把距离和框高拼进诊断信息
#include <memory>
#include <string>


#include "edgerover_ros_bridge/srv/vlm_ask.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/string.hpp"
#include "vision_msgs/msg/detection2_d_array.hpp"

using edgerover_ros_bridge::srv::VlmAsk;
using geometry_msgs::msg::Twist;

namespace {
constexpr double kImgW = 640.0;         // /edgerover/detections 是 640x640 letterbox 坐标
constexpr double kSearchWz = 0.3;       // 搜索时的原地转速 rad/s
constexpr double kSearchTimeout = 45.0; // 转两圈还没找到就放弃
constexpr double kApproachVx = 0.10;    // 接近时的前进速度 m/s
constexpr double kTurnGain = 0.8;       // 框中心偏差（-1..1）到角速度的比例系数，上板调
constexpr double kMaxWz = 0.4;          // 接近时角速度上限
constexpr double kStopDist = 0.45;      // 正前方激光小于这个距离就停 m
constexpr double kStopBoxH = 0.75;      // 检测框高度占画面比例超过这个值也停（激光无效时兜底）
constexpr double kLostTimeout = 1.5;    // 接近中目标丢失超过这个时间，回到搜索
constexpr double kReacquireWz = 0.12;   // 丢目标后朝原方向慢转的角速度 rad/s，比全速自转更容易重新锁定
constexpr double kSeenWindow = 0.5;     // 这么久以内看到过才算“当前可见”
}  // namespace

class MissionNode : public rclcpp::Node {
 public:
  MissionNode() : Node("edgerover_mission") {
    cmd_pub_ = create_publisher<Twist>("/cmd_vel", 10);
    res_pub_ = create_publisher<std_msgs::msg::String>("/edgerover/mission/result", 10);
    target_sub_ = create_subscription<std_msgs::msg::String>(
        "/edgerover/mission/target", 10,
        [this](std_msgs::msg::String::ConstSharedPtr m) { OnTarget(*m); });
    // 检测话题是 SensorDataQoS（BEST_EFFORT），订阅端必须匹配
    det_sub_ = create_subscription<vision_msgs::msg::Detection2DArray>(
        "/edgerover/detections", rclcpp::SensorDataQoS(),
        [this](vision_msgs::msg::Detection2DArray::ConstSharedPtr m) { OnDet(*m); });
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10, [this](sensor_msgs::msg::LaserScan::ConstSharedPtr m) { OnScan(*m); });
    vlm_ = create_client<VlmAsk>("/edgerover/vlm/ask");
    // 10 Hz 控制循环：底盘 500 ms 收不到 /cmd_vel 会自动停车，所以要持续发
    timer_ = create_wall_timer(std::chrono::milliseconds(100), [this]() { Tick(); });
    state_t0_ = Now();
  }

 private:
  enum class State { kIdle, kSearch, kApproach, kDescribe };

  double Now() { return get_clock()->now().seconds(); }

  void Report(const std::string& text) {
    RCLCPP_INFO(get_logger(), "%s", text.c_str());
    std_msgs::msg::String m;
    m.data = text;
    res_pub_->publish(m);
  }

  void SetState(State s) {
    static const char* const kNames[] = {"IDLE", "SEARCH", "APPROACH", "DESCRIBE"};
    state_ = s;
    state_t0_ = Now();
    Report(std::string("state=") + kNames[static_cast<int>(s)] + " target=" + target_);
  }

  void OnTarget(const std_msgs::msg::String& msg) {
    if (msg.data.empty() || msg.data == "stop") {
      SetState(State::kIdle);          // 中止：IDLE 不再发速度，这里再补发一次零速
      cmd_pub_->publish(Twist());
      return;
    }
    target_ = msg.data;
    has_det_ = false;
    reacquire_wz_ = kSearchWz;         // 新任务从全速搜索开始，丢掉“上一次朝哪转”的记忆
    SetState(State::kSearch);
  }

  void OnDet(const vision_msgs::msg::Detection2DArray& msg) {
    // 在目标类别里取分数最高的一个框
    double best_score = -1.0;
    for (const auto& d : msg.detections) {
      if (d.results.empty()) continue;
      const auto& h = d.results[0].hypothesis;
      if (h.class_id == target_ && h.score > best_score) {
        best_score = h.score;
        det_cx_ = d.bbox.center.position.x;
        det_h_ = d.bbox.size_y;
      }
    }
    if (best_score >= 0.0) {
      has_det_ = true;
      det_t_ = Now();
    }
  }

  void OnScan(const sensor_msgs::msg::LaserScan& msg) {
    // 0° 是车头正前方；取 ±10° 内的有效点（雷达约 35% 的点是 0，要滤掉）
    const int n = static_cast<int>(msg.ranges.size());
    front_ = -1.0;                     // -1 表示正前方没有有效点
    if (n == 0) return;
    for (int i = -10; i <= 10; ++i) {
      const double r = msg.ranges[static_cast<size_t>(((i % n) + n) % n)];
      if (r > msg.range_min && r < msg.range_max && (front_ < 0.0 || r < front_)) front_ = r;
    }
  }

  void Tick() {
    const double t = Now();
    Twist cmd;

    if (state_ == State::kSearch) {
      if (has_det_ && t - det_t_ < kSeenWindow) {
        SetState(State::kApproach);
      } else if (t - state_t0_ > kSearchTimeout) {
        Report("没有找到 " + target_);
        SetState(State::kIdle);
      } else {
        cmd.angular.z = reacquire_wz_; // 首次搜索全速转；从 APPROACH 退回时朝最后看到目标的一侧慢转
      }
      cmd_pub_->publish(cmd);
    } else if (state_ == State::kApproach) {
      if (!has_det_ || t - det_t_ > kLostTimeout) {
        // 打出丢失时的激光距离和目标框高度，用来判断是“太近出画面”还是“根本没检出”
        char buf[128];
        std::snprintf(buf, sizeof(buf), "目标丢失 front=%.2f box_h=%.2f",
                      front_, det_h_ / kImgW);
        Report(buf);
        reacquire_wz_ = last_err_ >= 0.0 ? kReacquireWz : -kReacquireWz;  // 朝最后看到目标的一侧转回去
        SetState(State::kSearch);
        return;
      }
      const bool close = (front_ > 0.0 && front_ < kStopDist) || (det_h_ / kImgW > kStopBoxH);
      if (close) {
        cmd_pub_->publish(Twist());    // 到位，先停车再问 VLM
        SetState(State::kDescribe);
        AskVlm();
        return;
      }
      // 目标在画面左侧时 err 为正 → 逆时针（左）转
      const double err = (kImgW / 2.0 - det_cx_) / (kImgW / 2.0);
      last_err_ = err; 
      cmd.angular.z = std::clamp(kTurnGain * err, -kMaxWz, kMaxWz);
      // 偏差大时只转不走，对准后再前进，避免斜着冲过去
      cmd.linear.x = std::fabs(err) < 0.4 ? kApproachVx : 0.0;
      cmd_pub_->publish(cmd);
    }
    // IDLE / DESCRIBE 不发速度指令
  }

  void AskVlm() {
    if (!vlm_->service_is_ready()) {
      Report("VLM 服务不可用（ros_bridge 没启动？）");
      SetState(State::kIdle);
      return;
    }
    auto req = std::make_shared<VlmAsk::Request>();
    req->question = "描述一下画面里的" + target_ + "和它周围的环境。";
    req->timeout_s = 0.0f;             // <=0 表示用 bridge 的参数 vlm_timeout_s
    // 异步调用：VLM 要几秒到几十秒，不能阻塞控制循环
    vlm_->async_send_request(req, [this](rclcpp::Client<VlmAsk>::SharedFuture fut) {
      const auto res = fut.get();
      if (res->success) {
        Report("VLM: " + res->answer);
      } else {
        Report("VLM 失败: " + res->error);
      }
      SetState(State::kIdle);
    });
  }

  State state_ = State::kIdle;
  std::string target_;
  double state_t0_ = 0.0;
  bool has_det_ = false;               // 是否看到过目标
  double det_t_ = 0.0;                 // 最近一次看到目标的时间
  double det_cx_ = 0.0;                // 目标框中心 x（像素）
  double last_err_ = 0.0;              // 最近一次接近时目标的横向偏差，负值表示目标在右侧
  double reacquire_wz_ = kSearchWz;    // 当前搜索时的角速度，APPROACH 丢失后会改成慢速回转
  double det_h_ = 0.0;                 // 目标框高度（像素）
  double front_ = -1.0;                // 正前方最近的有效激光距离，-1 表示无效

  rclcpp::Publisher<Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr res_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr target_sub_;
  rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr det_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Client<VlmAsk>::SharedPtr vlm_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  // Ctrl-C 后不用补发停车：底盘 500 ms 看门狗会自己停
  rclcpp::spin(std::make_shared<MissionNode>());
  rclcpp::shutdown();
  return 0;
}