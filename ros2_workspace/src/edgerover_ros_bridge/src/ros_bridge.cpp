// edgerover ros_bridge — step 3: det + vlm busy + vlm ask service
#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <sstream>
#include <string>
#include <thread>

#include <zmq.h>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <memory>
#include <sensor_msgs/msg/image.hpp>
#include <edgerover_ipc/frame_ring.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <edgerover_ros_bridge/srv/vlm_ask.hpp>

#include <mutex>
#include <algorithm>
#include "edgerover_ros_bridge/msg/stats.hpp"

using namespace std::chrono_literals;
using json = nlohmann::json;
using VlmAsk = edgerover_ros_bridge::srv::VlmAsk;

static const std::array<const char*, 80> kCoco = {
 "person","bicycle","car","motorcycle","airplane","bus","train","truck","boat","traffic light",
 "fire hydrant","stop sign","parking meter","bench","bird","cat","dog","horse","sheep","cow",
 "elephant","bear","zebra","giraffe","backpack","umbrella","handbag","tie","suitcase","frisbee",
 "skis","snowboard","sports ball","kite","baseball bat","baseball glove","skateboard","surfboard","tennis racket","bottle",
 "wine glass","cup","fork","knife","spoon","bowl","banana","apple","sandwich","orange",
 "broccoli","carrot","hot dog","pizza","donut","cake","chair","couch","potted plant","bed",
 "dining table","toilet","tv","laptop","mouse","remote","keyboard","cell phone","microwave","oven",
 "toaster","sink","refrigerator","book","clock","vase","scissors","teddy bear","hair drier","toothbrush"};

static std::string coco_name(int c) { return (c >= 0 && c < 80) ? kCoco[c] : std::to_string(c); }

static int64_t steady_ns() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count();   // == CLOCK_MONOTONIC
}

static std::string recv_str(void* sock, int flags) {   // 失败抛 -1
  zmq_msg_t m; zmq_msg_init(&m);
  if (zmq_msg_recv(&m, sock, flags) < 0) { zmq_msg_close(&m); throw -1; }
  std::string s(static_cast<char*>(zmq_msg_data(&m)), zmq_msg_size(&m));
  zmq_msg_close(&m);
  return s;
}

class RosBridge : public rclcpp::Node {
 public:
  RosBridge() : Node("edgerover_ros_bridge") {
    det_ep_        = declare_parameter<std::string>("det_endpoint", "ipc:///tmp/edgerover_det");
    state_ep_      = declare_parameter<std::string>("state_endpoint", "ipc:///tmp/edgerover_state");
    vlm_ep_        = declare_parameter<std::string>("vlm_endpoint", "ipc:///tmp/edgerover_vlm");
    vlm_timeout_s_ = declare_parameter<double>("vlm_timeout_s", 30.0);
    frame_id_      = declare_parameter<std::string>("frame_id", "camera_link");
    det_coords_    = declare_parameter<std::string>("det_coords", "letterbox");  // letterbox | source
    lb_size_       = declare_parameter<int>("letterbox_size", 640);
    publish_image_ = declare_parameter<bool>("publish_image", true);
    image_rate_hz_ = declare_parameter<double>("image_rate_hz", 5.0);
    shm_name_      = declare_parameter<std::string>("shm_name", "/edgerover_frames");
    busy_max_s_    = declare_parameter<double>("vlm_busy_max_s", 60.0);
    stats_rate_hz_ = declare_parameter<double>("stats_rate_hz", 1.0);

    det_pub_ = create_publisher<vision_msgs::msg::Detection2DArray>(
        "/edgerover/detections", rclcpp::SensorDataQoS());
    busy_pub_ = create_publisher<std_msgs::msg::Bool>(
        "/edgerover/vlm/busy", rclcpp::QoS(1).reliable().transient_local());

    vlm_cbg_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    vlm_srv_ = create_service<VlmAsk>(
        "/edgerover/vlm/ask",
        [this](const std::shared_ptr<VlmAsk::Request> req, std::shared_ptr<VlmAsk::Response> res) {
          on_ask(*req, *res);
        },
        rmw_qos_profile_services_default, vlm_cbg_);

    update_offset();
    offset_timer_ = create_wall_timer(10s, [this] { update_offset(); });
    busy_timer_ = create_wall_timer(std::chrono::seconds(1), [this] {
        if (busy_ && steady_ns() - busy_since_ns_.load() > int64_t(busy_max_s_ * 1e9)) {
            RCLCPP_WARN(get_logger(), "vlm busy > %.0fs without idle, forcing busy=false", busy_max_s_);
            busy_ = false;
            std_msgs::msg::Bool m; m.data = false;
            busy_pub_->publish(m);
        }
    });

    if (publish_image_) 
    {
      img_pub_   = create_publisher<sensor_msgs::msg::Image>("/edgerover/image", rclcpp::SensorDataQoS());
      img_timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / image_rate_hz_), [this] { on_image_tick(); });
    }


    stats_pub_ = create_publisher<edgerover_ros_bridge::msg::Stats>("/edgerover/stats", 10);
    last_stats_ns_ = steady_ns();
    stats_timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / stats_rate_hz_),
                                     [this] { on_stats_tick(); });

    zctx_ = zmq_ctx_new();
    io_thread_ = std::thread([this] { io_loop(); });
    RCLCPP_INFO(get_logger(), "det=%s state=%s vlm=%s coords=%s",
                det_ep_.c_str(), state_ep_.c_str(), vlm_ep_.c_str(), det_coords_.c_str());
  }

  ~RosBridge() override {
    running_ = false;
    if (io_thread_.joinable()) io_thread_.join();
    zmq_ctx_term(zctx_);
  }

 private:
  void update_offset() { offset_ns_ = now().nanoseconds() - steady_ns(); }


  // ---------- 统计 ----------
  void record_det(double age, double inf, size_t cnt) {
    std::lock_guard<std::mutex> lk(st_mtx_);
    win_.n++;
    win_.age_sum += age;  win_.age_max = std::max(win_.age_max, age);
    win_.inf_sum += inf;  win_.inf_max = std::max(win_.inf_max, inf);
    last_det_count_ = static_cast<uint32_t>(cnt);
    last_det_ns_ = steady_ns();
  }

  void on_stats_tick() {
    const int64_t t = steady_ns();
    const double dt = std::max(1e-3, (t - last_stats_ns_) / 1e9);
    last_stats_ns_ = t;

    edgerover_ros_bridge::msg::Stats m;
    m.header.stamp = now();
    m.header.frame_id = frame_id_;
    {
      std::lock_guard<std::mutex> lk(st_mtx_);
      m.det_hz = win_.n / dt;
      m.det_age_ms     = win_.n ? win_.age_sum / win_.n : 0.0;
      m.det_age_ms_max = win_.age_max;
      m.infer_ms       = win_.n ? win_.inf_sum / win_.n : 0.0;
      m.infer_ms_max   = win_.inf_max;
      m.det_count      = last_det_count_;
      m.det_silence_s  = last_det_ns_ ? (t - last_det_ns_) / 1e9 : -1.0;
      win_ = DetWin{};

      m.vlm_last_ok           = vlm_.ok;
      m.vlm_last_total_ms     = vlm_.total;
      m.vlm_last_rtt_ms       = vlm_.rtt;
      m.vlm_last_frame_age_ms = vlm_.age;
      m.vlm_ok_count          = vlm_.ok_n;
      m.vlm_err_count         = vlm_.err_n;
    }
    m.image_hz = img_n_.exchange(0) / dt;
    m.vlm_busy = busy_.load();
    stats_pub_->publish(m);
  }

  void* make_sub(const std::string& ep) 
  {
    void* s = zmq_socket(zctx_, ZMQ_SUB);
    int hwm = 10, linger = 0;
    zmq_setsockopt(s, ZMQ_RCVHWM, &hwm, sizeof hwm);
    zmq_setsockopt(s, ZMQ_LINGER, &linger, sizeof linger);
    zmq_setsockopt(s, ZMQ_SUBSCRIBE, "", 0);
    zmq_connect(s, ep.c_str());
    return s;
  }

  // ---------- IO 线程：det + state ----------
  void io_loop() 
  {
    void* det = make_sub(det_ep_);
    void* st  = make_sub(state_ep_);
    zmq_pollitem_t items[2] = {{det, 0, ZMQ_POLLIN, 0}, {st, 0, ZMQ_POLLIN, 0}};
    while (running_ && rclcpp::ok()) {
      if (zmq_poll(items, 2, 200) <= 0) continue;
      if (items[0].revents & ZMQ_POLLIN)
        try { for (;;) on_det(recv_str(det, ZMQ_DONTWAIT)); } catch (int) {}
      if (items[1].revents & ZMQ_POLLIN)
        try { for (;;) on_state(recv_str(st, ZMQ_DONTWAIT)); } catch (int) {}
    }
    zmq_close(det);
    zmq_close(st);
  }

  void on_state(const std::string& s) {

    bool busy;
    if (s.find("busy") != std::string::npos) 
    {
        busy = true;
    }
    else if (s.find("idle") != std::string::npos) 
    {
        busy = false;
    }
    else 
    { 
        RCLCPP_WARN(get_logger(), "unknown state msg: %s", s.c_str()); return; 
    }

    if (busy && !busy_) busy_since_ns_ = steady_ns();
    busy_ = busy;

    std_msgs::msg::Bool m; m.data = busy;

    busy_pub_->publish(m);

    RCLCPP_INFO(get_logger(), "vlm %s", busy ? "busy" : "idle");


  }

  void on_det(const std::string& s) {
    json j;
    try { j = json::parse(s); } catch (const std::exception& e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "bad det json: %s", e.what());
      return;
    }
    const double w = j.value("w", 3840.0), h = j.value("h", 2160.0);
    double sc = 1.0, px = 0.0, py = 0.0;
    if (det_coords_ == "letterbox") {
      sc = std::min(lb_size_ / w, lb_size_ / h);
      px = (lb_size_ - w * sc) / 2.0;
      py = (lb_size_ - h * sc) / 2.0;
    }

    vision_msgs::msg::Detection2DArray arr;
    arr.header.frame_id = frame_id_;
    arr.header.stamp = rclcpp::Time(j.value("ts_ns", int64_t(0)) + offset_ns_.load(), RCL_ROS_TIME);

    for (const auto& d : j.value("dets", json::array())) {
      const auto& b = d.at("box");
      const double x1 = b[0].get<double>() * sc + px, y1 = b[1].get<double>() * sc + py;
      const double x2 = b[2].get<double>() * sc + px, y2 = b[3].get<double>() * sc + py;

      vision_msgs::msg::Detection2D det;
      det.header = arr.header;
      det.id = std::to_string(d.value("id", -1));
      det.bbox.center.position.x = (x1 + x2) / 2.0;
      det.bbox.center.position.y = (y1 + y2) / 2.0;
      det.bbox.size_x = x2 - x1;
      det.bbox.size_y = y2 - y1;

      vision_msgs::msg::ObjectHypothesisWithPose hyp;
      hyp.hypothesis.class_id = coco_name(d.value("cls", -1));
      hyp.hypothesis.score = d.value("score", 0.0);
      det.results.push_back(hyp);
      arr.detections.push_back(std::move(det));
    }

    record_det(j.value("age_ms", 0.0), j.value("infer_ms", 0.0), arr.detections.size());

    det_pub_->publish(arr);
  }

  // ---------- 图像：FrameRing -> sensor_msgs/Image ----------
  void on_image_tick() {
    const int64_t tnow = steady_ns();
    if (!reader_) {
      if (tnow - last_open_try_ns_ < 1'000'000'000LL) return;   // 每秒重试一次
      last_open_try_ns_ = tnow;
      try {
        reader_ = std::make_unique<edgerover::ipc::FrameRingReader>(shm_name_);
        last_img_ok_ns_ = tnow;
        last_img_id_ = 0;
        RCLCPP_INFO(get_logger(), "FrameRing %s opened, slot=%u", shm_name_.c_str(), reader_->slot_size());
      } catch (const std::exception& e) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000, "FrameRing open failed: %s", e.what());
        return;
      }
    }

    auto msg = std::make_unique<sensor_msgs::msg::Image>();
    msg->data.resize(reader_->slot_size());
    edgerover::ipc::FrameInfo info;
    using edgerover::ipc::ReadResult;
    const ReadResult r = reader_->read_latest(msg->data.data(), msg->data.size(), info, last_img_id_);

    if (r != ReadResult::kOk) {
      if (r == ReadResult::kBufferTooSmall)
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "FrameRing buffer too small");
      if (tnow - last_img_ok_ns_ > 3'000'000'000LL) {           // 3s 无新帧 -> 重开
        RCLCPP_WARN(get_logger(), "FrameRing stalled >3s, reopening");
        reader_.reset();
      }
      return;                                                   // kNoNewFrame / kTorn：下次再读
    }
    last_img_ok_ns_ = tnow;
    last_img_id_ = info.frame_id;

    if (info.format != 0) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "unsupported format %u", info.format);
      return;
    }
    msg->data.resize(info.data_size);
    msg->header.frame_id = frame_id_;
    msg->header.stamp = rclcpp::Time(info.timestamp_ns + offset_ns_.load(), RCL_ROS_TIME);
    msg->width = info.width;
    msg->height = info.height;
    msg->encoding = "rgb8";
    msg->is_bigendian = 0;
    msg->step = info.width * 3;
    img_pub_->publish(std::move(msg));
    ++img_n_;
  }

  void on_ask(const VlmAsk::Request& req, VlmAsk::Response& res) {
    ask_impl(req, res);
    std::lock_guard<std::mutex> lk(st_mtx_);
    vlm_.ok    = res.success;
    vlm_.total = res.vlm_total_ms;
    vlm_.rtt   = res.rtt_ms;
    vlm_.age   = res.frame_age_ms;
    if (res.success) vlm_.ok_n++; else vlm_.err_n++;
  }


  // ---------- VLM 服务（lazy pirate：每次新建 REQ，超时即丢） ----------
  void ask_impl(const VlmAsk::Request& req, VlmAsk::Response& res){
    const int64_t t0 = steady_ns();
    res.success = false;
    if (req.question.empty()) { res.error = "empty question"; return; }
    const double tmo = req.timeout_s > 0 ? req.timeout_s : vlm_timeout_s_;

    void* s = zmq_socket(zctx_, ZMQ_REQ);
    int linger = 0;
    zmq_setsockopt(s, ZMQ_LINGER, &linger, sizeof linger);
    zmq_connect(s, vlm_ep_.c_str());

    std::string reply;
    if (zmq_send(s, req.question.data(), req.question.size(), 0) < 0) {
      res.error = std::string("send failed: ") + zmq_strerror(zmq_errno());
    } else {
      zmq_pollitem_t it{s, 0, ZMQ_POLLIN, 0};
      if (zmq_poll(&it, 1, static_cast<long>(tmo * 1000)) <= 0) {
        res.error = "timeout after " + std::to_string(tmo) + "s";
      } else {
        try { reply = recv_str(s, 0); } catch (int) { res.error = "recv failed"; }
      }
    }
    zmq_close(s);
    res.rtt_ms = (steady_ns() - t0) / 1e6;
    if (!res.error.empty()) {
      RCLCPP_WARN(get_logger(), "vlm ask failed: %s", res.error.c_str());
      return;
    }

    const auto nl = reply.find('\n');
    res.header = reply.substr(0, nl);
    if (res.header.rfind("OK", 0) != 0) {           // "ERR <reason>"
      res.error = res.header;
      return;
    }
    res.answer = (nl == std::string::npos) ? "" : reply.substr(nl + 1);
    std::istringstream iss(res.header);
    for (std::string tok; iss >> tok;) {
      const auto eq = tok.find('=');
      if (eq == std::string::npos) continue;
      const std::string k = tok.substr(0, eq), v = tok.substr(eq + 1);
      try {
        if (k == "frame") res.frame_id = std::stoll(v);
        else if (k == "age") res.frame_age_ms = std::stof(v);    // stof 会忽略 "ms"
        else if (k == "total") res.vlm_total_ms = std::stof(v);
      } catch (...) {}
    }
    res.success = true;
    RCLCPP_INFO(get_logger(), "vlm ok frame=%ld rtt=%.0fms", res.frame_id, res.rtt_ms);
  }

    std::string det_ep_, state_ep_, vlm_ep_, frame_id_, det_coords_;
    bool publish_image_;
    double image_rate_hz_;
    std::string shm_name_;
    std::unique_ptr<edgerover::ipc::FrameRingReader> reader_;
    uint64_t last_img_id_{0};
    int64_t last_img_ok_ns_{0}, last_open_try_ns_{0};
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr img_pub_;
    rclcpp::TimerBase::SharedPtr img_timer_;
    double vlm_timeout_s_;
    int lb_size_;
    std::atomic<int64_t> offset_ns_{0};
    rclcpp::Publisher<vision_msgs::msg::Detection2DArray>::SharedPtr det_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr busy_pub_;
    rclcpp::CallbackGroup::SharedPtr vlm_cbg_;
    rclcpp::Service<VlmAsk>::SharedPtr vlm_srv_;
    rclcpp::TimerBase::SharedPtr offset_timer_;
    void* zctx_{nullptr};
    std::thread io_thread_;
    std::atomic<bool> running_{true};
    std::atomic<bool> busy_{false};
    std::atomic<int64_t> busy_since_ns_{0};
    double busy_max_s_;
    rclcpp::TimerBase::SharedPtr busy_timer_;
    struct DetWin { uint32_t n = 0; double age_sum = 0, age_max = 0, inf_sum = 0, inf_max = 0; };
    struct VlmLast { bool ok = false; float total = 0, rtt = 0, age = 0; uint32_t ok_n = 0, err_n = 0; };
    std::mutex st_mtx_;            // on_det 在 IO 线程，on_ask 在服务线程，统计定时器在 executor 线程
    DetWin win_;
    VlmLast vlm_;
    uint32_t last_det_count_{0};
    int64_t last_det_ns_{0};
    std::atomic<uint32_t> img_n_{0};
    int64_t last_stats_ns_{0};
    double stats_rate_hz_;
    rclcpp::Publisher<edgerover_ros_bridge::msg::Stats>::SharedPtr stats_pub_;
    rclcpp::TimerBase::SharedPtr stats_timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RosBridge>();
  rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), 2);
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}