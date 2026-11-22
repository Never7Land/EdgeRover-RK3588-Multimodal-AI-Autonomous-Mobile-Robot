
// 订阅底盘 /odom，广播 odom -> child_frame 的 TF（child_frame 参数为空时用消息里的 base_link）。
// frame 名与时间戳直接取自 Odometry 消息（固件：odom / base_link，
// stamp 为 rmw_uros_epoch_millis，已与 agent 同步）。

#include <memory>
#include <string>
#include <cmath>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>

class OdomTfNode : public rclcpp::Node 
{
  public:
    OdomTfNode() : Node("odom_tf") 
    {

      // 可选参数 child_frame：覆盖 TF 的子坐标系名；为空时沿用 /odom 消息里的 child_frame_id（固件填的是 base_link）
      child_frame_ = declare_parameter<std::string>("child_frame", "");

      const auto topic = declare_parameter<std::string>("odom_topic", "/odom");

      // /odom 由固件以 default QoS（RELIABLE, KEEP_LAST 10）发布，订阅端保持一致

      sub_ = create_subscription<nav_msgs::msg::Odometry>(
          topic, 
          rclcpp::QoS(10),
          [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) { OnOdom(*msg); }
        );
        
      broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
      
      // 打印订阅的话题和实际使用的子坐标系；参数为空时显示 "(from msg)"，表示用消息里的值
      RCLCPP_INFO(get_logger(), 
                  "odom->TF: subscribing %s, child_frame=%s", 
                  topic.c_str(),
                  child_frame_.empty() ? "(from msg)" : child_frame_.c_str());
    }

  private:
    void OnOdom(const nav_msgs::msg::Odometry &msg) 
    {
        if (msg.header.frame_id.empty() || msg.child_frame_id.empty()) 
        {
            RCLCPP_WARN_THROTTLE(
              get_logger(), 
              *get_clock(), 
              5000,                 
              "odom message has empty frame_id/child_frame_id, skip"
            );

          return;
        }

        const rclcpp::Time stamp(msg.header.stamp);

        const double skew = std::abs((now() - stamp).seconds());

        if (skew > 1.0) 
        {
          RCLCPP_WARN_THROTTLE(
            get_logger(), 
            *get_clock(), 
            5000,          
            "odom stamp skew %.1fs vs host clock (time sync failed?)", skew);
        }

        geometry_msgs::msg::TransformStamped t;

        t.header                  = msg.header;

        // 参数非空就用参数（如 base_footprint），让 TF 接到 URDF 根节点上，避免 base_link 有两个父节点；为空时沿用消息里的 base_link
        t.child_frame_id          = child_frame_.empty() ? msg.child_frame_id : child_frame_;

        t.transform.translation.x = msg.pose.pose.position.x;
        t.transform.translation.y = msg.pose.pose.position.y;
        t.transform.translation.z = msg.pose.pose.position.z;

        t.transform.rotation      = msg.pose.pose.orientation;

        broadcaster_->sendTransform(t);
    }

    std::string child_frame_;  // 保存 child_frame 参数值，每条 odom 消息的回调里都要用

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
};

int main(int argc, char **argv) 
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdomTfNode>());
  rclcpp::shutdown();
  return 0;
}
