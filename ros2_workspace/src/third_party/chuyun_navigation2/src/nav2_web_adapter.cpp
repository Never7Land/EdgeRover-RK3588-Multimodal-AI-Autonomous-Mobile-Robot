#include <chrono>
#include <memory>
#include <optional>

#include <action_msgs/srv/cancel_goal.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/empty.hpp>

// The Nav2 navigator already accepts /goal_pose. Adapt the console's
// cancellation topic to the standard action cancellation service.
class Nav2WebAdapter : public rclcpp::Node
{
public:
  Nav2WebAdapter() : Node("nav2_web_adapter")
  {
    client_ = create_client<Cancel>("/navigate_to_pose/_action/cancel_goal");
    subscription_ = create_subscription<std_msgs::msg::Empty>(
      "/navigation/cancel", 1, [this](std_msgs::msg::Empty::ConstSharedPtr) {
        if (pending_) {
          return;
        }
        if (!client_->service_is_ready()) {
          RCLCPP_WARN(get_logger(), "Nav2 cancellation service is not ready");
          return;
        }
        auto request = std::make_shared<Cancel::Request>();
        // Zero UUID and zero timestamp mean cancel all current goals.
        try {
          const auto result = client_->async_send_request(
            request, [this](rclcpp::Client<Cancel>::SharedFuture future) {
              pending_.reset();
              try {
                const auto response = future.get();
                RCLCPP_INFO(get_logger(), "Nav2 cancel result=%d, goals=%zu",
                  response->return_code, response->goals_canceling.size());
              } catch (const std::exception & error) {
                RCLCPP_ERROR(get_logger(), "Nav2 cancellation failed: %s", error.what());
              }
            });
          pending_ = result.request_id;
          deadline_ = std::chrono::steady_clock::now() + std::chrono::seconds(2);
        } catch (const std::exception & error) {
          RCLCPP_ERROR(get_logger(), "Cannot send Nav2 cancellation: %s", error.what());
        }
      });
    timer_ = create_wall_timer(std::chrono::milliseconds(100), [this]() {
      if (pending_ && std::chrono::steady_clock::now() >= deadline_) {
        client_->remove_pending_request(*pending_);
        pending_.reset();
        RCLCPP_ERROR(get_logger(), "Nav2 cancellation response timed out");
      }
    });
  }

private:
  using Cancel = action_msgs::srv::CancelGoal;
  rclcpp::Client<Cancel>::SharedPtr client_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::optional<int64_t> pending_;
  std::chrono::steady_clock::time_point deadline_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Nav2WebAdapter>());
  rclcpp::shutdown();
  return 0;
}
