#include <algorithm>
#include <array>
#include <cmath>      // std::fabs 需要
#include <memory>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

namespace
{
  template<typename Covariance>
  void set_diagonal(Covariance & covariance, const std::array<double, 6> & diagonal)
  {
    std::fill(covariance.begin(), covariance.end(), 0.0);
    for (std::size_t index = 0; index < diagonal.size(); ++index) {
      covariance[index * 6 + index] = diagonal[index];
    }
  }

  template<typename Covariance>
  void set_diagonal_3d(Covariance & covariance, const std::array<double, 3> & diagonal)
  {
    std::fill(covariance.begin(), covariance.end(), 0.0);
    for (std::size_t index = 0; index < diagonal.size(); ++index) {
      covariance[index * 3 + index] = diagonal[index];
    }
  }
}  // namespace

class EkfInputAdapter : public rclcpp::Node
{
  public:
    EkfInputAdapter()
    : Node("ekf_input_adapter")
    {
      //可调参数：用 declare_parameter 声明，可以在 launch 或 yaml 里修改，不需要重新编译
      calib_duration_ = declare_parameter("gyro_calib_duration", 2.0);       // 启动校准时长（秒）
      still_linear_ = declare_parameter("still_linear_threshold", 0.001);    // 静止判定：线速度阈值（m/s）
      still_angular_ = declare_parameter("still_angular_threshold", 0.001);  // 静止判定：角速度阈值（rad/s）

      const auto input_qos = rclcpp::SensorDataQoS().keep_last(20);
      const auto output_qos = rclcpp::QoS(rclcpp::KeepLast(20)).reliable();

      odom_publisher_ = create_publisher<nav_msgs::msg::Odometry>(
        "/odom/ekf_input", output_qos);
      imu_publisher_ = create_publisher<sensor_msgs::msg::Imu>(
        "/imu/ekf_input", output_qos);

      odom_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
        "/odom", input_qos,
        [this](const nav_msgs::msg::Odometry::SharedPtr source) {

        //根据轮子的速度判断小车是否静止，供 IMU 回调做零速修正
        robot_still_ = std::fabs(source->twist.twist.linear.x) < still_linear_ 
                      && std::fabs(source->twist.twist.angular.z) < still_angular_;

          auto message = *source;
          message.child_frame_id = "base_footprint";

          // Wheel odometry: x/y and yaw pose, forward/lateral velocity.
          set_diagonal(
            message.pose.covariance,
            {0.01, 0.01, 1.0e6, 1.0e6, 1.0e6, 0.04});
          set_diagonal(
            message.twist.covariance,
            {0.0025, 0.0004, 1.0e6, 1.0e6, 1.0e6, 0.04});
          odom_publisher_->publish(message);
        });

      imu_subscription_ = create_subscription<sensor_msgs::msg::Imu>(
        "/imu", input_qos,
        [this](const sensor_msgs::msg::Imu::SharedPtr source) {
          auto message = *source;
          if (message.header.frame_id.empty()) {
            message.header.frame_id = "imu_link";
          }

          const double raw_z = message.angular_velocity.z;

          //启动校准。前 N 秒对 z 角速度求平均，得到零偏
          if (!calibrated_) 
          {
            if (!robot_still_) 
            {
              // 校准期间车动了，数据作废，从头开始（防止把真实的转动当成零偏）
              bias_sum_ = 0.0;
              bias_count_ = 0;
              calib_started_ = false;
              return;
            }
            if (!calib_started_) {
              calib_start_ = now();
              calib_started_ = true;
            }
            bias_sum_ += raw_z;
            ++bias_count_;
            if ((now() - calib_start_).seconds() >= calib_duration_) {
              gyro_z_bias_ = bias_sum_ / bias_count_;
              calibrated_ = true;
              RCLCPP_INFO(
                get_logger(), "Gyro Z bias calibrated: %.5f rad/s (%d samples)",
                gyro_z_bias_, bias_count_);
            }
            return;  // 校准完成前不发布 IMU，避免 EKF 吃到带零偏的数据
          }

          //每一帧减去零偏
          message.angular_velocity.z = raw_z - gyro_z_bias_;

          //零速修正（ZUPT）。轮子不动时，认为车身没有转动
          if (robot_still_) 
          {
            message.angular_velocity.z = 0.0;
          }

          // The MCU currently marks all IMU covariances as unknown (-1).
          // Supply conservative values so robot_localization can use gyro Z.
          set_diagonal_3d(message.orientation_covariance, {0.10, 0.10, 0.04});
          set_diagonal_3d(message.angular_velocity_covariance, {0.04, 0.04, 0.0025});
          set_diagonal_3d(message.linear_acceleration_covariance, {0.10, 0.10, 0.10});
          imu_publisher_->publish(message);
        });

      RCLCPP_INFO(
        get_logger(),
        "EKF inputs ready. Keep robot still for %.1fs to calibrate gyro.", calib_duration_);
    }

  private:
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;


    double calib_duration_{2.0};
    double still_linear_{0.001};
    double still_angular_{0.001};
    bool robot_still_{false};
    bool calibrated_{false};
    bool calib_started_{false};
    rclcpp::Time calib_start_;
    double bias_sum_{0.0};
    int bias_count_{0};
    double gyro_z_bias_{0.0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EkfInputAdapter>());
  rclcpp::shutdown();
  return 0;
}
