#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <cmath>
#include <chrono>

using namespace std::chrono_literals;
using std::placeholders::_1;

class WallAlertNode : public rclcpp::Node
{
public:
  WallAlertNode()
  : Node("wall_alert_node"), stop_robot_(false)
  {
    // --- Parameters ---------------------------------------------------------
    declare_parameter<double>("alert_distance", 0.5);
    get_parameter("alert_distance", alert_distance_);

    // --- Publisher ----------------------------------------------------------
    vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    // --- Subscriptions ------------------------------------------------------
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", rclcpp::SensorDataQoS(),
      std::bind(&WallAlertNode::scan_callback, this, _1));

    teleop_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "/raw_cmd_vel", 10,
      std::bind(&WallAlertNode::teleop_callback, this, _1));

    // --- Timer to enforce commands at 10 Hz -------------------------------
    timer_ = create_wall_timer(
      100ms, std::bind(&WallAlertNode::timer_callback, this));

    RCLCPP_INFO(get_logger(),
      "Wall‑Alert node started. alert_distance = %.2f m", alert_distance_);
  }

private:
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    int idx = std::lround((0.0 - msg->angle_min) / msg->angle_increment);
    idx = std::clamp(idx, 0, static_cast<int>(msg->ranges.size()) - 1);

    double dist = msg->ranges[idx];
    bool obstacle = (dist >= msg->range_min &&
                     dist <= msg->range_max &&
                     std::isfinite(dist) &&
                     dist < alert_distance_);

    if (obstacle && !stop_robot_) {
      RCLCPP_WARN(get_logger(),
        "⚠️  Obstacle at 0° within %.2f m (dist=%.2f). Stopping forward.",
        alert_distance_, dist);
    } else if (!obstacle && stop_robot_) {
      RCLCPP_INFO(get_logger(),
        "✅  0° clear (dist=%.2f). Resuming teleop.", dist);
    }
    stop_robot_ = obstacle;
  }

  void teleop_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    last_cmd_ = *msg;
  }

  void timer_callback()
  {
    geometry_msgs::msg::Twist cmd = last_cmd_;

    if (stop_robot_) {
 
      if (cmd.linear.x > 0.0) {
        cmd.linear.x = 0.0;
      }
    }

    vel_pub_->publish(cmd);
  }

  // -------------------------------------------------------------------------
  double alert_distance_{0.5};
  bool   stop_robot_;
  geometry_msgs::msg::Twist last_cmd_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr      vel_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr   teleop_sub_;
  rclcpp::TimerBase::SharedPtr                                timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WallAlertNode>());
  rclcpp::shutdown();
  return 0;
}




