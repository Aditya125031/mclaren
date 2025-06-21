#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <cmath>
#include <chrono>
#include <algorithm>

using namespace std::chrono_literals;
using std::placeholders::_1;

class WallAlertNode : public rclcpp::Node
{
public:
  WallAlertNode()
  : Node("wall_alert_node"), stop_robot_(false)
  {
    // --- Parameters ---------------------------------------------------------
    last_cmd_ = geometry_msgs::msg::Twist();

    declare_parameter<double>("alert_distance", 0.65);
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
    const double half_fov_rad=M_PI/6.0;
    int idx_min=std::lround((-half_fov_rad-msg->angle_min)/msg->angle_increment);
    int idx_max=std::lround((half_fov_rad-msg->angle_min)/msg->angle_increment);
    
    idx_min=std::clamp(idx_min, 0, static_cast<int>(msg->ranges.size())-1);
    idx_max=std::clamp(idx_max, 0, static_cast<int>(msg->ranges.size())-1);
    if (idx_min > idx_max) std::swap(idx_min, idx_max);
    
    double min_dist=msg->range_max;
      for(int i=idx_min; i<=idx_max; ++i)
      {
    	RCLCPP_INFO(get_logger(), "Ray[%d] = %.2f m", i, msg->ranges[i]);
    	
        double d=msg->ranges[i];
        if (std::isfinite(d) && d >= msg->range_min && d <= msg->range_max) 
        {
      min_dist = std::min(min_dist, d);
         }
      }
      
      bool obstacle=(min_dist<alert_distance_);
      
    if(obstacle && !stop_robot_)
    {
    	RCLCPP_WARN(get_logger(),
      "⚠️  Obstacle within %.2f m in ±30° cone (closest=%.2f m). Stopping forward.",
      alert_distance_, min_dist);
    }
    else if(!obstacle && stop_robot_)
    {
     	RCLCPP_INFO(get_logger(),
      "✅  ±30° cone clear (closest=%.2f m). Resuming teleop.", min_dist);
    }
    stop_robot_=obstacle;
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





