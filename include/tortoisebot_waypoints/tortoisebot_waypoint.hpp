
#ifndef TORTOISEBOT_WAYPOINT_HPP_
#define TORTOISEBOT_WAYPOINT_HPP_

#include "iostream"
#include <functional>
#include <math.h>
#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/detail/odometry__struct.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/utils.h"
#include <tf2/LinearMath/Quaternion.h>

#include "rclcpp_action/server_goal_handle.hpp"
#include "tortoisebot_waypoints/action/detail/waypoint_action__struct.hpp"
#include "tortoisebot_waypoints/action/waypoint_action.hpp"

#define ERROR_DIST 0.05
#define ERROR_YAW 0.034906
#define PI 3.1415926535

class MyActionServer : public rclcpp::Node {
public:
  using Waypoint = tortoisebot_waypoints::action::WaypointAction;
  using GoalHandleMove = rclcpp_action::ServerGoalHandle<Waypoint>;

  explicit MyActionServer(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

private:
  rclcpp_action::Server<Waypoint>::SharedPtr action_server_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;

  float pos_current_x_;
  float pos_current_y_;
  double pos_current_yaw_;

  rclcpp_action::GoalResponse
  handle_goal(const rclcpp_action::GoalUUID &uuid,
              std::shared_ptr<const Waypoint::Goal> goal);
  rclcpp_action::CancelResponse
  handle_cancel(const std::shared_ptr<GoalHandleMove> goal_handle);

  void handle_accepted(const std::shared_ptr<GoalHandleMove> goal_handle);

  void execute(const std::shared_ptr<GoalHandleMove> goal_handle);

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr odom);
}; // class MyActionServer

class MyActionClient : public rclcpp::Node {
public:
  using Waypoint = tortoisebot_waypoints::action::WaypointAction;
  using GoalHandleMove = rclcpp_action::ClientGoalHandle<Waypoint>;

  explicit MyActionClient(
      const rclcpp::NodeOptions &node_options = rclcpp::NodeOptions());

  bool is_goal_done() const;
  bool goal_done_;
  float get_pos_current_x(void) const;
  float get_pos_current_y(void) const;
  double get_pos_current_yaw(void) const;
  float get_goal_x(void) const;
  float get_goal_y(void) const;
  double get_goal_yaw(void) const;

private:
  float pos_current_x_;
  float pos_current_y_;
  double pos_current_yaw_;
  float goal_x_;
  float goal_y_;
  double goal_yaw_;
  void send_goal();
  void goal_response_callback(const GoalHandleMove::SharedPtr &goal_handle);
  void
  feedback_callback(GoalHandleMove::SharedPtr,
                    const std::shared_ptr<const Waypoint::Feedback> feedback);
  void result_callback(const GoalHandleMove::WrappedResult &result);

private:
  rclcpp_action::Client<Waypoint>::SharedPtr client_ptr_;
  rclcpp::TimerBase::SharedPtr timer_;
};

#endif