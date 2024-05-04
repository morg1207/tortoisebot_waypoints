#include <functional>
#include <memory>
#include <thread>

#include <math.h>

#include "nav_msgs/msg/detail/odometry__struct.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/twist.hpp"
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
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : Node("my_action_server", options) {
    using namespace std::placeholders;

    this->action_server_ = rclcpp_action::create_server<Waypoint>(
        this, "tortoisebot_as",
        std::bind(&MyActionServer::handle_goal, this, _1, _2),
        std::bind(&MyActionServer::handle_cancel, this, _1),
        std::bind(&MyActionServer::handle_accepted, this, _1));

    // publisher_ =
    //    this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "odom", 10, std::bind(&MyActionServer::odom_callback, this, _1));

    pos_current_x_ = 0.0;
    pos_current_y_ = 0.0;
    pos_current_yaw_ = 0.0;
  }

private:
  rclcpp_action::Server<Waypoint>::SharedPtr action_server_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;

  float pos_current_x_;
  float pos_current_y_;
  double pos_current_yaw_;

  rclcpp_action::GoalResponse
  handle_goal(const rclcpp_action::GoalUUID &uuid,
              std::shared_ptr<const Waypoint::Goal> goal) {
    RCLCPP_INFO(
        this->get_logger(),
        "Received goal request with position: x =  %.3f --  y =  %.3f  -- %.3f",
        goal->position.x, goal->position.y, goal->position.z);
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse
  handle_cancel(const std::shared_ptr<GoalHandleMove> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleMove> goal_handle) {
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a
    // new thread
    std::thread{std::bind(&MyActionServer::execute, this, _1), goal_handle}
        .detach();
  }

  void execute(const std::shared_ptr<GoalHandleMove> goal_handle) {

    const auto goal = goal_handle->get_goal();
    RCLCPP_INFO(this->get_logger(),
                "Executing goal position: x =  %.3f --  y =  %.3f  -- %.3f",
                goal->position.x, goal->position.y, goal->position.z);

    auto feedback = std::make_shared<Waypoint::Feedback>();
    auto &position = feedback->position;
    auto &state = feedback->state;

    auto result = std::make_shared<Waypoint::Result>();
    auto cmd_vel_msg = geometry_msgs::msg::Twist();
    rclcpp::Rate loop_rate(25);

    float error_pos = 0.0;
    float error_yaw = 0.0;
    float desired_yaw = 0.0;

    desired_yaw = atan2(goal->position.y - this->pos_current_y_,
                        goal->position.x - this->pos_current_x_);

    error_pos = sqrt(pow(goal->position.y - this->pos_current_y_, 2) +
                     pow(goal->position.x - this->pos_current_x_, 2));
    error_yaw = desired_yaw - this->pos_current_yaw_;

    while (error_pos > ERROR_DIST && rclcpp::ok() && !result->success) {

      // error
      desired_yaw = atan2(goal->position.y - this->pos_current_y_,
                          goal->position.x - this->pos_current_x_);

      error_pos = sqrt(pow(goal->position.y - this->pos_current_y_, 2) +
                       pow(goal->position.x - this->pos_current_x_, 2));
      error_yaw = desired_yaw - this->pos_current_yaw_;

      if (error_yaw > PI) {
        error_yaw = error_yaw - 2 * PI;
      }
      if (error_yaw < -PI) {
        error_yaw = error_yaw + 2 * PI;
      }

      RCLCPP_INFO(this->get_logger(),
                  "Errors  error_pos =  %.3f --  err_yaw = %.3f   -- "
                  "current_yaw = %.3f",
                  error_pos, error_yaw, this->pos_current_yaw_);

      // Check if there is a cancel request
      if (goal_handle->is_canceling()) {
        result->success = false;
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        return;
      }
      //  if ((abs(error_yaw) > ERROR_YAW && abs(error_yaw) < PI) ||
      //    (abs(2 * PI - abs(error_yaw)) > ERROR_YAW && abs(error_yaw) > PI)) {
      if (abs(error_yaw) > ERROR_YAW) {

        RCLCPP_INFO(this->get_logger(), "Correct yaw");
        if (error_yaw > 0)
          cmd_vel_msg.angular.z = 0.65;
        else
          cmd_vel_msg.angular.z = -0.65;
        cmd_vel_msg.linear.x = 0.0;
        publisher_->publish(cmd_vel_msg);
      } else {
        RCLCPP_INFO(this->get_logger(), "Correct pose");
        cmd_vel_msg.linear.x = 0.65;
        cmd_vel_msg.angular.z = 0.0;
        publisher_->publish(cmd_vel_msg);
      }
      position.x = pos_current_x_;
      position.y = pos_current_y_;
      position.z = pos_current_yaw_;
      state = "Doing goal";
      goal_handle->publish_feedback(feedback);
      RCLCPP_INFO(this->get_logger(), "Publish feedback");

      loop_rate.sleep();
    }

    // Check if goal is done
    if (rclcpp::ok()) {
      result->success = true;
      cmd_vel_msg.linear.x = 0.0;
      cmd_vel_msg.angular.z = 0.0;
      publisher_->publish(cmd_vel_msg);
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    }
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr odom) {

    pos_current_x_ = odom->pose.pose.position.x;
    pos_current_y_ = odom->pose.pose.position.y;
    tf2::Quaternion q(
        odom->pose.pose.orientation.x, odom->pose.pose.orientation.y,
        odom->pose.pose.orientation.z, odom->pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch;
    m.getRPY(roll, pitch, pos_current_yaw_);
    // RCLCPP_INFO(this->get_logger(), "Odom recived");
  }
}; // class MyActionServer

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto action_server = std::make_shared<MyActionServer>();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(action_server);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}