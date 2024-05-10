#include "tortoisebot_waypoint.hpp"
#include "tortoisebot_waypoints/tortoisebot_waypoint.hpp"
#include <iostream>

MyActionServer::MyActionServer(const rclcpp::NodeOptions &options)
    : Node("my_action_server", options) {
  using namespace std::placeholders;

  this->action_server_ = rclcpp_action::create_server<Waypoint>(
      this, "tortoisebot_as",
      std::bind(&MyActionServer::handle_goal, this, _1, _2),
      std::bind(&MyActionServer::handle_cancel, this, _1),
      std::bind(&MyActionServer::handle_accepted, this, _1));

  publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

  subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "odom", 10, std::bind(&MyActionServer::odom_callback, this, _1));

  pos_current_x_ = 0.0;
  pos_current_y_ = 0.0;
  pos_current_yaw_ = 0.0;
}

rclcpp_action::GoalResponse
MyActionServer::handle_goal(const rclcpp_action::GoalUUID &uuid,
                            std::shared_ptr<const Waypoint::Goal> goal) {
  RCLCPP_INFO(
      this->get_logger(),
      "Received goal request with position: x =  %.3f --  y =  %.3f  -- %.3f",
      goal->position.x, goal->position.y, goal->position.z);
  (void)uuid;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse MyActionServer::handle_cancel(
    const std::shared_ptr<GoalHandleMove> goal_handle) {
  RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void MyActionServer::handle_accepted(
    const std::shared_ptr<GoalHandleMove> goal_handle) {
  using namespace std::placeholders;
  std::thread{std::bind(&MyActionServer::execute, this, _1), goal_handle}
      .detach();
}

void MyActionServer::execute(
    const std::shared_ptr<GoalHandleMove> goal_handle) {
  // Implementación de la función execute
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
  int type_control = 1;

  desired_yaw = atan2(goal->position.y - this->pos_current_y_,
                      goal->position.x - this->pos_current_x_);

  error_pos = sqrt(pow(goal->position.y - this->pos_current_y_, 2) +
                   pow(goal->position.x - this->pos_current_x_, 2));
  error_yaw = desired_yaw - this->pos_current_yaw_;

  while (rclcpp::ok() && type_control != 0) {

    switch (type_control) {
    case 1:
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

        RCLCPP_INFO(this->get_logger(), "correcting orientation");
        if (error_yaw > 0)
          cmd_vel_msg.angular.z = 0.65;
        else
          cmd_vel_msg.angular.z = -0.65;
        cmd_vel_msg.linear.x = 0.0;
        publisher_->publish(cmd_vel_msg);
      } else {
        RCLCPP_INFO(this->get_logger(), "correcting position");
        cmd_vel_msg.linear.x = 0.65;
        cmd_vel_msg.angular.z = 0.0;
        publisher_->publish(cmd_vel_msg);
      }
      position.x = pos_current_x_;
      position.y = pos_current_y_;
      position.z = pos_current_yaw_;

      goal_handle->publish_feedback(feedback);
      RCLCPP_INFO(this->get_logger(), "Publish feedback");
      if (abs(error_pos) < ERROR_DIST) {
        type_control = 2;
      }

      break;
    case 2:
      RCLCPP_INFO(this->get_logger(), "Case 2");
      // Check if there is a cancel request
      if (goal_handle->is_canceling()) {
        result->success = false;
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        return;
      }
      error_yaw = goal->position.z - this->pos_current_yaw_;

      if (error_yaw > PI) {
        error_yaw = error_yaw - 2 * PI;
      }
      if (error_yaw < -PI) {
        error_yaw = error_yaw + 2 * PI;
      }
      RCLCPP_INFO(this->get_logger(),
                  "Error orientation: yaw =  %.3f  error_yaw =  %.3f",
                  this->pos_current_yaw_, error_yaw);

      if (error_yaw > 0)
        cmd_vel_msg.angular.z = 0.65;
      else
        cmd_vel_msg.angular.z = -0.65;

      cmd_vel_msg.linear.x = 0.0;
      publisher_->publish(cmd_vel_msg);
      state = "Doing goal";

      position.x = pos_current_x_;
      position.y = pos_current_y_;
      position.z = pos_current_yaw_;

      goal_handle->publish_feedback(feedback);
      RCLCPP_INFO(this->get_logger(), "Publish feedback");
      if (abs(error_yaw) < ERROR_YAW) {
        type_control = 0;
      }
      break;
    }

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

void MyActionServer::odom_callback(
    const nav_msgs::msg::Odometry::SharedPtr odom) {
  // Implementación de la función odom_callback
  this->pos_current_x_ = odom->pose.pose.position.x;
  this->pos_current_y_ = odom->pose.pose.position.y;
  tf2::Quaternion q(
      odom->pose.pose.orientation.x, odom->pose.pose.orientation.y,
      odom->pose.pose.orientation.z, odom->pose.pose.orientation.w);
  tf2::Matrix3x3 m(q);
  double roll, pitch;
  m.getRPY(roll, pitch, this->pos_current_yaw_);
}

MyActionClient::MyActionClient(const rclcpp::NodeOptions &node_options)
    : Node("my_action_client", node_options), goal_done_(false) {

  this->client_ptr_ = rclcpp_action::create_client<Waypoint>(
      this->get_node_base_interface(), this->get_node_graph_interface(),
      this->get_node_logging_interface(), this->get_node_waitables_interface(),
      "tortoisebot_as");

  this->timer_ =
      this->create_wall_timer(std::chrono::milliseconds(500),
                              std::bind(&MyActionClient::send_goal, this));
}

bool MyActionClient::is_goal_done() const { return this->goal_done_; }

void MyActionClient::send_goal() {
  using namespace std::placeholders;

  this->timer_->cancel();

  this->goal_done_ = false;

  if (!this->client_ptr_) {
    RCLCPP_ERROR(this->get_logger(), "Action client not initialized");
  }

  if (!this->client_ptr_->wait_for_action_server(std::chrono::seconds(10))) {
    RCLCPP_ERROR(this->get_logger(),
                 "Action server not available after waiting");
    this->goal_done_ = true;
    return;
  }

  auto goal_point = Waypoint::Goal();
  RCLCPP_INFO(this->get_logger(), "Send the x-coordinate of the waypoint");
  // Lee el valor ingresado por el usuario y lo almacena en la variable
  // 'valor'
  // std::cin >> goal_point.position.x;
  // RCLCPP_INFO(this->get_logger(), "Send the Y-coordinate of the waypoint");
  // std::cin >> goal_point.position.y; // Solicita al usuario que ingrese un
  // valor

  // RCLCPP_INFO(this->get_logger(), "Send the Z-orientation of the waypoint");
  // std::cin >> goal_point.position.z; // Solicita al usuario que ingrese un
  // valor

  goal_point.position.x = 0.3;
  goal_point.position.y = 0.3;
  goal_point.position.z = 1.57 :

      RCLCPP_INFO(this->get_logger(), "Sending goal");

  auto send_goal_options = rclcpp_action::Client<Waypoint>::SendGoalOptions();

  send_goal_options.goal_response_callback =
      std::bind(&MyActionClient::goal_response_callback, this, _1);

  send_goal_options.feedback_callback =
      std::bind(&MyActionClient::feedback_callback, this, _1, _2);

  send_goal_options.result_callback =
      std::bind(&MyActionClient::result_callback, this, _1);

  this->goal_x_ = goal_point.position.x;

  this->goal_y_ = goal_point.position.y;

  this->goal_yaw_ = goal_point.position.z;
  RCLCPP_ERROR(this->get_logger(),
               "Goal recivido x = %.3f   y = %.3f   yaw = %.3f", this->goal_x_,
               this->goal_y_, this->goal_yaw_);
  RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");

  auto goal_handle_future =
      this->client_ptr_->async_send_goal(goal_point, send_goal_options);
}

void MyActionClient::goal_response_callback(
    const GoalHandleMove::SharedPtr &goal_handle) {
  if (!goal_handle) {
    RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
  } else {
    RCLCPP_INFO(this->get_logger(),
                "Goal accepted by server, waiting for result");
  }
}

void MyActionClient::feedback_callback(
    GoalHandleMove::SharedPtr,
    const std::shared_ptr<const Waypoint::Feedback> feedback) {
  RCLCPP_INFO(this->get_logger(),
              "Feedback received : x  [%.3f] y  [%.3f] z  [%.3f] ",
              feedback->position.x, feedback->position.y, feedback->position.z);
  this->pos_current_x_ = feedback->position.x;
  this->pos_current_y_ = feedback->position.y;
  this->pos_current_yaw_ = feedback->position.z;
}

void MyActionClient::result_callback(
    const GoalHandleMove::WrappedResult &result) {
  this->goal_done_ = true;
  switch (result.code) {
  case rclcpp_action::ResultCode::SUCCEEDED:
    break;
  case rclcpp_action::ResultCode::ABORTED:
    RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
    return;
  case rclcpp_action::ResultCode::CANCELED:
    RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
    return;
  default:
    RCLCPP_ERROR(this->get_logger(), "Unknown result code");
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Result received: %d",
              result.result->success);
}

float MyActionClient::get_pos_current_x() const { return this->pos_current_x_; }
float MyActionClient::get_pos_current_y() const { return this->pos_current_y_; }
double MyActionClient::get_pos_current_yaw() const {
  return this->pos_current_yaw_;
}

float MyActionClient::get_goal_x() const {
  RCLCPP_ERROR(this->get_logger(), "Valor devuelto x  = %.3f", this->goal_x_);
  return this->goal_x_;
}
float MyActionClient::get_goal_y() const { return this->goal_y_; }
double MyActionClient::get_goal_yaw() const { return this->goal_yaw_; }
