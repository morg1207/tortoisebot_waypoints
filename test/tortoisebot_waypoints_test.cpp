#include "std_msgs/msg/float32.hpp"
#include "tortoisebot_waypoints/tortoisebot_waypoint.hpp"
#include "gtest/gtest.h"
#include <memory>

#ifndef ERROR_DIST_ACCEPT 0.08
#define ERROR_DIST_ACCEPT 0.08
#endif
//#ifndef ERROR_YAW 0.034906
//#define ERROR_YAW 0.034906
#ifndef ERROR_YAW_ACCEPT 0.034906
#define ERROR_YAW_ACCEPT 0.034906
#endif

#ifndef ACCEPT_DIST 0.1
#define ACCEPT_DIST 0.1
#endif

using std::placeholders::_1;
using namespace std::chrono_literals;

enum class MyEnumTest { TEST_DISTANCE, TEST_ORIENTATION };

class RclCppFixture {
public:
  RclCppFixture() { rclcpp::init(0, nullptr); }
  ~RclCppFixture() { rclcpp::shutdown(); }
};
RclCppFixture g_rclcppfixture;

class GoToWaypointTestFixture : public ::testing::Test {
public:
  GoToWaypointTestFixture() {

    action_success_ = false;

    actionServerCtrlNode = std::make_shared<MyActionServer>();

    actionClientCtrlNode = std::make_shared<MyActionClient>();

    pubSubNode = rclcpp::Node::make_shared("test_publisher");

    odom_subscriber = pubSubNode->create_subscription<nav_msgs::msg::Odometry>(
        "odom", 10,
        std::bind(&GoToWaypointTestFixture::odom_callback, this, _1));
    pos_current_x_ = 0.0;
    pos_current_y_ = 0.0;
    pos_current_yaw_ = 0.0;
    index_check_ = 0.0;
    check_recorrido_ = true;
    timer_ = pubSubNode->create_wall_timer(
        std::chrono::milliseconds(5000),
        std::bind(&GoToWaypointTestFixture::CheckRecorrido, this));
  }

  double GoToWaypointTestDistance(MyEnumTest);

protected:
  std::shared_ptr<rclcpp::Node> pubSubNode;
  std::shared_ptr<rclcpp::Node> actionServerCtrlNode;
  std::shared_ptr<MyActionClient> actionClientCtrlNode;

private:
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr odom) {
    pos_current_x_ = odom->pose.pose.position.x;
    pos_current_y_ = odom->pose.pose.position.y;

    tf2::Quaternion q(
        odom->pose.pose.orientation.x, odom->pose.pose.orientation.y,
        odom->pose.pose.orientation.z, odom->pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch;
    m.getRPY(roll, pitch, pos_current_yaw_);
  }
  void CheckRecorrido() {
    double recorrido = sqrt(pow(pos_current_x_ - pos_last_x_, 2) +
                            pow(pos_current_y_ - pos_last_y_, 2));
    RCLCPP_ERROR(actionServerCtrlNode->get_logger(), "Verificando recorrido");
    pos_last_x_ = pos_current_x_;
    pos_last_y_ = pos_current_y_;
    if (recorrido > ACCEPT_DIST) {

      check_recorrido_ = true;
    } else {
      index_check_++;
      check_recorrido_ = true;
      if (index_check_ == 5) {
        RCLCPP_ERROR(actionServerCtrlNode->get_logger(),
                     "Se imterrumpio el test");
        check_recorrido_ = false;
      }
      RCLCPP_ERROR(actionServerCtrlNode->get_logger(),
                   "No se alcanza avanzar lo suficiente");
    }
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr data_publisher_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber;

  double received_data_;
  bool action_success_;
  double pos_current_x_;
  double pos_current_y_;
  double pos_current_yaw_;
  double pos_last_x_;
  double pos_last_y_;
  double pos_last_yaw_;
  bool check_recorrido_;
  double index_check_;
};
double
GoToWaypointTestFixture::GoToWaypointTestDistance(MyEnumTest type_prueba) {
  float current_pos_x = 0;
  float current_pos_y = 0;
  double current_pos_yaw = 0;
  float goal_x = 0;
  float goal_y = 0;
  float goal_yaw = 0;

  RCLCPP_DEBUG(actionServerCtrlNode->get_logger(), "Iniciando testeo");
  if (type_prueba == MyEnumTest::TEST_DISTANCE) {
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(actionClientCtrlNode);
    while (!actionClientCtrlNode->is_goal_done() && check_recorrido_) {
      rclcpp::spin_some(pubSubNode);
      rclcpp::spin_some(actionServerCtrlNode);
      executor.spin_once();
    }
    current_pos_x = actionClientCtrlNode->get_pos_current_x();
    current_pos_y = actionClientCtrlNode->get_pos_current_y();
    current_pos_yaw = actionClientCtrlNode->get_pos_current_yaw();
    goal_x = actionClientCtrlNode->get_goal_x();
    goal_y = actionClientCtrlNode->get_goal_y();

    RCLCPP_ERROR(
        actionServerCtrlNode->get_logger(),
        "current_pos_x %.3f  current_pos_y %.3f goal_x %.3f  goal_y %.3f",
        current_pos_x, current_pos_y, goal_x, goal_y);
    double error_yaw = atan2(goal_y - current_pos_y, goal_x - current_pos_x);
    RCLCPP_ERROR(actionServerCtrlNode->get_logger(),
                 "Error de distancia final %.3f", error_yaw);

    double error =
        sqrt(pow(goal_x - current_pos_x, 2) + pow(goal_y - current_pos_y, 2));
    return abs(error);
  }
  if (type_prueba == MyEnumTest::TEST_ORIENTATION) {
    current_pos_x = actionClientCtrlNode->get_pos_current_x();
    current_pos_y = actionClientCtrlNode->get_pos_current_y();
    current_pos_yaw = actionClientCtrlNode->get_pos_current_yaw();
    goal_x = actionClientCtrlNode->get_goal_x();
    goal_y = actionClientCtrlNode->get_goal_y();
    goal_yaw = actionClientCtrlNode->get_goal_yaw();
    RCLCPP_ERROR(actionServerCtrlNode->get_logger(),
                 "Current pos yaw : %.3f  goal yaw:  %.3f", current_pos_yaw,
                 goal_yaw);
    double error_yaw = goal_yaw - current_pos_yaw;
    RCLCPP_ERROR(actionServerCtrlNode->get_logger(),
                 "Error de orientation final %.3f", error_yaw);

    if (error_yaw > PI) {
      error_yaw = error_yaw - 2 * PI;
    }
    if (error_yaw < -PI) {
      error_yaw = error_yaw + 2 * PI;
    }

    return abs(error_yaw);
  }
}

TEST_F(GoToWaypointTestFixture, DistanceTest) {
  EXPECT_LT(GoToWaypointTestDistance(MyEnumTest::TEST_DISTANCE),
            ERROR_DIST_ACCEPT);
  EXPECT_LT(GoToWaypointTestDistance(MyEnumTest::TEST_ORIENTATION),
            ERROR_YAW_ACCEPT);
}
// TEST_F(GoToWaypointTestFixture, OrientationTest) {
//  EXPECT_LT(, ERROR_YAW_ACCEPT);
//}