#include <chrono>
#include <memory>
#include <cmath>

#include "control_node.hpp"

/**
 * Constructor: Initializes the ControlNode with subscribers, publisher, and timer
 * 
 * Sets up:
 * - Subscriber to /path topic for planned paths
 * - Subscriber to /odom/filtered topic for robot position
 * - Publisher to /cmd_vel topic for velocity commands
 * - Timer for control loop (10 Hz = 100ms)
 */
ControlNode::ControlNode() 
: Node("control"),
  control_(robot::ControlCore(this->get_logger())),
  path_received_(false),
  odom_received_(false)
{
  // Declare parameters with default values
  this->declare_parameter<double>("lookahead_distance", 1.0);
  this->declare_parameter<double>("goal_tolerance", 0.1);
  this->declare_parameter<double>("linear_speed", 0.5);
  this->declare_parameter<double>("obstacle_check_distance", 0.5);
  this->declare_parameter<int>("obstacle_threshold", 50);

  // Get parameters
  double lookahead_distance = this->get_parameter("lookahead_distance").as_double();
  double goal_tolerance = this->get_parameter("goal_tolerance").as_double();
  double linear_speed = this->get_parameter("linear_speed").as_double();
  double obstacle_check_distance = this->get_parameter("obstacle_check_distance").as_double();
  int obstacle_threshold = this->get_parameter("obstacle_threshold").as_int();

  // Reinitialize control core with parameters
  control_ = robot::ControlCore(this->get_logger(), lookahead_distance, goal_tolerance, linear_speed);
  control_.setObstacleParams(obstacle_check_distance, obstacle_threshold);
  
  // Initialize flags
  costmap_received_ = false;

  // Create subscriber to receive planned paths from planner node
  path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
      "/path", 
      10, 
      std::bind(&ControlNode::pathCallback, this, std::placeholders::_1)
  );

  // Create subscriber to receive robot odometry (position and orientation)
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom/filtered", 
      10, 
      std::bind(&ControlNode::odomCallback, this, std::placeholders::_1)
  );

  // Create subscriber to receive costmap for obstacle detection
  costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/costmap",
      10,
      std::bind(&ControlNode::costmapCallback, this, std::placeholders::_1)
  );

  // Create publisher to send velocity commands to the /cmd_vel topic
  // The robot's motor controller will subscribe to this to move the robot
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

  // Create timer to update velocity commands at 10 Hz (100ms intervals)
  // This ensures smooth and consistent control
  control_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&ControlNode::controlLoop, this)
  );

  RCLCPP_INFO(this->get_logger(), 
              "Control node initialized. Listening to /path, /odom/filtered, and /costmap. Publishing to /cmd_vel");
}

/**
 * Callback function triggered when a new path message arrives
 */
void ControlNode::pathCallback(const nav_msgs::msg::Path::SharedPtr path) {
  // Store the latest path
  current_path_ = *path;
  path_received_ = true;

  if (path->poses.empty()) {
    RCLCPP_DEBUG(this->get_logger(), "Received empty path. Stopping robot.");
    // Publish zero velocity to stop the robot
    geometry_msgs::msg::Twist stop_cmd;
    stop_cmd.linear.x = 0.0;
    stop_cmd.angular.z = 0.0;
    cmd_vel_pub_->publish(stop_cmd);
  } else {
    RCLCPP_DEBUG(this->get_logger(), "Received new path with %zu waypoints", path->poses.size());
  }
}

/**
 * Callback function triggered when a new odometry message arrives
 */
void ControlNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom) {
  // Store the latest odometry
  robot_odom_ = *odom;
  odom_received_ = true;
}

/**
 * Callback function triggered when a new costmap message arrives
 */
void ControlNode::costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr costmap) {
  // Store the latest costmap for obstacle detection
  current_costmap_ = costmap;
  costmap_received_ = true;
}

/**
 * Timer callback: Periodically computes and publishes velocity commands
 */
void ControlNode::controlLoop() {
  // Skip control if no path or odometry data is available
  if (!path_received_ || !odom_received_) {
    return;
  }

  // If path is empty, stop the robot
  if (current_path_.poses.empty()) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                         "Path is empty. Stopping robot.");
    geometry_msgs::msg::Twist stop_cmd;
    stop_cmd.linear.x = 0.0;
    stop_cmd.angular.z = 0.0;
    cmd_vel_pub_->publish(stop_cmd);
    return;
  }

  // Extract robot position and orientation
  double robot_x = robot_odom_.pose.pose.position.x;
  double robot_y = robot_odom_.pose.pose.position.y;
  double robot_yaw = robot::ControlCore::extractYaw(robot_odom_.pose.pose.orientation);

  // Check if goal is reached
  if (control_.goalReached(current_path_, robot_x, robot_y)) {
    RCLCPP_INFO(this->get_logger(), "Goal reached! Stopping robot.");
    // Publish zero velocity to stop the robot
    geometry_msgs::msg::Twist stop_cmd;
    stop_cmd.linear.x = 0.0;
    stop_cmd.angular.z = 0.0;
    cmd_vel_pub_->publish(stop_cmd);
    return;
  }

  // Find the lookahead point on the path
  auto lookahead_point = control_.findLookaheadPoint(
      current_path_,
      robot_x,
      robot_y,
      robot_yaw
  );

  if (!lookahead_point) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                         "Could not find lookahead point. Stopping robot.");
    geometry_msgs::msg::Twist stop_cmd;
    stop_cmd.linear.x = 0.0;
    stop_cmd.angular.z = 0.0;
    cmd_vel_pub_->publish(stop_cmd);
    return;
  }

  // Debug: Log path and lookahead info (only occasionally to avoid spam)
  static int debug_counter = 0;
  if (debug_counter++ % 50 == 0) {  // Log every 5 seconds at 10Hz
    if (!current_path_.poses.empty()) {
      const auto& goal = current_path_.poses.back();
      RCLCPP_INFO(this->get_logger(), 
                  "Robot: (%.2f, %.2f), Goal: (%.2f, %.2f), Lookahead: (%.2f, %.2f), Path size: %zu",
                  robot_x, robot_y,
                  goal.pose.position.x, goal.pose.position.y,
                  lookahead_point->pose.position.x, lookahead_point->pose.position.y,
                  current_path_.poses.size());
    }
  }

  // Compute velocity command using Pure Pursuit Control
  // Pass costmap if available for obstacle detection
  nav_msgs::msg::OccupancyGrid::SharedPtr costmap_ptr = costmap_received_ ? current_costmap_ : nullptr;
  geometry_msgs::msg::Twist cmd_vel = control_.computeVelocity(
      *lookahead_point,
      robot_x,
      robot_y,
      robot_yaw,
      costmap_ptr
  );

  // Publish the velocity command
  cmd_vel_pub_->publish(cmd_vel);

  RCLCPP_DEBUG(this->get_logger(), 
               "Published cmd_vel: linear=%.2f m/s, angular=%.2f rad/s",
               cmd_vel.linear.x, cmd_vel.angular.z);
}

int main(int argc, char ** argv)
{
  // Initialize ROS2
  rclcpp::init(argc, argv);
  
  // Create and spin the control node (this blocks until shutdown)
  rclcpp::spin(std::make_shared<ControlNode>());
  
  // Cleanup ROS2
  rclcpp::shutdown();
  return 0;
}
