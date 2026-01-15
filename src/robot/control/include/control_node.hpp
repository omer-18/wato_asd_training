#ifndef CONTROL_NODE_HPP_
#define CONTROL_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "control_core.hpp"

/**
 * ControlNode: ROS2 node that implements Pure Pursuit Control for path following
 * 
 * This node subscribes to:
 * - /path: Receives the global path as a series of waypoints from the planner
 * - /odom/filtered: Tracks the robot's current position and orientation
 * 
 * This node publishes:
 * - /cmd_vel: Outputs velocity commands (linear and angular velocities) to move the robot
 * 
 * The node uses a timer to update velocity commands at a fixed rate (10 Hz) for
 * smooth and consistent control.
 */
class ControlNode : public rclcpp::Node {
  public:
    ControlNode();

  private:
    // Core Pure Pursuit Control logic
    robot::ControlCore control_;

    // ROS2 subscriber: receives planned path from /path topic
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    
    // ROS2 subscriber: receives robot position from /odom/filtered topic
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    
    // ROS2 subscriber: receives costmap to detect obstacles
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
    
    // ROS2 publisher: publishes velocity commands to /cmd_vel topic
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

    // Timer: periodically updates velocity commands (10 Hz = 100ms)
    rclcpp::TimerBase::SharedPtr control_timer_;

    // Data storage
    nav_msgs::msg::Path current_path_;      // Current path to follow
    nav_msgs::msg::Odometry robot_odom_;    // Current robot odometry
    nav_msgs::msg::OccupancyGrid::SharedPtr current_costmap_;  // Current costmap for obstacle detection
    bool path_received_;                    // Flag indicating if path has been received
    bool odom_received_;                    // Flag indicating if odometry has been received
    bool costmap_received_;                 // Flag indicating if costmap has been received

    /**
     * Callback function called when a new path message is received
     * 
     * @param path Shared pointer to the received Path message
     */
    void pathCallback(const nav_msgs::msg::Path::SharedPtr path);

    /**
     * Callback function called when a new odometry message is received
     * 
     * @param odom Shared pointer to the received Odometry message
     */
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom);

    /**
     * Callback function called when a new costmap message is received
     * 
     * @param costmap Shared pointer to the received OccupancyGrid message
     */
    void costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr costmap);

    /**
     * Timer callback: Periodically computes and publishes velocity commands
     * 
     * This function runs at 10 Hz and:
     * 1. Checks if path and odometry data are available
     * 2. Finds the lookahead point on the path
     * 3. Computes velocity commands using Pure Pursuit Control
     * 4. Publishes the commands to move the robot
     */
    void controlLoop();
};

#endif
