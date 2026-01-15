#ifndef PLANNER_NODE_HPP_
#define PLANNER_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"

#include "planner_core.hpp"

/**
 * PlannerNode: ROS2 node that plans paths from robot position to goal using A* algorithm
 * 
 * This node implements a state machine with two states:
 * - WAITING_FOR_GOAL: Waiting for a goal point to be set
 * - WAITING_FOR_ROBOT_TO_REACH_GOAL: Planning and monitoring progress toward goal
 * 
 * The planner uses the A* algorithm on the occupancy grid to find optimal paths
 * that avoid obstacles and minimize travel distance.
 */
class PlannerNode : public rclcpp::Node {
  public:
    PlannerNode();

  private:
    // Core planning logic (A* algorithm implementation)
    robot::PlannerCore planner_;

    // State machine for planner behavior
    enum class State {
        WAITING_FOR_GOAL,                    // Waiting for a goal to be set
        WAITING_FOR_ROBOT_TO_REACH_GOAL      // Planning and tracking progress
    };
    State state_;

    // ROS2 subscriber: receives global map from /map topic
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    
    // ROS2 subscriber: receives goal point from /goal_point topic
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr goal_sub_;
    
    // ROS2 subscriber: receives robot position from /odom/filtered topic
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    
    // ROS2 publisher: publishes planned path to /path topic
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

    // Timer: periodically checks if goal is reached or replanning is needed
    // Runs every 500ms to monitor progress and trigger replanning if needed
    rclcpp::TimerBase::SharedPtr timer_;

    // Data storage
    nav_msgs::msg::OccupancyGrid current_map_;      // Current global map
    geometry_msgs::msg::PointStamped goal_;         // Current goal point
    geometry_msgs::msg::Pose robot_pose_;           // Current robot pose
    bool goal_received_;                            // Flag indicating if goal has been received

    // Goal reached threshold (distance in meters to consider goal reached)
    static constexpr double GOAL_TOLERANCE = 0.5;

    /**
     * Callback function called when a new map message is received
     * 
     * If we're actively planning (WAITING_FOR_ROBOT_TO_REACH_GOAL), replan
     * when the map updates, as new obstacles may have been discovered.
     * 
     * @param map Shared pointer to the received OccupancyGrid message
     */
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr map);

    /**
     * Callback function called when a new goal point is received
     * 
     * Transitions to WAITING_FOR_ROBOT_TO_REACH_GOAL state and triggers
     * path planning to the new goal.
     * 
     * @param goal Shared pointer to the received PointStamped message
     */
    void goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr goal);

    /**
     * Callback function called when a new odometry message is received
     * 
     * Updates the robot's current position for path planning and progress tracking.
     * 
     * @param odom Shared pointer to the received Odometry message
     */
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom);

    /**
     * Timer callback: Periodically checks goal status and triggers replanning
     * 
     * This function runs every 500ms and:
     * 1. Checks if the goal has been reached
     * 2. Triggers replanning if needed (e.g., robot not making progress)
     */
    void timerCallback();

    /**
     * Check if the robot has reached the goal point
     * 
     * @return true if robot is within GOAL_TOLERANCE distance of goal
     */
    bool goalReached() const;

    /**
     * Plan a path from robot's current position to the goal using A* algorithm
     * 
     * This function:
     * 1. Converts robot and goal positions to grid coordinates
     * 2. Calls the A* algorithm in PlannerCore
     * 3. Publishes the resulting path
     */
    void planPath();
};

#endif
