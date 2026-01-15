#include <chrono>
#include <memory>
#include <cmath>

#include "planner_node.hpp"

/**
 * Constructor: Initializes the PlannerNode with subscribers, publisher, and timer
 * 
 * Sets up:
 * - Subscriber to /map topic for global map
 * - Subscriber to /goal_point topic for goal points
 * - Subscriber to /odom/filtered topic for robot position
 * - Publisher to /path topic for planned paths
 * - Timer for monitoring progress and replanning
 */
PlannerNode::PlannerNode() 
: Node("planner"), 
  planner_(robot::PlannerCore(this->get_logger())),
  state_(State::WAITING_FOR_GOAL),
  goal_received_(false)
{
    // Create subscriber to receive global map from map_memory node
    // Queue size of 10 allows buffering if processing is slow
    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/map", 
        10, 
        std::bind(&PlannerNode::mapCallback, this, std::placeholders::_1)
    );

    // Create subscriber to receive goal points (set via Foxglove or other interface)
    goal_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
        "/goal_point", 
        10, 
        std::bind(&PlannerNode::goalCallback, this, std::placeholders::_1)
    );

    // Create subscriber to receive robot odometry (position and orientation)
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom/filtered", 
        10, 
        std::bind(&PlannerNode::odomCallback, this, std::placeholders::_1)
    );

    // Create publisher to send planned paths to the /path topic
    // The control node will subscribe to this to follow the path
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);

    // Create timer to check goal status and trigger replanning every 500ms
    // This allows the planner to react to changes and monitor progress
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(500),
        std::bind(&PlannerNode::timerCallback, this)
    );

    RCLCPP_INFO(this->get_logger(), 
                "Planner node initialized. Listening to /map, /goal_point, and /odom/filtered. Publishing to /path");
}

/**
 * Callback function triggered when a new map message arrives
 */
void PlannerNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr map) {
    // Store the latest map
    current_map_ = *map;

    // If we're actively planning, replan when the map updates
    // This is important because new obstacles may have been discovered,
    // which could invalidate the current path
    if (state_ == State::WAITING_FOR_ROBOT_TO_REACH_GOAL) {
        RCLCPP_DEBUG(this->get_logger(), "Map updated, replanning...");
        planPath();
    }
}

/**
 * Callback function triggered when a new goal point is received
 */
void PlannerNode::goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr goal) {
    // Store the goal point
    goal_ = *goal;
    goal_received_ = true;

    // Transition to planning state
    state_ = State::WAITING_FOR_ROBOT_TO_REACH_GOAL;

    RCLCPP_INFO(this->get_logger(), 
                "New goal received: (%.2f, %.2f) in frame '%s'. Planning path...",
                goal->point.x, goal->point.y, goal->header.frame_id.c_str());

    // Immediately plan a path to the new goal
    planPath();
}

/**
 * Callback function triggered when a new odometry message arrives
 */
void PlannerNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom) {
    // Update robot's current pose (position and orientation)
    robot_pose_ = odom->pose.pose;
}

/**
 * Timer callback: Periodically checks goal status and triggers replanning
 */
void PlannerNode::timerCallback() {
    // Only check if we're actively planning
    if (state_ == State::WAITING_FOR_ROBOT_TO_REACH_GOAL) {
        // Check if the goal has been reached
        if (goalReached()) {
            RCLCPP_INFO(this->get_logger(), "Goal reached! Waiting for new goal.");
            state_ = State::WAITING_FOR_GOAL;
            goal_received_ = false;
            
            // Publish an empty path to stop the robot
            nav_msgs::msg::Path empty_path;
            empty_path.header.stamp = this->get_clock()->now();
            empty_path.header.frame_id = "sim_world";  // Use sim_world to match odometry frame
            path_pub_->publish(empty_path);
        } else {
            // Robot hasn't reached goal yet - periodically replan to account for
            // any deviations or new obstacles discovered
            // Only replan if we have a valid map and goal
            if (!current_map_.data.empty() && goal_received_) {
                RCLCPP_DEBUG(this->get_logger(), "Replanning due to timer...");
                planPath();
            }
        }
    }
}

/**
 * Check if the robot has reached the goal point
 */
bool PlannerNode::goalReached() const {
    // Calculate Euclidean distance from robot to goal
    double dx = goal_.point.x - robot_pose_.position.x;
    double dy = goal_.point.y - robot_pose_.position.y;
    double distance = std::sqrt(dx * dx + dy * dy);

    // Goal is reached if within tolerance (0.5 meters)
    return distance < GOAL_TOLERANCE;
}

/**
 * Plan a path from robot's current position to the goal using A* algorithm
 */
void PlannerNode::planPath() {
    // Check prerequisites: need both a goal and a valid map
    if (!goal_received_ || current_map_.data.empty()) {
        RCLCPP_WARN(this->get_logger(), "Cannot plan path: Missing map or goal!");
        return;
    }

    // Extract robot and goal positions
    double robot_x = robot_pose_.position.x;
    double robot_y = robot_pose_.position.y;
    double goal_x = goal_.point.x;
    double goal_y = goal_.point.y;

    RCLCPP_DEBUG(this->get_logger(), 
                 "Planning path from (%.2f, %.2f) to (%.2f, %.2f)",
                 robot_x, robot_y, goal_x, goal_y);

    // Use the core planner to compute the path using A* algorithm
    nav_msgs::msg::Path path = planner_.planPath(
        current_map_,
        robot_x, robot_y,
        goal_x, goal_y
    );

    // Set path header
    path.header.stamp = this->get_clock()->now();
    path.header.frame_id = "sim_world";  // Use sim_world to match odometry frame

    // Publish the planned path
    path_pub_->publish(path);

    if (path.poses.empty()) {
        RCLCPP_WARN(this->get_logger(), "No path found! Goal may be unreachable.");
    } else {
        RCLCPP_INFO(this->get_logger(), "Path planned with %zu waypoints", path.poses.size());
    }
}

int main(int argc, char ** argv)
{
    // Initialize ROS2
    rclcpp::init(argc, argv);
    
    // Create and spin the planner node (this blocks until shutdown)
    rclcpp::spin(std::make_shared<PlannerNode>());
    
    // Cleanup ROS2
    rclcpp::shutdown();
    return 0;
}
