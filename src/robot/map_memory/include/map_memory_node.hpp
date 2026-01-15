#ifndef MAP_MEMORY_NODE_HPP_
#define MAP_MEMORY_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/quaternion.hpp"

#include "map_memory_core.hpp"

/**
 * MapMemoryNode: ROS2 node that builds a global map from local costmaps
 * 
 * This node subscribes to local costmaps from the costmap node and odometry
 * data to stitch together a global map of the environment. The global map
 * accumulates knowledge as the robot explores, allowing it to remember
 * previously detected obstacles and plan better paths.
 */
class MapMemoryNode : public rclcpp::Node {
  public:
    MapMemoryNode();

  private:
    // Core map memory processing logic
    robot::MapMemoryCore map_memory_;

    // ROS2 subscriber: receives local costmaps from /costmap topic
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
    
    // ROS2 subscriber: receives robot position/pose from /odom/filtered topic
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    
    // ROS2 publisher: publishes global map to /map topic (for planner to use)
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;

    // Timer: limits map update frequency for optimization
    // Updates are checked every 1 second, but only performed if robot moved 1.5m
    rclcpp::TimerBase::SharedPtr timer_;

    /**
     * Callback function called when a new costmap message is received
     * Stores the latest costmap for integration into the global map
     * 
     * @param costmap Shared pointer to the received OccupancyGrid message
     */
    void costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr costmap);

    /**
     * Callback function called when a new odometry message is received
     * Tracks robot movement and determines when to update the global map
     * 
     * @param odom Shared pointer to the received Odometry message
     */
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom);

    /**
     * Timer callback: Periodically checks if map should be updated
     * 
     * This function is called every 1 second by the timer. It checks if:
     * 1. The robot has moved at least 1.5 meters since last update
     * 2. A new costmap has been received
     * 
     * If both conditions are met, it integrates the costmap into the global map
     * and publishes the updated map.
     */
    void timerCallback();
    
    /**
     * Extract yaw angle from quaternion orientation
     * Helper function to convert quaternion to yaw (rotation around z-axis)
     * 
     * @param quat Quaternion message
     * @return Yaw angle in radians, range [-pi, pi]
     */
    double extractYaw(const geometry_msgs::msg::Quaternion& quat);
};

#endif
