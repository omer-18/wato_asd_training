#ifndef COSTMAP_NODE_HPP_
#define COSTMAP_NODE_HPP_
 
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
 
#include "costmap_core.hpp"
 
/**
 * CostmapNode: ROS2 node that processes laser scan data to create a costmap
 * 
 * This node subscribes to laser scan data from the /lidar topic and converts
 * it into an occupancy grid (costmap) that represents obstacles in the robot's
 * local environment. The costmap is published to /costmap for use by other nodes.
 */
class CostmapNode : public rclcpp::Node {
public:
    CostmapNode();
 
private:
    // Core costmap processing logic
    robot::CostmapCore costmap_;

    // ROS2 subscriber: receives laser scan data from /lidar topic
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
    
    // ROS2 publisher: publishes costmap (occupancy grid) to /costmap topic
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub_;

    /**
     * Callback function called when a new laser scan message is received
     * Processes the laser scan data and publishes the resulting costmap
     * 
     * @param scan Shared pointer to the received LaserScan message
     */
    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan);
};
 
#endif
