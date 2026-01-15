#include <memory>
 
#include "costmap_node.hpp"

/**
 * Constructor: Initializes the CostmapNode with subscribers and publishers
 * 
 * Sets up:
 * - Subscriber to /lidar topic for laser scan data
 * - Publisher to /costmap topic for occupancy grid output
 */
CostmapNode::CostmapNode()
: Node("costmap"), costmap_(robot::CostmapCore(this->get_logger()))
{
    // Create subscriber to receive laser scan data from the /lidar topic
    // Queue size of 10 means we can buffer up to 10 messages if processing is slow
    lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/lidar", 
        10, 
        std::bind(&CostmapNode::laserCallback, this, std::placeholders::_1)
    );

    // Create publisher to send costmap (occupancy grid) to the /costmap topic
    // Queue size of 10 allows buffering of published messages
    costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);

    RCLCPP_INFO(this->get_logger(), "Costmap node initialized. Listening to /lidar, publishing to /costmap");
}

/**
 * Callback function triggered when a new laser scan message arrives
 * 
 * This function:
 * 1. Processes the laser scan data using CostmapCore
 * 2. Converts the scan into an occupancy grid (costmap)
 * 3. Publishes the resulting costmap to the /costmap topic
 * 
 * @param scan Shared pointer to the received LaserScan message containing
 *             range measurements from the lidar sensor
 */
void CostmapNode::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
    // Process the laser scan and generate a costmap using the core logic
    nav_msgs::msg::OccupancyGrid costmap = costmap_.processLaserScan(scan);
    
    // Publish the generated costmap so other nodes (like map_memory) can use it
    costmap_pub_->publish(costmap);
    
    RCLCPP_DEBUG(this->get_logger(), "Published costmap with %dx%d cells", 
                 costmap.info.width, costmap.info.height);
}
 
int main(int argc, char ** argv)
{
    // Initialize ROS2
    rclcpp::init(argc, argv);
    
    // Create and spin the costmap node (this blocks until shutdown)
    rclcpp::spin(std::make_shared<CostmapNode>());
    
    // Cleanup ROS2
    rclcpp::shutdown();
    return 0;
}
