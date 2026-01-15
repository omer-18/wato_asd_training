#include <chrono>
#include <memory>
#include <cmath>

#include "map_memory_node.hpp"

/**
 * Constructor: Initializes the MapMemoryNode with subscribers, publisher, and timer
 * 
 * Sets up:
 * - Subscriber to /costmap topic for local costmaps
 * - Subscriber to /odom/filtered topic for robot position
 * - Publisher to /map topic for global map output
 * - Timer to limit map update frequency (optimization)
 */
MapMemoryNode::MapMemoryNode() 
: Node("map_memory"), map_memory_(robot::MapMemoryCore(this->get_logger()))
{
    // Create subscriber to receive local costmaps from the costmap node
    // Queue size of 10 allows buffering if processing is slow
    costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/costmap", 
        10, 
        std::bind(&MapMemoryNode::costmapCallback, this, std::placeholders::_1)
    );

    // Create subscriber to receive robot odometry (position and orientation)
    // This tells us where the robot is in the global frame
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom/filtered", 
        10, 
        std::bind(&MapMemoryNode::odomCallback, this, std::placeholders::_1)
    );

    // Create publisher to send the global map to the /map topic
    // The planner node will subscribe to this to plan paths
    map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);

    // Create timer to check for map updates every 1 second
    // This limits update frequency for optimization - we don't need to update
    // the map every time we receive a costmap, only when the robot has moved
    timer_ = this->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&MapMemoryNode::timerCallback, this)
    );

    // Publish an initial empty map so the planner can start planning immediately
    // This is important - without this, the planner won't be able to plan
    // until the robot moves and updates the map
    nav_msgs::msg::OccupancyGrid initial_map = map_memory_.getGlobalMap();
    map_pub_->publish(initial_map);

    RCLCPP_INFO(this->get_logger(), 
                "Map memory node initialized. Listening to /costmap and /odom/filtered, publishing to /map");
}

/**
 * Callback function triggered when a new costmap message arrives
 * 
 * Stores the latest costmap for later integration into the global map.
 * The actual integration happens in the timer callback to limit frequency.
 * 
 * @param costmap Shared pointer to the received costmap (local occupancy grid)
 */
void MapMemoryNode::costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr costmap) {
    // Store the latest costmap in the core for processing
    map_memory_.storeLatestCostmap(costmap);
    
    RCLCPP_DEBUG(this->get_logger(), "Received costmap: %dx%d cells", 
                 costmap->info.width, costmap->info.height);
}

/**
 * Callback function triggered when a new odometry message arrives
 * 
 * Tracks the robot's position and calculates distance traveled since
 * the last map update. If the robot has moved 1.5 meters or more,
 * marks that a map update should be performed.
 * 
 * @param odom Shared pointer to the received odometry message
 */
void MapMemoryNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom) {
    // Extract robot position from odometry message
    double x = odom->pose.pose.position.x;
    double y = odom->pose.pose.position.y;
    
    // Extract robot orientation (yaw) from quaternion
    // We need this for proper coordinate transformation when integrating costmaps
    double yaw = extractYaw(odom->pose.pose.orientation);

    // Check if robot has moved far enough to trigger a map update
    // This prevents excessive map updates when the robot is stationary
    bool should_update = map_memory_.checkRobotMovement(x, y, yaw);

    if (should_update) {
        RCLCPP_DEBUG(this->get_logger(), 
                     "Robot moved 1.5m+ since last update. Map update will be triggered.");
    }
}

/**
 * Extract yaw angle from quaternion orientation
 * Helper function to convert quaternion to yaw (rotation around z-axis)
 */
double MapMemoryNode::extractYaw(const geometry_msgs::msg::Quaternion& quat) {
    // Convert quaternion to yaw (rotation around z-axis)
    // Formula: yaw = atan2(2*(w*z + x*y), 1 - 2*(y^2 + z^2))
    // For ROS2, the quaternion is (x, y, z, w)
    double w = quat.w;
    double x = quat.x;
    double y = quat.y;
    double z = quat.z;

    // Yaw angle from quaternion
    double siny_cosp = 2.0 * (w * z + x * y);
    double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
    double yaw = std::atan2(siny_cosp, cosy_cosp);

    return yaw;
}

/**
 * Timer callback: Periodically checks and performs map updates
 * 
 * This function runs every 1 second and:
 * 1. Checks if robot has moved 1.5 meters since last update
 * 2. Checks if a new costmap has been received
 * 3. If both conditions are met, integrates the costmap into global map
 * 4. Publishes the updated global map
 * 
 * This approach optimizes performance by limiting update frequency.
 * Also publishes the map periodically to ensure planner always has latest data.
 */
void MapMemoryNode::timerCallback() {
    // Check if conditions are met for a map update
    if (map_memory_.shouldUpdateMap()) {
        // Integrate the latest costmap into the global map
        map_memory_.integrateCostmap();
        
        // Get the updated global map
        nav_msgs::msg::OccupancyGrid global_map = map_memory_.getGlobalMap();
        
        // Update timestamp to current time
        global_map.header.stamp = this->get_clock()->now();
        
        // Publish the updated global map
        map_pub_->publish(global_map);
        
        RCLCPP_DEBUG(this->get_logger(), 
                    "Published updated global map: %dx%d cells", 
                    global_map.info.width, global_map.info.height);
    } else {
        // Even if no update, publish the current map periodically
        // This ensures the planner always has the latest map data
        nav_msgs::msg::OccupancyGrid global_map = map_memory_.getGlobalMap();
        global_map.header.stamp = this->get_clock()->now();
        map_pub_->publish(global_map);
    }
}

int main(int argc, char ** argv)
{
    // Initialize ROS2
    rclcpp::init(argc, argv);
    
    // Create and spin the map memory node (this blocks until shutdown)
    rclcpp::spin(std::make_shared<MapMemoryNode>());
    
    // Cleanup ROS2
    rclcpp::shutdown();
    return 0;
}
