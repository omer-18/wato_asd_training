#ifndef MAP_MEMORY_CORE_HPP_
#define MAP_MEMORY_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include <memory>
#include <cmath>

namespace robot
{

/**
 * MapMemoryCore: Core logic for building a global map from local costmaps
 * 
 * This class maintains a global occupancy grid that accumulates knowledge
 * from local costmaps as the robot explores. It transforms costmaps from
 * the robot's local frame to the global frame and merges them using linear
 * fusion (new known values overwrite old, unknown values are preserved).
 */
class MapMemoryCore {
  public:
    /**
     * Constructor: Initializes the global map and tracking variables
     * 
     * @param logger ROS2 logger for debug/info messages
     */
    explicit MapMemoryCore(const rclcpp::Logger& logger);

    /**
     * Store the latest costmap received from the costmap node
     * 
     * @param costmap Shared pointer to the latest costmap message
     */
    void storeLatestCostmap(const nav_msgs::msg::OccupancyGrid::SharedPtr costmap);

    /**
     * Check if the robot has moved far enough to trigger a map update
     * 
     * Calculates the Euclidean distance from the last update position.
     * Returns true if the robot has moved at least 1.5 meters.
     * Also updates the robot's current orientation.
     * 
     * @param x Current robot x position in global frame
     * @param y Current robot y position in global frame
     * @param yaw Current robot yaw (orientation) in radians
     * @return true if robot has moved >= 1.5m since last update
     */
    bool checkRobotMovement(double x, double y, double yaw);

    /**
     * Check if conditions are met to update the global map
     * 
     * @return true if robot has moved 1.5m+ AND a new costmap has been received
     */
    bool shouldUpdateMap();

    /**
     * Integrate the latest costmap into the global map
     * 
     * This function:
     * 1. Transforms the costmap from robot frame to global frame
     * 2. Merges the costmap into the global map using linear fusion
     * 3. Updates the last update position
     */
    void integrateCostmap();

    /**
     * Get the current global map
     * 
     * @return OccupancyGrid message containing the global map
     */
    nav_msgs::msg::OccupancyGrid getGlobalMap();

  private:
    rclcpp::Logger logger_;

    // Global map parameters
    double map_resolution_;      // Resolution of global map (should match costmap: 0.1 m/cell)
    int map_width_;              // Width of global map in cells (1000 = 100m at 0.1m resolution)
    int map_height_;             // Height of global map in cells (1000 = 100m at 0.1m resolution)
    double map_origin_x_;        // X origin of global map in meters (centered at 0, so -50m)
    double map_origin_y_;        // Y origin of global map in meters (centered at 0, so -50m)

    // Global map storage (1D vector representing 2D grid: grid_[y * width + x])
    std::vector<int8_t> global_grid_;

    // Robot position and orientation tracking
    double last_update_x_;       // X position of last map update
    double last_update_y_;       // Y position of last map update
    double current_x_;           // Current robot x position
    double current_y_;            // Current robot y position
    double current_yaw_;         // Current robot yaw (orientation) in radians
    const double distance_threshold_;  // Distance threshold for updates (1.5 meters)

    // Latest costmap storage
    nav_msgs::msg::OccupancyGrid::SharedPtr latest_costmap_;

    // Flags
    bool costmap_received_;      // True if a costmap has been received
    bool robot_moved_enough_;    // True if robot has moved >= distance_threshold

    /**
     * Initialize the global map with default values
     * Sets up a large grid (e.g., 100m x 100m) centered at origin
     */
    void initializeGlobalMap();

    /**
     * Convert world coordinates (meters) to grid cell indices
     * 
     * @param x_world X coordinate in world frame (meters)
     * @param y_world Y coordinate in world frame (meters)
     * @param x_grid Output: X index in grid
     * @param y_grid Output: Y index in grid
     * @return true if coordinates are within map bounds
     */
    bool worldToGrid(double x_world, double y_world, int& x_grid, int& y_grid) const;

    /**
     * Convert grid cell indices to world coordinates (meters)
     * 
     * @param x_grid X index in grid
     * @param y_grid Y index in grid
     * @param x_world Output: X coordinate in world frame (meters)
     * @param y_world Output: Y coordinate in world frame (meters)
     */
    void gridToWorld(int x_grid, int y_grid, double& x_world, double& y_world) const;

    /**
     * Transform a point from costmap (robot) frame to global map frame
     * 
     * The costmap is in the robot's local frame (centered at robot).
     * We need to transform it to the global frame using the robot's position and orientation.
     * This performs both translation and rotation.
     * 
     * @param costmap_x X coordinate in costmap frame (relative to robot)
     * @param costmap_y Y coordinate in costmap frame (relative to robot)
     * @param robot_x Robot's x position in global frame
     * @param robot_y Robot's y position in global frame
     * @param robot_yaw Robot's yaw (orientation) in global frame (radians)
     * @param global_x Output: X coordinate in global frame
     * @param global_y Output: Y coordinate in global frame
     */
    void transformToGlobalFrame(double costmap_x, double costmap_y,
                                 double robot_x, double robot_y, double robot_yaw,
                                 double& global_x, double& global_y) const;

    /**
     * Merge a costmap cell into the global map using linear fusion
     * 
     * Linear fusion rules:
     * - If new cell has a known value (0-100), overwrite global cell
     * - If new cell is unknown (-1), keep the global cell's current value
     * 
     * @param global_x_grid X index in global map
     * @param global_y_grid Y index in global map
     * @param costmap_value Value from costmap cell (-1, 0-100)
     */
    void mergeCell(int global_x_grid, int global_y_grid, int8_t costmap_value);
};

}  

#endif
