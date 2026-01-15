#ifndef COSTMAP_CORE_HPP_
#define COSTMAP_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include <vector>
#include <cmath>

namespace robot
{

/**
 * CostmapCore: Core logic for processing laser scans into occupancy grids
 * 
 * This class handles the conversion of raw laser scan data (polar coordinates)
 * into a discretized 2D grid (costmap) where each cell represents the probability
 * or cost of an obstacle existing at that location.
 */
class CostmapCore {
  public:
    /**
     * Constructor: Initializes the costmap core with default parameters
     * 
     * @param logger ROS2 logger for debug/info messages
     */
    explicit CostmapCore(const rclcpp::Logger& logger);

    /**
     * Main processing function: Converts a laser scan into an occupancy grid
     * 
     * This function performs the following steps:
     * 1. Initializes/resets the costmap grid
     * 2. Converts laser scan ranges to grid coordinates
     * 3. Marks obstacle cells
     * 4. Inflates obstacles to create safety margins
     * 5. Converts the 2D grid into an OccupancyGrid ROS message
     * 
     * @param scan Shared pointer to the laser scan message to process
     * @return OccupancyGrid message containing the costmap
     */
    nav_msgs::msg::OccupancyGrid processLaserScan(const sensor_msgs::msg::LaserScan::SharedPtr scan);

  private:
    rclcpp::Logger logger_;

    // Costmap parameters
    double resolution_;           // Grid resolution in meters per cell (e.g., 0.1 m/cell)
    int width_;                  // Width of the costmap in cells
    int height_;                 // Height of the costmap in cells
    double inflation_radius_;    // Radius for obstacle inflation in meters
    int max_cost_;               // Maximum cost value (100 = fully occupied)
    int occupied_cost_;         // Cost value for detected obstacles (100)
    
    // 2D grid representing the costmap (stored as 1D vector: grid_[y * width + x])
    // Values: -1 = unknown, 0 = free, 1-100 = cost (higher = more dangerous)
    std::vector<int8_t> grid_;

    /**
     * Initializes or resets the costmap grid to default values
     * Sets all cells to 0 (free space) initially
     */
    void initializeCostmap();

    /**
     * Converts a point from polar coordinates (range, angle) to grid indices
     * 
     * The costmap is centered at the robot (0,0), so we convert from
     * robot-relative coordinates to grid cell indices.
     * 
     * @param range Distance from robot to obstacle in meters
     * @param angle Angle of the laser beam in radians
     * @param x_grid Output parameter: x index in the grid
     * @param y_grid Output parameter: y index in the grid
     */
    void convertToGrid(double range, double angle, int& x_grid, int& y_grid);

    /**
     * Marks a cell as occupied (obstacle detected)
     * 
     * @param x_grid X index of the cell to mark
     * @param y_grid Y index of the cell to mark
     */
    void markObstacle(int x_grid, int y_grid);

    /**
     * Inflates obstacles to create safety margins around detected obstacles
     * 
     * For each obstacle cell, this function assigns decreasing costs to
     * surrounding cells based on their distance from the obstacle. This ensures
     * the robot avoids getting too close to obstacles, even if it doesn't
     * directly hit them.
     * 
     * The cost formula: cost = max_cost * (1 - distance / inflation_radius)
     */
    void inflateObstacles();

    /**
     * Converts the internal 2D grid representation into a ROS OccupancyGrid message
     * 
     * @param scan Original laser scan message (used for timestamp and frame_id)
     * @return OccupancyGrid message ready to be published
     */
    nav_msgs::msg::OccupancyGrid createOccupancyGrid(const sensor_msgs::msg::LaserScan::SharedPtr scan);
};

}  

#endif
