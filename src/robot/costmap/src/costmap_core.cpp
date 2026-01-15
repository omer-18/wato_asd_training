#include "costmap_core.hpp"
#include <algorithm>

namespace robot
{

/**
 * Constructor: Sets up default costmap parameters
 * 
 * Default values:
 * - Resolution: 0.1 m/cell (10 cm per cell)
 * - Size: 200x200 cells (20m x 20m at 0.1m resolution)
 * - Inflation radius: 1.0 meter
 * - Max cost: 100 (fully occupied)
 */
CostmapCore::CostmapCore(const rclcpp::Logger& logger) 
    : logger_(logger),
      resolution_(0.1),        // 10 cm per cell
      width_(200),             // 200 cells = 20 meters
      height_(200),            // 200 cells = 20 meters
      inflation_radius_(2.0),  // Inflate obstacles by 2.0 meters (increased for better safety margin)
      max_cost_(100),          // Maximum cost value
      occupied_cost_(100)      // Cost for detected obstacles
{
    // Pre-allocate the grid to avoid reallocation during processing
    grid_.resize(width_ * height_, 0);
    
    RCLCPP_INFO(logger_, "CostmapCore initialized: %dx%d grid, %.2f m/cell resolution, %.2f m inflation", 
                 width_, height_, resolution_, inflation_radius_);
}

/**
 * Main processing function: Converts laser scan to occupancy grid
 */
nav_msgs::msg::OccupancyGrid CostmapCore::processLaserScan(
    const sensor_msgs::msg::LaserScan::SharedPtr scan) 
{
    // Step 1: Initialize/reset the costmap grid to free space (0)
    initializeCostmap();

    // Step 2: Convert each laser scan range to grid coordinates and mark obstacles
    for (size_t i = 0; i < scan->ranges.size(); ++i) {
        // Calculate the angle for this laser beam
        // angle = start_angle + (beam_index * angular_resolution)
        double angle = scan->angle_min + i * scan->angle_increment;
        double range = scan->ranges[i];

        // Only process valid range measurements
        // Invalid measurements are outside the sensor's min/max range
        if (range >= scan->range_min && range <= scan->range_max) {
            // Convert from polar coordinates (range, angle) to grid indices (x, y)
            int x_grid, y_grid;
            convertToGrid(range, angle, x_grid, y_grid);

            // Check if the grid indices are within bounds
            if (x_grid >= 0 && x_grid < width_ && y_grid >= 0 && y_grid < height_) {
                // Mark this cell as occupied (obstacle detected)
                markObstacle(x_grid, y_grid);
            }
        }
    }

    // Step 3: Inflate obstacles to create safety margins
    // This prevents the robot from getting too close to obstacles
    inflateObstacles();

    // Step 4: Convert the 2D grid into a ROS OccupancyGrid message
    return createOccupancyGrid(scan);
}

/**
 * Initialize or reset the costmap grid
 */
void CostmapCore::initializeCostmap() {
    // Reset all cells to 0 (free space)
    // Using std::fill is more efficient than iterating manually
    std::fill(grid_.begin(), grid_.end(), 0);
}

/**
 * Convert polar coordinates (range, angle) to grid cell indices
 * 
 * The costmap is centered at the robot, so:
 * - Robot is at grid center: (width/2, height/2)
 * - Positive x is forward, positive y is left (standard ROS convention)
 */
void CostmapCore::convertToGrid(double range, double angle, int& x_grid, int& y_grid) {
    // Convert from polar to Cartesian coordinates (robot frame)
    // x = range * cos(angle)  [forward/backward]
    // y = range * sin(angle)  [left/right]
    double x_world = range * std::cos(angle);
    double y_world = range * std::sin(angle);

    // Convert from world coordinates to grid indices
    // Add width/2 and height/2 to center the grid at the robot
    x_grid = static_cast<int>((x_world / resolution_) + (width_ / 2));
    y_grid = static_cast<int>((y_world / resolution_) + (height_ / 2));
}

/**
 * Mark a cell as occupied (obstacle detected)
 */
void CostmapCore::markObstacle(int x_grid, int y_grid) {
    // Calculate the 1D index from 2D coordinates
    int index = y_grid * width_ + x_grid;
    
    // Set the cell to occupied cost (100 = fully occupied)
    // Only set if current value is lower (don't overwrite higher costs from inflation)
    if (grid_[index] < occupied_cost_) {
        grid_[index] = occupied_cost_;
    }
}

/**
 * Inflate obstacles to create safety margins
 * 
 * For each obstacle cell, we assign decreasing costs to surrounding cells
 * based on their Euclidean distance. This creates a "gradient" of danger
 * around obstacles, making the robot prefer paths further from obstacles.
 */
void CostmapCore::inflateObstacles() {
    // Create a temporary copy of the grid to avoid modifying while iterating
    std::vector<int8_t> inflated_grid = grid_;

    // Calculate inflation radius in grid cells
    int inflation_cells = static_cast<int>(inflation_radius_ / resolution_);

    // Iterate through all cells in the grid
    for (int y = 0; y < height_; ++y) {
        for (int x = 0; x < width_; ++x) {
            int index = y * width_ + x;
            
            // Only inflate around occupied cells (obstacles)
            if (grid_[index] == occupied_cost_) {
                // Check all cells within the inflation radius
                for (int dy = -inflation_cells; dy <= inflation_cells; ++dy) {
                    for (int dx = -inflation_cells; dx <= inflation_cells; ++dx) {
                        int nx = x + dx;
                        int ny = y + dy;

                        // Check bounds
                        if (nx >= 0 && nx < width_ && ny >= 0 && ny < height_) {
                            // Calculate Euclidean distance from obstacle to this cell
                            double distance = std::sqrt(dx * dx + dy * dy) * resolution_;

                            // Only assign cost if within inflation radius
                            if (distance <= inflation_radius_ && distance > 0.0) {
                                // Calculate cost based on distance
                                // Closer to obstacle = higher cost
                                // Formula: cost = max_cost * (1 - distance / inflation_radius)
                                // Ensure we don't divide by zero and get smooth gradient
                                double cost_ratio = 1.0 - (distance / inflation_radius_);
                                int new_cost = static_cast<int>(
                                    max_cost_ * cost_ratio
                                );
                                
                                // Ensure cost is at least 1 if we're inflating (for visibility)
                                if (new_cost < 1 && distance < inflation_radius_) {
                                    new_cost = 1;
                                }

                                // Only update if new cost is higher than current
                                // This preserves obstacle cells and higher costs
                                int neighbor_index = ny * width_ + nx;
                                if (inflated_grid[neighbor_index] < new_cost) {
                                    inflated_grid[neighbor_index] = static_cast<int8_t>(new_cost);
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    // Replace the original grid with the inflated version
    grid_ = inflated_grid;
}

/**
 * Convert internal grid representation to ROS OccupancyGrid message
 */
nav_msgs::msg::OccupancyGrid CostmapCore::createOccupancyGrid(
    const sensor_msgs::msg::LaserScan::SharedPtr scan) 
{
    nav_msgs::msg::OccupancyGrid grid_msg;

    // Set header information (timestamp and frame reference)
    grid_msg.header.stamp = scan->header.stamp;
    grid_msg.header.frame_id = scan->header.frame_id;  // Usually "base_link" or "laser_frame"

    // Set grid metadata
    grid_msg.info.resolution = resolution_;              // Meters per cell
    grid_msg.info.width = width_;                       // Number of cells in x direction
    grid_msg.info.height = height_;                     // Number of cells in y direction

    // Set origin: The costmap is centered at the robot (0, 0)
    // So the origin is at (-width/2 * resolution, -height/2 * resolution)
    grid_msg.info.origin.position.x = -(width_ / 2.0) * resolution_;
    grid_msg.info.origin.position.y = -(height_ / 2.0) * resolution_;
    grid_msg.info.origin.position.z = 0.0;
    grid_msg.info.origin.orientation.w = 1.0;  // No rotation (identity quaternion)

    // Copy the grid data (already in row-major order as required by OccupancyGrid)
    // OccupancyGrid expects data in row-major order: [row0, row1, row2, ...]
    grid_msg.data = grid_;

    return grid_msg;
}

}