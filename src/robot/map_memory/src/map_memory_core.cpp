#include "map_memory_core.hpp"
#include <algorithm>

namespace robot
{

/**
 * Constructor: Initializes global map and tracking variables
 * 
 * Sets up a large global map (100m x 100m) to accumulate knowledge
 * from local costmaps as the robot explores.
 */
MapMemoryCore::MapMemoryCore(const rclcpp::Logger& logger)
    : logger_(logger),
      map_resolution_(0.1),        // 0.1 m/cell (matches costmap resolution)
      map_width_(1000),            // 1000 cells = 100 meters
      map_height_(1000),           // 1000 cells = 100 meters
      map_origin_x_(-50.0),        // Origin at -50m (centers map at 0,0)
      map_origin_y_(-50.0),        // Origin at -50m (centers map at 0,0)
      last_update_x_(0.0),         // Initialize to origin
      last_update_y_(0.0),
      current_x_(0.0),
      current_y_(0.0),
      current_yaw_(0.0),           // Initialize orientation to 0
      distance_threshold_(1.5),     // Update map when robot moves 1.5 meters
      costmap_received_(false),
      robot_moved_enough_(false)
{
    // Initialize the global map
    initializeGlobalMap();
    
    RCLCPP_DEBUG(logger_, "MapMemoryCore initialized: %dx%d global map, %.2f m/cell resolution", 
                 map_width_, map_height_, map_resolution_);
}

/**
 * Initialize the global map with unknown values
 */
void MapMemoryCore::initializeGlobalMap() {
    // Pre-allocate the global grid
    // -1 = unknown, 0 = free, 1-100 = cost (higher = more dangerous)
    global_grid_.resize(map_width_ * map_height_, -1);
    
    RCLCPP_DEBUG(logger_, "Global map initialized: %zu cells", global_grid_.size());
}

/**
 * Store the latest costmap for integration
 */
void MapMemoryCore::storeLatestCostmap(const nav_msgs::msg::OccupancyGrid::SharedPtr costmap) {
    latest_costmap_ = costmap;
    costmap_received_ = true;
}

/**
 * Check if robot has moved far enough to trigger map update
 */
bool MapMemoryCore::checkRobotMovement(double x, double y, double yaw) {
    // Update current position and orientation
    current_x_ = x;
    current_y_ = y;
    current_yaw_ = yaw;

    // Calculate Euclidean distance from last update position
    double dx = x - last_update_x_;
    double dy = y - last_update_y_;
    double distance = std::sqrt(dx * dx + dy * dy);

    // Check if robot has moved at least the threshold distance
    if (distance >= distance_threshold_) {
        robot_moved_enough_ = true;
        return true;
    }

    return false;
}

/**
 * Check if conditions are met to update the global map
 */
bool MapMemoryCore::shouldUpdateMap() {
    // Update map only if:
    // 1. Robot has moved at least 1.5 meters since last update
    // 2. A new costmap has been received
    return robot_moved_enough_ && costmap_received_;
}

/**
 * Integrate the latest costmap into the global map
 * 
 * This is the core function that:
 * 1. Transforms each costmap cell from robot frame to global frame
 * 2. Merges the costmap into the global map using linear fusion
 */
void MapMemoryCore::integrateCostmap() {
    if (!latest_costmap_) {
        RCLCPP_WARN(logger_, "Cannot integrate costmap: no costmap stored");
        return;
    }

    // Get costmap parameters
    const auto& costmap = latest_costmap_;
    double costmap_resolution = costmap->info.resolution;
    int costmap_width = costmap->info.width;
    int costmap_height = costmap->info.height;
    
    // Get costmap origin (relative to robot)
    // Costmap is centered at robot, so origin is typically (-width/2 * resolution, -height/2 * resolution)
    double costmap_origin_x = costmap->info.origin.position.x;
    double costmap_origin_y = costmap->info.origin.position.y;

    RCLCPP_DEBUG(logger_, "Integrating costmap %dx%d into global map at robot position (%.2f, %.2f)",
                 costmap_width, costmap_height, current_x_, current_y_);

    // Iterate through each cell in the costmap
    // Use center of cell for more accurate transformation
    for (int cy = 0; cy < costmap_height; ++cy) {
        for (int cx = 0; cx < costmap_width; ++cx) {
            // Get costmap cell value
            int costmap_index = cy * costmap_width + cx;
            int8_t costmap_value = costmap->data[costmap_index];

            // Skip unknown cells (-1) - we only integrate known information
            // This implements the linear fusion rule: unknown values don't overwrite
            if (costmap_value == -1) {
                continue;
            }

            // Calculate world coordinates of this costmap cell CENTER (in robot frame)
            // Costmap cell (cx, cy) -> world coordinates relative to robot
            // Add 0.5 to get center of cell for more accurate transformation
            double costmap_x_world = costmap_origin_x + ((cx + 0.5) * costmap_resolution);
            double costmap_y_world = costmap_origin_y + ((cy + 0.5) * costmap_resolution);

            // Transform from robot frame to global frame
            // Rotate and translate using robot's position and orientation
            double global_x, global_y;
            transformToGlobalFrame(costmap_x_world, costmap_y_world,
                                   current_x_, current_y_, current_yaw_,
                                   global_x, global_y);

            // Convert global world coordinates to global map grid indices
            int global_x_grid, global_y_grid;
            if (worldToGrid(global_x, global_y, global_x_grid, global_y_grid)) {
                // Merge this cell into the global map using linear fusion
                // Use maximum cost to preserve obstacles and high-cost areas
                mergeCell(global_x_grid, global_y_grid, costmap_value);
            }
            // If coordinates are out of bounds, skip this cell
        }
    }

    // Update last update position to current position
    last_update_x_ = current_x_;
    last_update_y_ = current_y_;
    
    // Reset flags for next update cycle
    robot_moved_enough_ = false;
    costmap_received_ = false;

    RCLCPP_DEBUG(logger_, "Costmap integrated. Global map updated at position (%.2f, %.2f)",
                 current_x_, current_y_);
}

/**
 * Transform point from costmap (robot) frame to global frame
 * 
 * Performs rotation and translation to transform from robot's local frame
 * to the global map frame.
 */
void MapMemoryCore::transformToGlobalFrame(double costmap_x, double costmap_y,
                                           double robot_x, double robot_y, double robot_yaw,
                                           double& global_x, double& global_y) const {
    // Rotate the costmap point from robot frame to global frame
    // Rotation matrix: [cos(θ) -sin(θ)] [x]
    //                  [sin(θ)  cos(θ)] [y]
    double cos_yaw = std::cos(robot_yaw);
    double sin_yaw = std::sin(robot_yaw);
    
    // Apply rotation
    double rotated_x = costmap_x * cos_yaw - costmap_y * sin_yaw;
    double rotated_y = costmap_x * sin_yaw + costmap_y * cos_yaw;
    
    // Translate by robot position
    global_x = robot_x + rotated_x;
    global_y = robot_y + rotated_y;
}

/**
 * Convert world coordinates to grid cell indices
 */
bool MapMemoryCore::worldToGrid(double x_world, double y_world, int& x_grid, int& y_grid) const {
    // Convert from world coordinates to grid indices
    // grid_index = (world_coord - origin) / resolution
    x_grid = static_cast<int>((x_world - map_origin_x_) / map_resolution_);
    y_grid = static_cast<int>((y_world - map_origin_y_) / map_resolution_);

    // Check if coordinates are within map bounds
    return (x_grid >= 0 && x_grid < map_width_ && y_grid >= 0 && y_grid < map_height_);
}

/**
 * Convert grid cell indices to world coordinates
 */
void MapMemoryCore::gridToWorld(int x_grid, int y_grid, double& x_world, double& y_world) const {
    // Convert from grid indices to world coordinates
    // world_coord = origin + (grid_index * resolution)
    x_world = map_origin_x_ + (x_grid * map_resolution_);
    y_world = map_origin_y_ + (y_grid * map_resolution_);
}

/**
 * Merge a costmap cell into the global map using linear fusion
 * 
 * Linear fusion rules:
 * - If new cell has a known value (0-100), use maximum of old and new (preserve obstacles)
 * - If new cell is unknown (-1), keep the global cell's current value
 * 
 * Note: We already skip unknown cells in integrateCostmap(), so this
 * function always receives known values. We use maximum to preserve
 * obstacles and high-cost areas from previous observations.
 */
void MapMemoryCore::mergeCell(int global_x_grid, int global_y_grid, int8_t costmap_value) {
    // Calculate 1D index from 2D grid coordinates
    int global_index = global_y_grid * map_width_ + global_x_grid;

    // Linear fusion with maximum: preserve obstacles and high-cost areas
    // If global cell is unknown (-1), always use new value
    // Otherwise, use maximum to preserve obstacles and high-cost zones
    if (global_grid_[global_index] == -1) {
        // Unknown cell - use new value
        global_grid_[global_index] = costmap_value;
    } else {
        // Known cell - use maximum to preserve obstacles and high-cost areas
        // This ensures obstacles don't get erased by free space observations
        global_grid_[global_index] = std::max(global_grid_[global_index], costmap_value);
    }
}

/**
 * Get the current global map as an OccupancyGrid message
 */
nav_msgs::msg::OccupancyGrid MapMemoryCore::getGlobalMap() {
    nav_msgs::msg::OccupancyGrid global_map;

    // Set header
    // Use timestamp from latest costmap if available, otherwise use zero
    if (latest_costmap_) {
        global_map.header.stamp = latest_costmap_->header.stamp;
    } else {
        // For initial map, use zero timestamp
        global_map.header.stamp.sec = 0;
        global_map.header.stamp.nanosec = 0;
    }
    global_map.header.frame_id = "sim_world";  // Use sim_world to match odometry frame

    // Set grid metadata
    global_map.info.resolution = map_resolution_;
    global_map.info.width = map_width_;
    global_map.info.height = map_height_;

    // Set origin
    global_map.info.origin.position.x = map_origin_x_;
    global_map.info.origin.position.y = map_origin_y_;
    global_map.info.origin.position.z = 0.0;
    global_map.info.origin.orientation.w = 1.0;  // No rotation

    // Copy the global grid data (already in row-major order)
    global_map.data = global_grid_;

    return global_map;
}

}
