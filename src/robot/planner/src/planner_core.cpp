#include "planner_core.hpp"
#include <algorithm>
#include <limits>
#include <unordered_set>

namespace robot
{

/**
 * Constructor: Initializes the planner core
 */
PlannerCore::PlannerCore(const rclcpp::Logger& logger) 
    : logger_(logger)
{
    RCLCPP_DEBUG(logger_, "PlannerCore initialized with A* pathfinding");
}

/**
 * Plan a path from start to goal using A* algorithm
 */
nav_msgs::msg::Path PlannerCore::planPath(
    const nav_msgs::msg::OccupancyGrid& map,
    double start_x, double start_y,
    double goal_x, double goal_y)
{
    nav_msgs::msg::Path path;
    path.header.frame_id = "sim_world";  // Use sim_world to match odometry frame

    // Step 1: Convert world coordinates to grid indices
    int start_x_grid, start_y_grid;
    int goal_x_grid, goal_y_grid;

    // Debug: Log map parameters (only when planning starts)
    RCLCPP_DEBUG(logger_, "Map: resolution=%.2f, origin=(%.2f, %.2f), size=(%ux%u)",
                map.info.resolution, map.info.origin.position.x, map.info.origin.position.y,
                map.info.width, map.info.height);
    RCLCPP_DEBUG(logger_, "Planning from world (%.2f, %.2f) to (%.2f, %.2f)",
                start_x, start_y, goal_x, goal_y);

    if (!worldToGrid(start_x, start_y, map, start_x_grid, start_y_grid)) {
        RCLCPP_WARN(logger_, "Start position (%.2f, %.2f) is outside map bounds!", start_x, start_y);
        return path;  // Return empty path
    }

    if (!worldToGrid(goal_x, goal_y, map, goal_x_grid, goal_y_grid)) {
        RCLCPP_WARN(logger_, "Goal position (%.2f, %.2f) is outside map bounds!", goal_x, goal_y);
        RCLCPP_WARN(logger_, "Map bounds: x=[%.2f, %.2f], y=[%.2f, %.2f]",
                    map.info.origin.position.x,
                    map.info.origin.position.x + map.info.width * map.info.resolution,
                    map.info.origin.position.y,
                    map.info.origin.position.y + map.info.height * map.info.resolution);
        return path;  // Return empty path
    }

    RCLCPP_DEBUG(logger_, "Converted to grid: start=(%d, %d), goal=(%d, %d)",
                start_x_grid, start_y_grid, goal_x_grid, goal_y_grid);

    // Check if start and goal are valid (not obstacles)
    // For start position, be more lenient - allow planning even if in slightly inflated area
    // This handles cases where the robot starts near an obstacle
    int8_t start_cost = getCellCost(start_x_grid, start_y_grid, map);
    if (start_cost >= OBSTACLE_THRESHOLD && start_cost < 80) {
        // Start is in a moderately high-cost area (inflated zone), but not a hard obstacle
        // With threshold of 30, this means cost is 30-79
        // Try to find a nearby better start position to avoid getting stuck
        RCLCPP_INFO(logger_, "Start position is in inflated zone (cost: %d). Searching for better start position...", start_cost);
        
        // Search for a lower-cost start cell nearby, but be less aggressive
        // Only search if we're in a high-cost zone (cost > 50) to avoid unnecessary adjustments
        if (start_cost > 50) {
            int search_radius = 10;  // Search up to 10 cells (1 meter) away - smaller radius
            int best_cost = start_cost;
            int best_x = start_x_grid;
            int best_y = start_y_grid;
            bool found_better = false;
            
            for (int radius = 1; radius <= search_radius; ++radius) {
                for (int dx = -radius; dx <= radius; ++dx) {
                    for (int dy = -radius; dy <= radius; ++dy) {
                        // Only check cells on the edge of the current radius
                        if (std::abs(dx) == radius || std::abs(dy) == radius) {
                            int test_x = start_x_grid + dx;
                            int test_y = start_y_grid + dy;
                            
                            if (isValidCell(test_x, test_y, map)) {
                                int8_t test_cost = getCellCost(test_x, test_y, map);
                                // Prefer cells with significantly lower cost (at least 20 points better)
                                if (test_cost < best_cost - 20) {
                                    best_cost = test_cost;
                                    best_x = test_x;
                                    best_y = test_y;
                                    found_better = true;
                                }
                            }
                        }
                    }
                }
                // If we found a good cell (cost < 30), use it
                if (found_better && best_cost < 30) {
                    break;
                }
            }
            
            if (found_better && best_cost < start_cost - 20) {
                start_x_grid = best_x;
                start_y_grid = best_y;
                double adjusted_x, adjusted_y;
                gridToWorld(start_x_grid, start_y_grid, map, adjusted_x, adjusted_y);
                RCLCPP_INFO(logger_, "Found better start position at (%.2f, %.2f) with cost %d (was %d)", 
                           adjusted_x, adjusted_y, best_cost, start_cost);
            } else {
                RCLCPP_DEBUG(logger_, "Allowing planning to proceed from inflated zone - path will move away from obstacle");
            }
        } else {
            // Cost is 30-50, which is acceptable - just proceed
            RCLCPP_DEBUG(logger_, "Start position in moderate inflation zone (cost: %d), proceeding with planning", start_cost);
        }
    } else if (start_cost >= 80) {
        // Start is in a hard obstacle - try to find nearest valid cell
        RCLCPP_WARN(logger_, "Start position is in a hard obstacle (cost: %d)! Searching for nearest valid cell...", start_cost);
        
        // Search for nearest valid cell within a reasonable radius
        int search_radius = 10;  // Search up to 10 cells (1 meter at 0.1m resolution) away
        bool found_valid = false;
        
        for (int radius = 1; radius <= search_radius && !found_valid; ++radius) {
            for (int dx = -radius; dx <= radius && !found_valid; ++dx) {
                for (int dy = -radius; dy <= radius && !found_valid; ++dy) {
                    // Only check cells on the edge of the current radius
                    if (std::abs(dx) == radius || std::abs(dy) == radius) {
                        int test_x = start_x_grid + dx;
                        int test_y = start_y_grid + dy;
                        
                        if (isValidCell(test_x, test_y, map)) {
                            start_x_grid = test_x;
                            start_y_grid = test_y;
                            found_valid = true;
                            
                            // Convert back to world coordinates for logging
                            double adjusted_x, adjusted_y;
                            gridToWorld(start_x_grid, start_y_grid, map, adjusted_x, adjusted_y);
                            RCLCPP_INFO(logger_, "Found valid start cell at (%.2f, %.2f) (grid: %d, %d)", 
                                       adjusted_x, adjusted_y, start_x_grid, start_y_grid);
                        }
                    }
                }
            }
        }
        
        if (!found_valid) {
            RCLCPP_WARN(logger_, "Could not find valid start cell within search radius!");
            return path;
        }
    } else if (!isValidCell(start_x_grid, start_y_grid, map)) {
        // Unknown or other invalid state
        RCLCPP_WARN(logger_, "Start position is invalid!");
        return path;
    }

    if (!isValidCell(goal_x_grid, goal_y_grid, map)) {
        RCLCPP_WARN(logger_, "Goal position is in an obstacle!");
        return path;
    }

    CellIndex start(start_x_grid, start_y_grid);
    CellIndex goal(goal_x_grid, goal_y_grid);

    // If start and goal are the same, return path with just the goal
    if (start == goal) {
        geometry_msgs::msg::PoseStamped pose;
        pose.pose.position.x = goal_x;
        pose.pose.position.y = goal_y;
        pose.pose.position.z = 0.0;
        pose.pose.orientation.w = 1.0;
        path.poses.push_back(pose);
        return path;
    }

    RCLCPP_DEBUG(logger_, "Planning from grid (%d, %d) to (%d, %d)", 
                 start_x_grid, start_y_grid, goal_x_grid, goal_y_grid);

    // Step 2: Initialize A* data structures
    // Open set: priority queue of nodes to be evaluated (min-heap by f_score)
    std::priority_queue<AStarNode, std::vector<AStarNode>, CompareF> open_set;
    
    // Closed set: set of nodes already evaluated (to avoid revisiting)
    std::unordered_set<CellIndex, CellIndexHash> closed_set;
    
    // g_score: cost from start to each node (actual cost)
    std::unordered_map<CellIndex, double, CellIndexHash> g_score;
    
    // f_score: estimated total cost (g + h) for each node
    std::unordered_map<CellIndex, double, CellIndexHash> f_score;
    
    // came_from: tracks the path by storing which node each node came from
    std::unordered_map<CellIndex, CellIndex, CellIndexHash> came_from;

    // Initialize start node
    double start_g = 0.0;
    double start_h = heuristic(start_x_grid, start_y_grid, goal_x_grid, goal_y_grid);
    double start_f = start_g + start_h;

    g_score[start] = start_g;
    f_score[start] = start_f;
    open_set.push(AStarNode(start, start_f));

    // Step 3: A* main loop
    bool path_found = false;
    int iterations = 0;
    const int MAX_ITERATIONS = map.info.width * map.info.height;  // Safety limit

    while (!open_set.empty() && iterations < MAX_ITERATIONS) {
        iterations++;

        // Get node with lowest f_score from open set
        AStarNode current = open_set.top();
        open_set.pop();
        CellIndex current_cell = current.index;

        // Check if we've reached the goal
        if (current_cell == goal) {
            path_found = true;
            RCLCPP_DEBUG(logger_, "Path found in %d iterations", iterations);
            break;
        }

        // Add current node to closed set (we've evaluated it)
        closed_set.insert(current_cell);

        // Explore neighbors
        std::vector<CellIndex> neighbors = getNeighbors(current_cell);
        for (const CellIndex& neighbor : neighbors) {
            // Skip if neighbor is in closed set (already evaluated)
            if (closed_set.find(neighbor) != closed_set.end()) {
                continue;
            }

            // Skip if neighbor is invalid (obstacle or out of bounds)
            if (!isValidCell(neighbor.x, neighbor.y, map)) {
                continue;
            }

            // Calculate tentative g_score (cost from start to neighbor)
            // g_score[neighbor] = g_score[current] + movement_cost(current, neighbor)
            // Add a penalty based on cell cost to strongly prefer paths through lower-cost areas
            // This encourages paths to stay in free space (cost 0) rather than inflated zones
            double base_cost = movementCost(current_cell, neighbor);
            int8_t cell_cost = getCellCost(neighbor.x, neighbor.y, map);
            // Add penalty: cells with cost > 0 get penalty, encouraging paths through free space
            // Increased penalty (0.01 per cost unit) to more strongly prefer free space
            double cost_penalty = 0.0;
            if (cell_cost > 0 && cell_cost < OBSTACLE_THRESHOLD) {
                // Penalty increases with cell cost to prefer free space
                // Higher penalty to avoid inflated zones more aggressively
                cost_penalty = static_cast<double>(cell_cost) * 0.01;  // Penalty (0.01 per cost unit)
            }
            double tentative_g = g_score[current_cell] + base_cost + cost_penalty;

            // Check if we've found a better path to this neighbor
            // If neighbor not in g_score, it's infinity (first time seeing it)
            auto g_it = g_score.find(neighbor);
            if (g_it == g_score.end() || tentative_g < g_it->second) {
                // This path to neighbor is better, record it
                came_from[neighbor] = current_cell;
                g_score[neighbor] = tentative_g;
                
                // Calculate f_score = g_score + heuristic
                double h = heuristic(neighbor.x, neighbor.y, goal_x_grid, goal_y_grid);
                double f = tentative_g + h;
                f_score[neighbor] = f;

                // Add neighbor to open set for future evaluation
                open_set.push(AStarNode(neighbor, f));
            }
        }
    }

    if (!path_found) {
        // If no path found and robot is in an inflated zone, try to find a nearby valid start cell
        int8_t start_cost = getCellCost(start_x_grid, start_y_grid, map);
        if (start_cost >= OBSTACLE_THRESHOLD && start_cost < 100) {
            RCLCPP_INFO(logger_, "No path found from inflated zone (cost: %d). Searching for nearby valid start cell...", start_cost);
            
            // Search for a valid cell nearby to start planning from
            int search_radius = 15;  // Search up to 15 cells (1.5 meters) away
            bool found_valid = false;
            
            for (int radius = 1; radius <= search_radius && !found_valid; ++radius) {
                for (int dx = -radius; dx <= radius && !found_valid; ++dx) {
                    for (int dy = -radius; dy <= radius && !found_valid; ++dy) {
                        // Only check cells on the edge of the current radius
                        if (std::abs(dx) == radius || std::abs(dy) == radius) {
                            int test_x = start_x_grid + dx;
                            int test_y = start_y_grid + dy;
                            
                            if (isValidCell(test_x, test_y, map)) {
                                // Found a valid cell, try planning from there
                                CellIndex new_start(test_x, test_y);
                                
                                // Quick check: can we reach goal from this new start?
                                if (isValidCell(goal_x_grid, goal_y_grid, map)) {
                                    // Try a simple path: if new start and goal are both valid, create a direct path
                                    // This is a fallback - in practice, you'd want to call planPath recursively
                                    // but for now, we'll return empty and let the timer trigger a replan
                                    RCLCPP_INFO(logger_, "Found valid start cell at grid (%d, %d), will replan on next cycle", test_x, test_y);
                                    found_valid = true;
                                }
                            }
                        }
                    }
                }
            }
        }
        
        RCLCPP_WARN(logger_, "No path found from (%.2f, %.2f) to (%.2f, %.2f) after %d iterations",
                    start_x, start_y, goal_x, goal_y, iterations);
        return path;  // Return empty path - control node will handle stopping
    }

    // Step 4: Reconstruct path from goal back to start
    std::vector<CellIndex> path_cells;
    CellIndex current = goal;

    // Follow came_from links from goal back to start
    while (current != start) {
        path_cells.push_back(current);
        auto it = came_from.find(current);
        if (it == came_from.end()) {
            RCLCPP_ERROR(logger_, "Path reconstruction failed!");
            return path;  // Return empty path
        }
        current = it->second;
    }
    path_cells.push_back(start);  // Add start node

    // Reverse to get path from start to goal
    std::reverse(path_cells.begin(), path_cells.end());

    // Step 5: Convert grid path to world coordinates
    for (const CellIndex& cell : path_cells) {
        double x_world, y_world;
        gridToWorld(cell.x, cell.y, map, x_world, y_world);

        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = "sim_world";  // Use sim_world to match odometry frame
        pose.header.stamp.sec = 0;
        pose.header.stamp.nanosec = 0;
        pose.pose.position.x = x_world;
        pose.pose.position.y = y_world;
        pose.pose.position.z = 0.0;
        pose.pose.orientation.w = 1.0;  // Default orientation (no rotation)
        path.poses.push_back(pose);
    }

    RCLCPP_DEBUG(logger_, "Path reconstructed with %zu waypoints. First waypoint: (%.2f, %.2f), Last waypoint: (%.2f, %.2f)",
                path.poses.size(),
                path.poses.front().pose.position.x, path.poses.front().pose.position.y,
                path.poses.back().pose.position.x, path.poses.back().pose.position.y);
    return path;
}

/**
 * Convert world coordinates to grid cell indices
 */
bool PlannerCore::worldToGrid(double x_world, double y_world,
                              const nav_msgs::msg::OccupancyGrid& map,
                              int& x_grid, int& y_grid) const {
    // Convert from world coordinates to grid indices
    // grid_index = (world_coord - origin) / resolution
    x_grid = static_cast<int>((x_world - map.info.origin.position.x) / map.info.resolution);
    y_grid = static_cast<int>((y_world - map.info.origin.position.y) / map.info.resolution);

    // Check if coordinates are within map bounds
    return (x_grid >= 0 && x_grid < static_cast<int>(map.info.width) &&
            y_grid >= 0 && y_grid < static_cast<int>(map.info.height));
}

/**
 * Convert grid cell indices to world coordinates
 */
void PlannerCore::gridToWorld(int x_grid, int y_grid,
                               const nav_msgs::msg::OccupancyGrid& map,
                               double& x_world, double& y_world) const {
    // Convert from grid indices to world coordinates
    // world_coord = origin + (grid_index * resolution)
    x_world = map.info.origin.position.x + (x_grid * map.info.resolution);
    y_world = map.info.origin.position.y + (y_grid * map.info.resolution);
}

/**
 * Check if a cell is valid for pathfinding
 */
bool PlannerCore::isValidCell(int x_grid, int y_grid, const nav_msgs::msg::OccupancyGrid& map) const {
    // Check bounds
    if (x_grid < 0 || x_grid >= static_cast<int>(map.info.width) ||
        y_grid < 0 || y_grid >= static_cast<int>(map.info.height)) {
        return false;
    }

    // Check if cell is an obstacle
    int8_t cost = getCellCost(x_grid, y_grid, map);
    
    // Unknown cells (-1) are considered valid (we can explore them)
    // Free cells (0) are valid
    // Low cost cells (< OBSTACLE_THRESHOLD) are valid
    // High cost cells (>= OBSTACLE_THRESHOLD) are obstacles
    return (cost == -1 || cost < OBSTACLE_THRESHOLD);
}

/**
 * Get the cost value of a cell
 */
int8_t PlannerCore::getCellCost(int x_grid, int y_grid, const nav_msgs::msg::OccupancyGrid& map) const {
    // Calculate 1D index from 2D coordinates
    // OccupancyGrid data is stored in row-major order: data[y * width + x]
    int index = y_grid * map.info.width + x_grid;
    
    if (index < 0 || index >= static_cast<int>(map.data.size())) {
        return 100;  // Out of bounds = obstacle
    }
    
    return map.data[index];
}

/**
 * Calculate heuristic distance (Euclidean distance)
 */
double PlannerCore::heuristic(int x_grid, int y_grid, int goal_x_grid, int goal_y_grid) const {
    // Euclidean distance: sqrt((dx)^2 + (dy)^2)
    double dx = goal_x_grid - x_grid;
    double dy = goal_y_grid - y_grid;
    return std::sqrt(dx * dx + dy * dy);
}

/**
 * Get neighboring cells (8-connected)
 */
std::vector<CellIndex> PlannerCore::getNeighbors(const CellIndex& cell) const {
    std::vector<CellIndex> neighbors;
    
    // 8-connected neighbors: N, S, E, W, NE, NW, SE, SW
    neighbors.push_back(CellIndex(cell.x + 1, cell.y));      // East
    neighbors.push_back(CellIndex(cell.x - 1, cell.y));      // West
    neighbors.push_back(CellIndex(cell.x, cell.y + 1));      // North
    neighbors.push_back(CellIndex(cell.x, cell.y - 1));      // South
    neighbors.push_back(CellIndex(cell.x + 1, cell.y + 1));  // Northeast
    neighbors.push_back(CellIndex(cell.x - 1, cell.y + 1));  // Northwest
    neighbors.push_back(CellIndex(cell.x + 1, cell.y - 1));  // Southeast
    neighbors.push_back(CellIndex(cell.x - 1, cell.y - 1));  // Southwest
    
    return neighbors;
}

/**
 * Calculate movement cost between two adjacent cells
 */
double PlannerCore::movementCost(const CellIndex& from, const CellIndex& to) const {
    int dx = to.x - from.x;
    int dy = to.y - from.y;
    
    // Cardinal moves (N, S, E, W): cost = 1.0
    // Diagonal moves (NE, NW, SE, SW): cost = sqrt(2) ≈ 1.414
    if (dx == 0 || dy == 0) {
        return 1.0;  // Cardinal move
    } else {
        return std::sqrt(2.0);  // Diagonal move
    }
}

}
