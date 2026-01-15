#ifndef PLANNER_CORE_HPP_
#define PLANNER_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include <unordered_map>
#include <queue>
#include <vector>
#include <cmath>

namespace robot
{

// ------------------- Supporting Structures for A* -------------------

/**
 * CellIndex: Represents a 2D grid cell index (x, y)
 * 
 * Used to identify cells in the occupancy grid for A* pathfinding.
 */
struct CellIndex
{
    int x;
    int y;

    CellIndex(int xx, int yy) : x(xx), y(yy) {}
    CellIndex() : x(0), y(0) {}

    bool operator==(const CellIndex &other) const
    {
        return (x == other.x && y == other.y);
    }

    bool operator!=(const CellIndex &other) const
    {
        return (x != other.x || y != other.y);
    }
};

/**
 * CellIndexHash: Hash function for CellIndex
 * 
 * Allows CellIndex to be used as a key in std::unordered_map
 * for efficient lookups during A* pathfinding.
 */
struct CellIndexHash
{
    std::size_t operator()(const CellIndex &idx) const
    {
        // A simple hash combining x and y
        return std::hash<int>()(idx.x) ^ (std::hash<int>()(idx.y) << 1);
    }
};

/**
 * AStarNode: Represents a node in the A* open set
 * 
 * Contains the cell index and f_score (f = g + h) for priority queue ordering.
 */
struct AStarNode
{
    CellIndex index;
    double f_score;  // f = g + h (total estimated cost)

    AStarNode(CellIndex idx, double f) : index(idx), f_score(f) {}
};

/**
 * CompareF: Comparator for the priority queue (min-heap by f_score)
 * 
 * This ensures the node with the smallest f_score is always at the top
 * of the priority queue, which is essential for A* optimality.
 */
struct CompareF
{
    bool operator()(const AStarNode &a, const AStarNode &b)
    {
        // We want the node with the smallest f_score on top
        // Note: priority_queue is a max-heap by default, so we reverse the comparison
        return a.f_score > b.f_score;
    }
};

/**
 * PlannerCore: Core logic for A* pathfinding on occupancy grids
 * 
 * This class implements the A* algorithm to find optimal paths from a start
 * position to a goal position on an occupancy grid, avoiding obstacles.
 */
class PlannerCore {
  public:
    /**
     * Constructor: Initializes the planner core
     * 
     * @param logger ROS2 logger for debug/info messages
     */
    explicit PlannerCore(const rclcpp::Logger& logger);

    /**
     * Plan a path from start to goal using A* algorithm
     * 
     * This function:
     * 1. Converts world coordinates to grid indices
     * 2. Runs A* algorithm to find optimal path
     * 3. Reconstructs the path from goal back to start
     * 4. Converts path back to world coordinates
     * 
     * @param map The occupancy grid to plan on
     * @param start_x Start x position in world coordinates (meters)
     * @param start_y Start y position in world coordinates (meters)
     * @param goal_x Goal x position in world coordinates (meters)
     * @param goal_y Goal y position in world coordinates (meters)
     * @return Path message containing waypoints from start to goal
     */
    nav_msgs::msg::Path planPath(
        const nav_msgs::msg::OccupancyGrid& map,
        double start_x, double start_y,
        double goal_x, double goal_y
    );

  private:
    rclcpp::Logger logger_;

    // Cost threshold for considering a cell as an obstacle
    // Cells with cost >= OBSTACLE_THRESHOLD are considered obstacles
    // Lower threshold = more conservative (avoids even moderately high-cost cells)
    // Higher threshold = more aggressive (allows paths through more inflated zones)
    // Set to 50 to better avoid obstacles and inflated zones for safer navigation
    static constexpr int OBSTACLE_THRESHOLD = 50;  // 50% or higher cost = obstacle (safer navigation)

    /**
     * Convert world coordinates to grid cell indices
     * 
     * @param x_world X coordinate in world frame (meters)
     * @param y_world Y coordinate in world frame (meters)
     * @param map The occupancy grid
     * @param x_grid Output: X index in grid
     * @param y_grid Output: Y index in grid
     * @return true if coordinates are within map bounds
     */
    bool worldToGrid(double x_world, double y_world,
                     const nav_msgs::msg::OccupancyGrid& map,
                     int& x_grid, int& y_grid) const;

    /**
     * Convert grid cell indices to world coordinates
     * 
     * @param x_grid X index in grid
     * @param y_grid Y index in grid
     * @param map The occupancy grid
     * @param x_world Output: X coordinate in world frame (meters)
     * @param y_world Output: Y coordinate in world frame (meters)
     */
    void gridToWorld(int x_grid, int y_grid,
                     const nav_msgs::msg::OccupancyGrid& map,
                     double& x_world, double& y_world) const;

    /**
     * Check if a cell is valid for pathfinding (not an obstacle and within bounds)
     * 
     * @param x_grid X index in grid
     * @param y_grid Y index in grid
     * @param map The occupancy grid
     * @return true if cell is valid (free or low cost)
     */
    bool isValidCell(int x_grid, int y_grid, const nav_msgs::msg::OccupancyGrid& map) const;

    /**
     * Get the cost value of a cell in the occupancy grid
     * 
     * @param x_grid X index in grid
     * @param y_grid Y index in grid
     * @param map The occupancy grid
     * @return Cell cost value (-1 = unknown, 0 = free, 1-100 = cost)
     */
    int8_t getCellCost(int x_grid, int y_grid, const nav_msgs::msg::OccupancyGrid& map) const;

    /**
     * Calculate heuristic distance from a cell to the goal (Euclidean distance)
     * 
     * This is the "h" in A*: h(n) = estimated cost from node n to goal
     * Using Euclidean distance ensures the heuristic is admissible (never overestimates).
     * 
     * @param x_grid Current cell x index
     * @param y_grid Current cell y index
     * @param goal_x_grid Goal cell x index
     * @param goal_y_grid Goal cell y index
     * @return Estimated distance to goal
     */
    double heuristic(int x_grid, int y_grid, int goal_x_grid, int goal_y_grid) const;

    /**
     * Get neighboring cells for A* expansion
     * 
     * Returns the 8-connected neighbors (including diagonals) of a cell.
     * This allows the robot to move in 8 directions (N, S, E, W, NE, NW, SE, SW).
     * 
     * @param cell The cell to get neighbors for
     * @return Vector of neighboring cell indices
     */
    std::vector<CellIndex> getNeighbors(const CellIndex& cell) const;

    /**
     * Calculate movement cost between two adjacent cells
     * 
     * Diagonal moves cost more (√2) than cardinal moves (1) to account for
     * the longer distance traveled.
     * 
     * @param from Source cell
     * @param to Destination cell
     * @return Movement cost (1.0 for cardinal, ~1.414 for diagonal)
     */
    double movementCost(const CellIndex& from, const CellIndex& to) const;
};

}  

#endif
