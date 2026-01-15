#ifndef CONTROL_CORE_HPP_
#define CONTROL_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <optional>
#include <cmath>

namespace robot
{

/**
 * ControlCore: Core logic for Pure Pursuit Control
 * 
 * This class implements the Pure Pursuit Control algorithm to guide a robot
 * along a predefined path. The algorithm selects a "lookahead point" on the
 * path at a fixed distance ahead and calculates the steering angle needed to
 * reach this point based on the curvature of a circular arc.
 */
class ControlCore {
  public:
    /**
     * Constructor: Initializes the Pure Pursuit Controller
     * 
     * @param logger ROS2 logger for debug/info messages
     * @param lookahead_distance Distance ahead on path to select target point (meters)
     * @param goal_tolerance Distance to consider the goal reached (meters)
     * @param linear_speed Constant forward speed (m/s)
     */
    ControlCore(const rclcpp::Logger& logger, 
                double lookahead_distance = 1.0,
                double goal_tolerance = 0.1,
                double linear_speed = 0.5);
  
    /**
     * Find the lookahead point on the path
     * 
     * Searches for a point on the path that is approximately at the lookahead
     * distance from the robot's current position. If no such point exists
     * (e.g., path is too short), returns the final goal point.
     * 
     * @param path The path to follow
     * @param robot_x Current robot x position in global frame
     * @param robot_y Current robot y position in global frame
     * @param robot_yaw Current robot orientation (yaw angle in radians)
     * @return Optional PoseStamped of the lookahead point, or nullopt if path is empty
     */
    std::optional<geometry_msgs::msg::PoseStamped> findLookaheadPoint(
        const nav_msgs::msg::Path& path,
        double robot_x,
        double robot_y,
        double robot_yaw) const;

    /**
     * Compute velocity commands to reach the target point
     * 
     * Uses Pure Pursuit Control to calculate linear and angular velocities
     * based on the curvature of the path to the lookahead point.
     * 
     * @param target Lookahead point to reach
     * @param robot_x Current robot x position
     * @param robot_y Current robot y position
     * @param robot_yaw Current robot orientation (yaw angle in radians)
     * @return Twist message with linear and angular velocity commands
     */
    geometry_msgs::msg::Twist computeVelocity(
        const geometry_msgs::msg::PoseStamped& target,
        double robot_x,
        double robot_y,
        double robot_yaw) const;

    /**
     * Check if the robot has reached the goal
     * 
     * @param path The path being followed
     * @param robot_x Current robot x position
     * @param robot_y Current robot y position
     * @return true if robot is within goal_tolerance of the final goal
     */
    bool goalReached(const nav_msgs::msg::Path& path,
                     double robot_x,
                     double robot_y) const;

    /**
     * Compute Euclidean distance between two points
     * 
     * @param x1 First point x coordinate
     * @param y1 First point y coordinate
     * @param x2 Second point x coordinate
     * @param y2 Second point y coordinate
     * @return Distance in meters
     */
    static double computeDistance(double x1, double y1, double x2, double y2);

    /**
     * Extract yaw angle from quaternion
     * 
     * Converts a quaternion orientation to a yaw angle (rotation around z-axis).
     * 
     * @param quat Quaternion message
     * @return Yaw angle in radians, range [-pi, pi]
     */
    static double extractYaw(const geometry_msgs::msg::Quaternion& quat);

  private:
    rclcpp::Logger logger_;
    double lookahead_distance_;
    double goal_tolerance_;
    double linear_speed_;
};

} 

#endif 
