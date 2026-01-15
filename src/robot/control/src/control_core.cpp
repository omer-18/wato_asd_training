#include "control_core.hpp"
#include <algorithm>
#include <limits>

namespace robot
{

ControlCore::ControlCore(const rclcpp::Logger& logger,
                         double lookahead_distance,
                         double goal_tolerance,
                         double linear_speed)
  : logger_(logger),
    lookahead_distance_(lookahead_distance),
    goal_tolerance_(goal_tolerance),
    linear_speed_(linear_speed)
{
  RCLCPP_INFO(logger_, 
              "ControlCore initialized: lookahead=%.2fm, goal_tolerance=%.2fm, linear_speed=%.2fm/s",
              lookahead_distance_, goal_tolerance_, linear_speed_);
}

std::optional<geometry_msgs::msg::PoseStamped> ControlCore::findLookaheadPoint(
    const nav_msgs::msg::Path& path,
    double robot_x,
    double robot_y,
    double robot_yaw) const
{
  // Return nullopt if path is empty
  if (path.poses.empty()) {
    return std::nullopt;
  }

  // If path has only one point, return it
  if (path.poses.size() == 1) {
    return path.poses[0];
  }

  // Find the closest point on the path to the robot
  // But ensure we're looking AHEAD, not behind
  double min_dist = std::numeric_limits<double>::max();
  size_t closest_idx = 0;
  size_t best_forward_idx = 0;
  bool found_forward = false;

  // First pass: find the closest point
  for (size_t i = 0; i < path.poses.size(); ++i) {
    double dist = computeDistance(
        robot_x, robot_y,
        path.poses[i].pose.position.x,
        path.poses[i].pose.position.y
    );
    if (dist < min_dist) {
      min_dist = dist;
      closest_idx = i;
    }
  }

  // Second pass: find the first point AHEAD of the robot that's at lookahead distance
  // We need to ensure we're looking forward along the path, not backward
  // Check if the closest point is ahead or behind by checking path direction
  size_t search_start = closest_idx;
  
  // If we're past the closest point (robot has moved beyond it), start from next point
  // Check if robot is ahead of the closest point by comparing distances to next point
  if (closest_idx < path.poses.size() - 1) {
    double dist_to_closest = computeDistance(
        robot_x, robot_y,
        path.poses[closest_idx].pose.position.x,
        path.poses[closest_idx].pose.position.y
    );
    double dist_to_next = computeDistance(
        robot_x, robot_y,
        path.poses[closest_idx + 1].pose.position.x,
        path.poses[closest_idx + 1].pose.position.y
    );
    // If next point is closer, we've passed the closest point
    if (dist_to_next < dist_to_closest) {
      search_start = closest_idx + 1;
    }
  }

  // Search forward from search_start to find lookahead point
  double furthest_dist = 0.0;
  for (size_t i = search_start; i < path.poses.size(); ++i) {
    double dist = computeDistance(
        robot_x, robot_y,
        path.poses[i].pose.position.x,
        path.poses[i].pose.position.y
    );

    // If we found a point at or beyond the lookahead distance, use it
    if (dist >= lookahead_distance_) {
      return path.poses[i];
    }
    
    // Track the furthest point we've seen (in case we don't find one at exact distance)
    if (dist > furthest_dist) {
      furthest_dist = dist;
      best_forward_idx = i;
      found_forward = true;
    }
  }

  // If we found a forward point but it's closer than lookahead, use it anyway
  // (we're close to goal)
  if (found_forward) {
    return path.poses[best_forward_idx];
  }

  // Fallback: use the final goal point
  return path.poses.back();
}

geometry_msgs::msg::Twist ControlCore::computeVelocity(
    const geometry_msgs::msg::PoseStamped& target,
    double robot_x,
    double robot_y,
    double robot_yaw) const
{
  geometry_msgs::msg::Twist cmd_vel;

  // Calculate vector from robot to target point
  double dx = target.pose.position.x - robot_x;
  double dy = target.pose.position.y - robot_y;

  // Calculate distance to target
  double distance = std::sqrt(dx * dx + dy * dy);

  // If we're very close to the target, stop
  if (distance < goal_tolerance_) {
    cmd_vel.linear.x = 0.0;
    cmd_vel.angular.z = 0.0;
    return cmd_vel;
  }

  // Calculate angle to target point
  double angle_to_target = std::atan2(dy, dx);

  // Calculate the angle difference (error)
  double angle_error = angle_to_target - robot_yaw;

  // Normalize angle error to [-pi, pi]
  while (angle_error > M_PI) {
    angle_error -= 2.0 * M_PI;
  }
  while (angle_error < -M_PI) {
    angle_error += 2.0 * M_PI;
  }

  // Pure Pursuit Control: Calculate curvature
  // The curvature is based on the geometry of a circle that passes through
  // the robot's current position and the lookahead point
  // Standard formula: curvature = 2 * sin(alpha) / L
  // where alpha is the angle from robot heading to lookahead point, L is lookahead distance
  // Use the lookahead_distance parameter for consistent behavior
  // When close to goal (distance < lookahead), use actual distance for better responsiveness
  double effective_lookahead = (distance < lookahead_distance_) ? distance : lookahead_distance_;
  double curvature = 2.0 * std::sin(angle_error) / effective_lookahead;

  // Set linear velocity (constant speed)
  // Reduce speed when turning sharply for better control and obstacle avoidance
  double speed_factor = 1.0;
  if (std::abs(angle_error) > M_PI / 3.0) {  // If turning more than 60 degrees
    speed_factor = 0.5;  // Reduce speed to 50% when making very sharp turns
  } else if (std::abs(angle_error) > M_PI / 4.0) {  // If turning more than 45 degrees
    speed_factor = 0.7;  // Reduce speed to 70% when making sharp turns
  }
  cmd_vel.linear.x = linear_speed_ * speed_factor;

  // Set angular velocity based on curvature
  // For a differential drive robot: omega = v * curvature
  cmd_vel.angular.z = cmd_vel.linear.x * curvature;

  // Limit angular velocity for safety and smoothness
  const double max_angular_vel = 1.5; // rad/s (increased for faster navigation)
  if (std::abs(cmd_vel.angular.z) > max_angular_vel) {
    // If angular velocity is too high, reduce linear speed proportionally
    double reduction = max_angular_vel / std::abs(cmd_vel.angular.z);
    cmd_vel.angular.z = (cmd_vel.angular.z > 0) ? max_angular_vel : -max_angular_vel;
    cmd_vel.linear.x *= reduction;  // Reduce linear speed to maintain smooth motion
  }

  return cmd_vel;
}

bool ControlCore::goalReached(const nav_msgs::msg::Path& path,
                              double robot_x,
                              double robot_y) const
{
  if (path.poses.empty()) {
    return false;
  }

  // Check distance to final goal point
  const auto& goal = path.poses.back();
  double distance = computeDistance(
      robot_x, robot_y,
      goal.pose.position.x,
      goal.pose.position.y
  );

  return distance < goal_tolerance_;
}

double ControlCore::computeDistance(double x1, double y1, double x2, double y2)
{
  double dx = x2 - x1;
  double dy = y2 - y1;
  return std::sqrt(dx * dx + dy * dy);
}

double ControlCore::extractYaw(const geometry_msgs::msg::Quaternion& quat)
{
  // Convert quaternion to yaw (rotation around z-axis)
  // Formula: yaw = atan2(2*(w*z + x*y), 1 - 2*(y^2 + z^2))
  // But for ROS2, the quaternion is (x, y, z, w)
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

}  
