/*
 * T-ESKF: Trajectory Interface for Robot Control
 * Copyright (C) 2024 T-ESKF Contributors
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 */

#include "robot_sim/Trajectory.h"
#include <algorithm>
#include <cmath>

namespace ov_msckf {

// WaypointTrajectory implementation
WaypointTrajectory::WaypointTrajectory(const std::vector<TrajectoryPoint>& waypoints) 
  : waypoints_(waypoints) {
  // Sort waypoints by timestamp
  std::sort(waypoints_.begin(), waypoints_.end(), 
           [](const TrajectoryPoint& a, const TrajectoryPoint& b) {
             return a.timestamp < b.timestamp;
           });
}

TrajectoryPoint WaypointTrajectory::getDesiredState(double time) {
  if (waypoints_.empty()) {
    return TrajectoryPoint();
  }
  
  // Find the two waypoints to interpolate between
  if (time <= waypoints_.front().timestamp) {
    return waypoints_.front();
  }
  
  if (time >= waypoints_.back().timestamp) {
    return waypoints_.back();
  }
  
  // Find the interval containing the current time
  for (size_t i = 0; i < waypoints_.size() - 1; ++i) {
    if (time >= waypoints_[i].timestamp && time <= waypoints_[i + 1].timestamp) {
      return interpolate(waypoints_[i], waypoints_[i + 1], time);
    }
  }
  
  return waypoints_.back();
}

bool WaypointTrajectory::isFinished(double time) {
  if (waypoints_.empty()) return true;
  return time >= waypoints_.back().timestamp;
}

double WaypointTrajectory::getDuration() {
  if (waypoints_.empty()) return 0.0;
  return waypoints_.back().timestamp - waypoints_.front().timestamp;
}

void WaypointTrajectory::reset() {
  // Nothing to reset for waypoint trajectory
}

void WaypointTrajectory::addWaypoint(const TrajectoryPoint& waypoint) {
  waypoints_.push_back(waypoint);
  // Re-sort waypoints
  std::sort(waypoints_.begin(), waypoints_.end(), 
           [](const TrajectoryPoint& a, const TrajectoryPoint& b) {
             return a.timestamp < b.timestamp;
           });
}

TrajectoryPoint WaypointTrajectory::interpolate(const TrajectoryPoint& p1, 
                                               const TrajectoryPoint& p2, 
                                               double time) {
  double dt = p2.timestamp - p1.timestamp;
  if (dt <= 0) return p1;
  
  double alpha = (time - p1.timestamp) / dt;
  alpha = std::max(0.0, std::min(alpha, 1.0));
  
  TrajectoryPoint result;
  result.timestamp = time;
  result.x = p1.x + alpha * (p2.x - p1.x);
  result.y = p1.y + alpha * (p2.y - p1.y);
  
  // Interpolate angles carefully to handle wrap-around
  double angle_diff = p2.theta - p1.theta;
  while (angle_diff > M_PI) angle_diff -= 2 * M_PI;
  while (angle_diff < -M_PI) angle_diff += 2 * M_PI;
  result.theta = p1.theta + alpha * angle_diff;
  
  result.v = p1.v + alpha * (p2.v - p1.v);
  result.omega = p1.omega + alpha * (p2.omega - p1.omega);
  
  return result;
}

// CircleTrajectory implementation
CircleTrajectory::CircleTrajectory(double center_x, double center_y, double radius,
                                 double angular_velocity, double duration)
  : center_x_(center_x), center_y_(center_y), radius_(radius),
    angular_velocity_(angular_velocity), duration_(duration), start_time_(0) {}

TrajectoryPoint CircleTrajectory::getDesiredState(double time) {
  double t = time - start_time_;
  if (t < 0) t = 0;
  if (t > duration_) t = duration_;
  
  double angle = angular_velocity_ * t;
  
  TrajectoryPoint point;
  point.timestamp = time;
  point.x = center_x_ + radius_ * std::cos(angle);
  point.y = center_y_ + radius_ * std::sin(angle);
  point.theta = angle + M_PI / 2; // Tangent to circle
  point.v = radius_ * std::abs(angular_velocity_);
  point.omega = angular_velocity_;
  
  return point;
}

bool CircleTrajectory::isFinished(double time) {
  return (time - start_time_) >= duration_;
}

double CircleTrajectory::getDuration() {
  return duration_;
}

void CircleTrajectory::reset() {
  start_time_ = 0;
}

// Figure8Trajectory implementation
Figure8Trajectory::Figure8Trajectory(double center_x, double center_y, double scale,
                                   double angular_velocity, double duration)
  : center_x_(center_x), center_y_(center_y), scale_(scale),
    angular_velocity_(angular_velocity), duration_(duration), start_time_(0) {}

TrajectoryPoint Figure8Trajectory::getDesiredState(double time) {
  double t = time - start_time_;
  if (t < 0) t = 0;
  if (t > duration_) t = duration_;
  
  double angle = angular_velocity_ * t;
  
  // Parametric equations for figure-8 (lemniscate)
  double cos_t = std::cos(angle);
  double sin_t = std::sin(angle);
  double denom = 1 + sin_t * sin_t;
  
  TrajectoryPoint point;
  point.timestamp = time;
  point.x = center_x_ + scale_ * cos_t / denom;
  point.y = center_y_ + scale_ * sin_t * cos_t / denom;
  
  // Compute derivatives for velocity
  double dx_dt = -scale_ * angular_velocity_ * sin_t * (2 - sin_t * sin_t) / (denom * denom);
  double dy_dt = scale_ * angular_velocity_ * (cos_t * cos_t - sin_t * sin_t) / (denom * denom);
  
  point.theta = std::atan2(dy_dt, dx_dt);
  point.v = std::sqrt(dx_dt * dx_dt + dy_dt * dy_dt);
  
  // Approximate angular velocity from curvature
  point.omega = angular_velocity_;
  
  return point;
}

bool Figure8Trajectory::isFinished(double time) {
  return (time - start_time_) >= duration_;
}

double Figure8Trajectory::getDuration() {
  return duration_;
}

void Figure8Trajectory::reset() {
  start_time_ = 0;
}

// LineTrajectory implementation
LineTrajectory::LineTrajectory(double start_x, double start_y, double end_x, double end_y,
                              double velocity)
  : start_x_(start_x), start_y_(start_y), end_x_(end_x), end_y_(end_y),
    velocity_(velocity), start_time_(0) {
  
  distance_ = std::sqrt((end_x - start_x) * (end_x - start_x) + 
                       (end_y - start_y) * (end_y - start_y));
  duration_ = distance_ / velocity_;
  angle_ = std::atan2(end_y - start_y, end_x - start_x);
}

TrajectoryPoint LineTrajectory::getDesiredState(double time) {
  double t = time - start_time_;
  if (t < 0) t = 0;
  if (t > duration_) t = duration_;
  
  double progress = velocity_ * t;
  double alpha = (distance_ > 0) ? progress / distance_ : 1.0;
  alpha = std::max(0.0, std::min(alpha, 1.0));
  
  TrajectoryPoint point;
  point.timestamp = time;
  point.x = start_x_ + alpha * (end_x_ - start_x_);
  point.y = start_y_ + alpha * (end_y_ - start_y_);
  point.theta = angle_;
  point.v = (alpha < 1.0) ? velocity_ : 0.0;
  point.omega = 0.0;
  
  return point;
}

bool LineTrajectory::isFinished(double time) {
  return (time - start_time_) >= duration_;
}

double LineTrajectory::getDuration() {
  return duration_;
}

void LineTrajectory::reset() {
  start_time_ = 0;
}

} // namespace ov_msckf