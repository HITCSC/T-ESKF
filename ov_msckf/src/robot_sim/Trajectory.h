/*
 * T-ESKF: Trajectory Interface for Robot Control
 * Copyright (C) 2024 T-ESKF Contributors
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 */

#ifndef OV_MSCKF_TRAJECTORY_H
#define OV_MSCKF_TRAJECTORY_H

#include <Eigen/Eigen>
#include <vector>
#include <functional>

namespace ov_msckf {

/**
 * @brief Trajectory waypoint structure
 */
struct TrajectoryPoint {
  double timestamp;  ///< Time for this waypoint
  double x;         ///< X position
  double y;         ///< Y position
  double theta;     ///< Desired orientation
  double v;         ///< Desired linear velocity
  double omega;     ///< Desired angular velocity
  
  TrajectoryPoint() : timestamp(0), x(0), y(0), theta(0), v(0), omega(0) {}
  TrajectoryPoint(double t, double x_val, double y_val, double theta_val = 0, 
                 double v_val = 0, double omega_val = 0)
    : timestamp(t), x(x_val), y(y_val), theta(theta_val), v(v_val), omega(omega_val) {}
};

/**
 * @brief Base class for trajectory generators
 */
class TrajectoryBase {
public:
  virtual ~TrajectoryBase() = default;
  
  /**
   * @brief Get desired state at given time
   * @param time Current time
   * @return Desired trajectory point
   */
  virtual TrajectoryPoint getDesiredState(double time) = 0;
  
  /**
   * @brief Check if trajectory is finished
   * @param time Current time
   * @return True if trajectory is complete
   */
  virtual bool isFinished(double time) = 0;
  
  /**
   * @brief Get trajectory duration
   * @return Total trajectory time
   */
  virtual double getDuration() = 0;
  
  /**
   * @brief Reset trajectory to start
   */
  virtual void reset() = 0;
};

/**
 * @brief Waypoint-based trajectory
 */
class WaypointTrajectory : public TrajectoryBase {
public:
  /**
   * @brief Constructor
   * @param waypoints List of trajectory waypoints
   */
  WaypointTrajectory(const std::vector<TrajectoryPoint>& waypoints);
  
  TrajectoryPoint getDesiredState(double time) override;
  bool isFinished(double time) override;
  double getDuration() override;
  void reset() override;
  
  /**
   * @brief Add waypoint to trajectory
   * @param waypoint New waypoint to add
   */
  void addWaypoint(const TrajectoryPoint& waypoint);
  
  /**
   * @brief Get all waypoints
   * @return Vector of waypoints
   */
  std::vector<TrajectoryPoint> getWaypoints() const { return waypoints_; }

private:
  std::vector<TrajectoryPoint> waypoints_;
  
  /**
   * @brief Interpolate between two waypoints
   * @param p1 First waypoint
   * @param p2 Second waypoint
   * @param time Current time
   * @return Interpolated point
   */
  TrajectoryPoint interpolate(const TrajectoryPoint& p1, const TrajectoryPoint& p2, double time);
};

/**
 * @brief Circle trajectory generator
 */
class CircleTrajectory : public TrajectoryBase {
public:
  /**
   * @brief Constructor
   * @param center_x Circle center X
   * @param center_y Circle center Y
   * @param radius Circle radius
   * @param angular_velocity Angular velocity for circle
   * @param duration Total duration
   */
  CircleTrajectory(double center_x, double center_y, double radius, 
                  double angular_velocity, double duration);
  
  TrajectoryPoint getDesiredState(double time) override;
  bool isFinished(double time) override;
  double getDuration() override;
  void reset() override;

private:
  double center_x_, center_y_;
  double radius_;
  double angular_velocity_;
  double duration_;
  double start_time_;
};

/**
 * @brief Figure-8 trajectory generator
 */
class Figure8Trajectory : public TrajectoryBase {
public:
  /**
   * @brief Constructor
   * @param center_x Center X position
   * @param center_y Center Y position
   * @param scale Scale factor for figure-8
   * @param angular_velocity Angular velocity
   * @param duration Total duration
   */
  Figure8Trajectory(double center_x, double center_y, double scale,
                   double angular_velocity, double duration);
  
  TrajectoryPoint getDesiredState(double time) override;
  bool isFinished(double time) override;
  double getDuration() override;
  void reset() override;

private:
  double center_x_, center_y_;
  double scale_;
  double angular_velocity_;
  double duration_;
  double start_time_;
};

/**
 * @brief Straight line trajectory generator
 */
class LineTrajectory : public TrajectoryBase {
public:
  /**
   * @brief Constructor
   * @param start_x Start X position
   * @param start_y Start Y position
   * @param end_x End X position
   * @param end_y End Y position
   * @param velocity Linear velocity
   */
  LineTrajectory(double start_x, double start_y, double end_x, double end_y, 
                double velocity);
  
  TrajectoryPoint getDesiredState(double time) override;
  bool isFinished(double time) override;
  double getDuration() override;
  void reset() override;

private:
  double start_x_, start_y_;
  double end_x_, end_y_;
  double velocity_;
  double duration_;
  double start_time_;
  double distance_;
  double angle_;
};

} // namespace ov_msckf

#endif // OV_MSCKF_TRAJECTORY_H