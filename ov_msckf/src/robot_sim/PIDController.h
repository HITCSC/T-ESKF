/*
 * T-ESKF: PID Controller for Robot Trajectory Tracking
 * Copyright (C) 2024 T-ESKF Contributors
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 */

#ifndef OV_MSCKF_PID_CONTROLLER_H
#define OV_MSCKF_PID_CONTROLLER_H

#include "robot_sim/DualWheelRobot.h"
#include "robot_sim/Trajectory.h"
#include <Eigen/Eigen>

namespace ov_msckf {

/**
 * @brief PID Controller for dual-wheel robot trajectory tracking
 */
class PIDController {
public:
  /**
   * @brief PID parameters structure
   */
  struct PIDParams {
    // Position control parameters
    double kp_x;      ///< Proportional gain for X position
    double ki_x;      ///< Integral gain for X position
    double kd_x;      ///< Derivative gain for X position
    
    double kp_y;      ///< Proportional gain for Y position
    double ki_y;      ///< Integral gain for Y position
    double kd_y;      ///< Derivative gain for Y position
    
    // Orientation control parameters
    double kp_theta;  ///< Proportional gain for orientation
    double ki_theta;  ///< Integral gain for orientation
    double kd_theta;  ///< Derivative gain for orientation
    
    // Velocity control parameters
    double kp_v;      ///< Proportional gain for linear velocity
    double ki_v;      ///< Integral gain for linear velocity
    double kd_v;      ///< Derivative gain for linear velocity
    
    double kp_omega;  ///< Proportional gain for angular velocity
    double ki_omega;  ///< Integral gain for angular velocity
    double kd_omega;  ///< Derivative gain for angular velocity
    
    // Control limits
    double max_linear_accel;  ///< Maximum linear acceleration output
    double max_angular_accel; ///< Maximum angular acceleration output
    
    // Error integration limits
    double max_integral_error; ///< Maximum integral error to prevent windup
    
    PIDParams() : kp_x(2.0), ki_x(0.1), kd_x(0.5),
                  kp_y(2.0), ki_y(0.1), kd_y(0.5),
                  kp_theta(3.0), ki_theta(0.2), kd_theta(0.8),
                  kp_v(1.5), ki_v(0.05), kd_v(0.3),
                  kp_omega(2.0), ki_omega(0.1), kd_omega(0.4),
                  max_linear_accel(1.0), max_angular_accel(3.14),
                  max_integral_error(1.0) {}
  };

private:
  /**
   * @brief Individual PID controller state
   */
  struct PIDState {
    double error_integral;
    double previous_error;
    bool initialized;
    
    PIDState() : error_integral(0), previous_error(0), initialized(false) {}
    
    void reset() {
      error_integral = 0;
      previous_error = 0;
      initialized = false;
    }
  };

public:
  /**
   * @brief Constructor
   * @param params PID parameters
   */
  PIDController(const PIDParams& params = PIDParams());

  /**
   * @brief Compute control input for trajectory tracking
   * @param current_state Current robot state
   * @param desired_state Desired robot state from trajectory
   * @param dt Time step since last control update
   * @return Control input for robot
   */
  DualWheelRobot::ControlInput computeControl(const DualWheelRobot::RobotState& current_state,
                                             const TrajectoryPoint& desired_state,
                                             double dt);

  /**
   * @brief Reset controller state (clear integrals)
   */
  void reset();

  /**
   * @brief Get current PID parameters
   * @return PID parameters
   */
  PIDParams getParams() const { return params_; }

  /**
   * @brief Set PID parameters
   * @param params New PID parameters
   */
  void setParams(const PIDParams& params) { params_ = params; }

  /**
   * @brief Get tracking errors
   * @return Vector of current tracking errors [x, y, theta, v, omega]
   */
  Eigen::VectorXd getTrackingErrors() const;

  /**
   * @brief Get integral errors for debugging
   * @return Vector of integral errors [x, y, theta, v, omega]
   */
  Eigen::VectorXd getIntegralErrors() const;

private:
  PIDParams params_;            ///< PID controller parameters
  
  // Individual PID controllers for each state variable
  PIDState pid_x_;              ///< X position PID state
  PIDState pid_y_;              ///< Y position PID state
  PIDState pid_theta_;          ///< Orientation PID state
  PIDState pid_v_;              ///< Linear velocity PID state
  PIDState pid_omega_;          ///< Angular velocity PID state
  
  // Current errors for monitoring
  double current_error_x_;
  double current_error_y_;
  double current_error_theta_;
  double current_error_v_;
  double current_error_omega_;

  /**
   * @brief Compute PID output for a single variable
   * @param error Current error
   * @param kp Proportional gain
   * @param ki Integral gain
   * @param kd Derivative gain
   * @param pid_state PID state for this variable
   * @param dt Time step
   * @param max_integral Maximum integral value
   * @return PID output
   */
  double computePID(double error, double kp, double ki, double kd,
                   PIDState& pid_state, double dt, double max_integral);

  /**
   * @brief Normalize angle to [-π, π]
   * @param angle Input angle
   * @return Normalized angle
   */
  double normalizeAngle(double angle);

  /**
   * @brief Compute angle error with proper wrap-around handling
   * @param desired Desired angle
   * @param current Current angle
   * @return Angle error in [-π, π]
   */
  double computeAngleError(double desired, double current);
};

} // namespace ov_msckf

#endif // OV_MSCKF_PID_CONTROLLER_H