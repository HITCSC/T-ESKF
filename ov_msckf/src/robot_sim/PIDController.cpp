/*
 * T-ESKF: PID Controller for Robot Trajectory Tracking
 * Copyright (C) 2024 T-ESKF Contributors
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 */

#include "robot_sim/PIDController.h"
#include <algorithm>
#include <cmath>

namespace ov_msckf {

PIDController::PIDController(const PIDParams& params) : params_(params) {
  reset();
}

DualWheelRobot::ControlInput PIDController::computeControl(
    const DualWheelRobot::RobotState& current_state,
    const TrajectoryPoint& desired_state,
    double dt) {
  
  // Compute errors
  current_error_x_ = desired_state.x - current_state.x;
  current_error_y_ = desired_state.y - current_state.y;
  current_error_theta_ = computeAngleError(desired_state.theta, current_state.theta);
  current_error_v_ = desired_state.v - current_state.v;
  current_error_omega_ = desired_state.omega - current_state.omega;
  
  // Compute PID outputs
  double u_x = computePID(current_error_x_, params_.kp_x, params_.ki_x, params_.kd_x,
                         pid_x_, dt, params_.max_integral_error);
  
  double u_y = computePID(current_error_y_, params_.kp_y, params_.ki_y, params_.kd_y,
                         pid_y_, dt, params_.max_integral_error);
  
  double u_theta = computePID(current_error_theta_, params_.kp_theta, params_.ki_theta, 
                             params_.kd_theta, pid_theta_, dt, params_.max_integral_error);
  
  double u_v = computePID(current_error_v_, params_.kp_v, params_.ki_v, params_.kd_v,
                         pid_v_, dt, params_.max_integral_error);
  
  double u_omega = computePID(current_error_omega_, params_.kp_omega, params_.ki_omega,
                             params_.kd_omega, pid_omega_, dt, params_.max_integral_error);
  
  // Convert position and orientation errors to velocity commands
  // This is a common approach where position errors are converted to velocity references
  double desired_v_from_position = std::sqrt(u_x * u_x + u_y * u_y);
  double desired_omega_from_position = u_theta;
  
  // Combine position-based and direct velocity control
  double total_v_error = desired_v_from_position - current_state.v + u_v;
  double total_omega_error = desired_omega_from_position - current_state.omega + u_omega;
  
  // Create control input
  DualWheelRobot::ControlInput control;
  control.linear_accel = total_v_error;
  control.angular_accel = total_omega_error;
  
  // Apply control limits
  control.linear_accel = std::max(-params_.max_linear_accel, 
                                 std::min(control.linear_accel, params_.max_linear_accel));
  control.angular_accel = std::max(-params_.max_angular_accel,
                                  std::min(control.angular_accel, params_.max_angular_accel));
  
  return control;
}

void PIDController::reset() {
  pid_x_.reset();
  pid_y_.reset();
  pid_theta_.reset();
  pid_v_.reset();
  pid_omega_.reset();
  
  current_error_x_ = 0;
  current_error_y_ = 0;
  current_error_theta_ = 0;
  current_error_v_ = 0;
  current_error_omega_ = 0;
}

Eigen::VectorXd PIDController::getTrackingErrors() const {
  Eigen::VectorXd errors(5);
  errors << current_error_x_, current_error_y_, current_error_theta_, 
           current_error_v_, current_error_omega_;
  return errors;
}

Eigen::VectorXd PIDController::getIntegralErrors() const {
  Eigen::VectorXd integral_errors(5);
  integral_errors << pid_x_.error_integral, pid_y_.error_integral, 
                    pid_theta_.error_integral, pid_v_.error_integral, 
                    pid_omega_.error_integral;
  return integral_errors;
}

double PIDController::computePID(double error, double kp, double ki, double kd,
                                PIDState& pid_state, double dt, double max_integral) {
  // Update integral term
  pid_state.error_integral += error * dt;
  
  // Anti-windup: clamp integral term
  pid_state.error_integral = std::max(-max_integral, std::min(pid_state.error_integral, max_integral));
  
  // Compute derivative term
  double error_derivative = 0;
  if (pid_state.initialized && dt > 0) {
    error_derivative = (error - pid_state.previous_error) / dt;
  }
  
  // Update state for next iteration
  pid_state.previous_error = error;
  pid_state.initialized = true;
  
  // Compute PID output
  return kp * error + ki * pid_state.error_integral + kd * error_derivative;
}

double PIDController::normalizeAngle(double angle) {
  while (angle > M_PI) angle -= 2 * M_PI;
  while (angle < -M_PI) angle += 2 * M_PI;
  return angle;
}

double PIDController::computeAngleError(double desired, double current) {
  return normalizeAngle(desired - current);
}

} // namespace ov_msckf