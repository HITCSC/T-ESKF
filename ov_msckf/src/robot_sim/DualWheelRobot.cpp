/*
 * T-ESKF: Dual-Wheel Robot Simulator
 * Copyright (C) 2024 T-ESKF Contributors
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 */

#include "robot_sim/DualWheelRobot.h"
#include <algorithm>
#include <cmath>

namespace ov_msckf {

DualWheelRobot::DualWheelRobot() {
  // Initialize with default parameters
  params_ = RobotParams();
  current_state_ = RobotState();
}

DualWheelRobot::DualWheelRobot(const RobotParams& params) : params_(params) {
  current_state_ = RobotState();
}

void DualWheelRobot::initialize(const RobotState& initial_state) {
  current_state_ = initial_state;
  state_history_.clear();
  state_history_.push_back(current_state_);
}

void DualWheelRobot::update(const ControlInput& control, double dt) {
  // Clamp control inputs to valid range
  ControlInput clamped_control = clampControl(control);
  
  // Integrate dynamics using RK4
  current_state_ = integrateRK4(current_state_, clamped_control, dt);
  
  // Update timestamp
  current_state_.timestamp += dt;
  
  // Store in history
  state_history_.push_back(current_state_);
}

bool DualWheelRobot::isControlValid(const ControlInput& control) const {
  return (std::abs(control.linear_accel) <= params_.max_linear_accel &&
          std::abs(control.angular_accel) <= params_.max_angular_accel);
}

DualWheelRobot::ControlInput DualWheelRobot::clampControl(const ControlInput& control) const {
  ControlInput clamped;
  clamped.linear_accel = std::max(-params_.max_linear_accel, 
                                 std::min(control.linear_accel, params_.max_linear_accel));
  clamped.angular_accel = std::max(-params_.max_angular_accel,
                                  std::min(control.angular_accel, params_.max_angular_accel));
  return clamped;
}

DualWheelRobot::RobotState DualWheelRobot::integrateRK4(const RobotState& state, 
                                                       const ControlInput& control, 
                                                       double dt) {
  // RK4 integration for robot dynamics
  Eigen::VectorXd state_vec = state.toVector();
  
  // k1 = f(t, y)
  Eigen::VectorXd k1 = computeDerivatives(state, control);
  
  // k2 = f(t + dt/2, y + k1*dt/2)
  RobotState state2;
  state2.fromVector(state_vec + k1 * dt / 2.0);
  Eigen::VectorXd k2 = computeDerivatives(state2, control);
  
  // k3 = f(t + dt/2, y + k2*dt/2)
  RobotState state3;
  state3.fromVector(state_vec + k2 * dt / 2.0);
  Eigen::VectorXd k3 = computeDerivatives(state3, control);
  
  // k4 = f(t + dt, y + k3*dt)
  RobotState state4;
  state4.fromVector(state_vec + k3 * dt);
  Eigen::VectorXd k4 = computeDerivatives(state4, control);
  
  // y_{n+1} = y_n + dt/6 * (k1 + 2*k2 + 2*k3 + k4)
  Eigen::VectorXd new_state_vec = state_vec + dt / 6.0 * (k1 + 2*k2 + 2*k3 + k4);
  
  RobotState new_state;
  new_state.fromVector(new_state_vec);
  
  // Normalize angle to [-π, π]
  while (new_state.theta > M_PI) new_state.theta -= 2 * M_PI;
  while (new_state.theta < -M_PI) new_state.theta += 2 * M_PI;
  
  // Clamp velocities to maximum values
  new_state.v = std::max(-params_.max_linear_vel, std::min(new_state.v, params_.max_linear_vel));
  new_state.omega = std::max(-params_.max_angular_vel, std::min(new_state.omega, params_.max_angular_vel));
  
  return new_state;
}

Eigen::VectorXd DualWheelRobot::computeDerivatives(const RobotState& state, 
                                                  const ControlInput& control) {
  Eigen::VectorXd derivatives(6);
  
  // State derivatives for dual-wheel robot:
  // dx/dt = v * cos(theta)
  // dy/dt = v * sin(theta)
  // dtheta/dt = omega
  // dv/dt = linear_accel
  // domega/dt = angular_accel
  
  derivatives(0) = 0; // timestamp derivative (not used in integration)
  derivatives(1) = state.v * std::cos(state.theta);  // dx/dt
  derivatives(2) = state.v * std::sin(state.theta);  // dy/dt
  derivatives(3) = state.omega;                      // dtheta/dt
  derivatives(4) = control.linear_accel;             // dv/dt
  derivatives(5) = control.angular_accel;            // domega/dt
  
  return derivatives;
}

} // namespace ov_msckf