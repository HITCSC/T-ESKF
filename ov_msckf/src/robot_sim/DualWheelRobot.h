/*
 * T-ESKF: Dual-Wheel Robot Simulator
 * Copyright (C) 2024 T-ESKF Contributors
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 */

#ifndef OV_MSCKF_DUAL_WHEEL_ROBOT_H
#define OV_MSCKF_DUAL_WHEEL_ROBOT_H

#include <Eigen/Eigen>
#include <memory>
#include <vector>

namespace ov_msckf {

/**
 * @brief Dual-wheel drive robot simulator
 *
 * This class simulates a differential drive robot with two wheels.
 * The robot dynamics are based on angular and linear acceleration inputs.
 * State: [x, y, theta, v, omega] where:
 * - x, y: position
 * - theta: orientation
 * - v: linear velocity
 * - omega: angular velocity
 */
class DualWheelRobot {

public:
  /**
   * @brief Robot state structure
   */
  struct RobotState {
    double timestamp;     ///< Current time
    double x;            ///< X position
    double y;            ///< Y position
    double theta;        ///< Orientation (yaw angle)
    double v;            ///< Linear velocity
    double omega;        ///< Angular velocity
    
    RobotState() : timestamp(0), x(0), y(0), theta(0), v(0), omega(0) {}
    
    Eigen::VectorXd toVector() const {
      Eigen::VectorXd state(6);
      state << timestamp, x, y, theta, v, omega;
      return state;
    }
    
    void fromVector(const Eigen::VectorXd& vec) {
      timestamp = vec(0);
      x = vec(1);
      y = vec(2);
      theta = vec(3);
      v = vec(4);
      omega = vec(5);
    }
  };

  /**
   * @brief Control input structure
   */
  struct ControlInput {
    double linear_accel;   ///< Linear acceleration command
    double angular_accel;  ///< Angular acceleration command
    
    ControlInput() : linear_accel(0), angular_accel(0) {}
    ControlInput(double la, double aa) : linear_accel(la), angular_accel(aa) {}
  };

  /**
   * @brief Robot parameters
   */
  struct RobotParams {
    double wheelbase;        ///< Distance between wheels
    double max_linear_vel;   ///< Maximum linear velocity
    double max_angular_vel;  ///< Maximum angular velocity
    double max_linear_accel; ///< Maximum linear acceleration
    double max_angular_accel;///< Maximum angular acceleration
    
    RobotParams() : wheelbase(0.3), max_linear_vel(2.0), max_angular_vel(3.14),
                   max_linear_accel(1.0), max_angular_accel(3.14) {}
  };

public:
  /**
   * @brief Default constructor
   */
  DualWheelRobot();

  /**
   * @brief Constructor with parameters
   * @param params Robot parameters
   */
  DualWheelRobot(const RobotParams& params);

  /**
   * @brief Initialize robot with initial state
   * @param initial_state Initial robot state
   */
  void initialize(const RobotState& initial_state);

  /**
   * @brief Update robot state with control input
   * @param control Control input
   * @param dt Time step
   */
  void update(const ControlInput& control, double dt);

  /**
   * @brief Get current robot state
   * @return Current state
   */
  RobotState getCurrentState() const { return current_state_; }

  /**
   * @brief Get robot parameters
   * @return Robot parameters
   */
  RobotParams getParams() const { return params_; }

  /**
   * @brief Set robot parameters
   * @param params New parameters
   */
  void setParams(const RobotParams& params) { params_ = params; }

  /**
   * @brief Get state history
   * @return Vector of historical states
   */
  std::vector<RobotState> getStateHistory() const { return state_history_; }

  /**
   * @brief Clear state history
   */
  void clearHistory() { state_history_.clear(); }

  /**
   * @brief Check if control input is within limits
   * @param control Control input to check
   * @return True if within limits
   */
  bool isControlValid(const ControlInput& control) const;

  /**
   * @brief Clamp control input to valid range
   * @param control Control input to clamp
   * @return Clamped control input
   */
  ControlInput clampControl(const ControlInput& control) const;

private:
  RobotParams params_;           ///< Robot parameters
  RobotState current_state_;     ///< Current robot state
  std::vector<RobotState> state_history_; ///< State history for visualization

  /**
   * @brief Integrate robot dynamics using RK4
   * @param state Current state
   * @param control Control input
   * @param dt Time step
   * @return New state
   */
  RobotState integrateRK4(const RobotState& state, const ControlInput& control, double dt);

  /**
   * @brief Compute state derivatives
   * @param state Current state
   * @param control Control input
   * @return State derivatives
   */
  Eigen::VectorXd computeDerivatives(const RobotState& state, const ControlInput& control);
};

} // namespace ov_msckf

#endif // OV_MSCKF_DUAL_WHEEL_ROBOT_H