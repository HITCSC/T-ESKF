/*
 * T-ESKF: Reinforcement Learning Controller for Robot Trajectory Tracking
 * Copyright (C) 2024 T-ESKF Contributors
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 */

#include "robot_sim/RLController.h"
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>

namespace ov_msckf {

// SimpleNetwork implementation
SimpleNetwork::SimpleNetwork(int input_size, int hidden_size, int output_size)
  : input_size_(input_size), hidden_size_(hidden_size), output_size_(output_size) {
  
  // Initialize weights and biases with small random values
  std::random_device rd;
  std::mt19937 gen(rd());
  std::normal_distribution<double> dist(0.0, 0.1);
  
  W1_ = Eigen::MatrixXd::Zero(hidden_size, input_size);
  W2_ = Eigen::MatrixXd::Zero(output_size, hidden_size);
  b1_ = Eigen::VectorXd::Zero(hidden_size);
  b2_ = Eigen::VectorXd::Zero(output_size);
  
  // Random initialization
  for (int i = 0; i < hidden_size; ++i) {
    for (int j = 0; j < input_size; ++j) {
      W1_(i, j) = dist(gen);
    }
    b1_(i) = dist(gen);
  }
  
  for (int i = 0; i < output_size; ++i) {
    for (int j = 0; j < hidden_size; ++j) {
      W2_(i, j) = dist(gen);
    }
    b2_(i) = dist(gen);
  }
}

Eigen::VectorXd SimpleNetwork::forward(const Eigen::VectorXd& input) const {
  // Hidden layer: h = tanh(W1 * input + b1)
  Eigen::VectorXd hidden = W1_ * input + b1_;
  for (int i = 0; i < hidden.size(); ++i) {
    hidden(i) = tanh_activation(hidden(i));
  }
  
  // Output layer: output = tanh(W2 * hidden + b2)
  Eigen::VectorXd output = W2_ * hidden + b2_;
  for (int i = 0; i < output.size(); ++i) {
    output(i) = tanh_activation(output(i));
  }
  
  return output;
}

Eigen::VectorXd SimpleNetwork::getParameters() const {
  int param_count = getParameterCount();
  Eigen::VectorXd params(param_count);
  
  int idx = 0;
  
  // W1
  for (int i = 0; i < W1_.rows(); ++i) {
    for (int j = 0; j < W1_.cols(); ++j) {
      params(idx++) = W1_(i, j);
    }
  }
  
  // b1
  for (int i = 0; i < b1_.size(); ++i) {
    params(idx++) = b1_(i);
  }
  
  // W2
  for (int i = 0; i < W2_.rows(); ++i) {
    for (int j = 0; j < W2_.cols(); ++j) {
      params(idx++) = W2_(i, j);
    }
  }
  
  // b2
  for (int i = 0; i < b2_.size(); ++i) {
    params(idx++) = b2_(i);
  }
  
  return params;
}

void SimpleNetwork::setParameters(const Eigen::VectorXd& params) {
  int idx = 0;
  
  // W1
  for (int i = 0; i < W1_.rows(); ++i) {
    for (int j = 0; j < W1_.cols(); ++j) {
      W1_(i, j) = params(idx++);
    }
  }
  
  // b1
  for (int i = 0; i < b1_.size(); ++i) {
    b1_(i) = params(idx++);
  }
  
  // W2
  for (int i = 0; i < W2_.rows(); ++i) {
    for (int j = 0; j < W2_.cols(); ++j) {
      W2_(i, j) = params(idx++);
    }
  }
  
  // b2
  for (int i = 0; i < b2_.size(); ++i) {
    b2_(i) = params(idx++);
  }
}

int SimpleNetwork::getParameterCount() const {
  return W1_.size() + b1_.size() + W2_.size() + b2_.size();
}

double SimpleNetwork::tanh_activation(double x) const {
  return std::tanh(x);
}

// RLController implementation
RLController::RLController(const RLParams& params) : params_(params), rng_(std::random_device{}()) {
  policy_ = std::make_unique<SimpleNetwork>(params_.state_dim, params_.hidden_size, params_.action_dim);
}

DualWheelRobot::ControlInput RLController::computeControl(
    const DualWheelRobot::RobotState& current_state,
    const TrajectoryPoint& desired_state) {
  
  // Construct state vector
  Eigen::VectorXd state_vec = constructStateVector(current_state, desired_state);
  
  // Get action from policy
  Eigen::VectorXd action = policy_->forward(state_vec);
  
  // Convert to control input
  return actionToControl(action);
}

void RLController::train(DualWheelRobot& robot, std::shared_ptr<TrajectoryBase> trajectory, int episodes) {
  training_rewards_.clear();
  
  // Evolutionary strategy parameters
  std::normal_distribution<double> mutation_dist(0.0, params_.mutation_strength);
  
  // Current best policy
  SimpleNetwork best_policy = *policy_;
  double best_reward = evaluatePolicy(robot, trajectory, best_policy);
  
  std::cout << "Initial policy reward: " << best_reward << std::endl;
  
  for (int episode = 0; episode < episodes; ++episode) {
    std::vector<SimpleNetwork> population;
    std::vector<double> rewards;
    
    // Generate population by mutating best policy
    for (int i = 0; i < params_.population_size; ++i) {
      SimpleNetwork mutated = mutateNetwork(best_policy);
      population.push_back(mutated);
      
      double reward = evaluatePolicy(robot, trajectory, mutated);
      rewards.push_back(reward);
    }
    
    // Find best individual in population
    auto max_it = std::max_element(rewards.begin(), rewards.end());
    int best_idx = std::distance(rewards.begin(), max_it);
    double episode_best_reward = *max_it;
    
    // Update best policy if improvement found
    if (episode_best_reward > best_reward) {
      best_policy = population[best_idx];
      best_reward = episode_best_reward;
      *policy_ = best_policy;
    }
    
    training_rewards_.push_back(best_reward);
    
    if (episode % 10 == 0) {
      std::cout << "Episode " << episode << ", Best reward: " << best_reward << std::endl;
    }
  }
  
  std::cout << "Training complete. Final reward: " << best_reward << std::endl;
}

void RLController::reset() {
  replay_buffer_.clear();
}

void RLController::savePolicy(const std::string& filename) {
  std::ofstream file(filename);
  if (file.is_open()) {
    Eigen::VectorXd params = policy_->getParameters();
    for (int i = 0; i < params.size(); ++i) {
      file << params(i) << "\n";
    }
    file.close();
  }
}

bool RLController::loadPolicy(const std::string& filename) {
  std::ifstream file(filename);
  if (!file.is_open()) {
    return false;
  }
  
  std::vector<double> param_values;
  double value;
  while (file >> value) {
    param_values.push_back(value);
  }
  file.close();
  
  if (param_values.size() != policy_->getParameterCount()) {
    return false;
  }
  
  Eigen::VectorXd params(param_values.size());
  for (size_t i = 0; i < param_values.size(); ++i) {
    params(i) = param_values[i];
  }
  
  policy_->setParameters(params);
  return true;
}

Eigen::VectorXd RLController::constructStateVector(const DualWheelRobot::RobotState& current_state,
                                                  const TrajectoryPoint& desired_state) {
  Eigen::VectorXd state_vec(params_.state_dim);
  
  // Current state
  state_vec(0) = current_state.x;
  state_vec(1) = current_state.y;
  state_vec(2) = current_state.theta;
  state_vec(3) = current_state.v;
  state_vec(4) = current_state.omega;
  
  // Desired state
  state_vec(5) = desired_state.x;
  state_vec(6) = desired_state.y;
  state_vec(7) = desired_state.theta;
  state_vec(8) = desired_state.v;
  state_vec(9) = desired_state.omega;
  
  return state_vec;
}

DualWheelRobot::ControlInput RLController::actionToControl(const Eigen::VectorXd& action) {
  DualWheelRobot::ControlInput control;
  
  // Scale actions to control limits
  control.linear_accel = action(0) * 1.0;   // Scale to ±1.0 m/s²
  control.angular_accel = action(1) * 3.14; // Scale to ±π rad/s²
  
  return control;
}

double RLController::computeReward(const DualWheelRobot::RobotState& current_state,
                                  const TrajectoryPoint& desired_state,
                                  const DualWheelRobot::ControlInput& control) {
  // Position error
  double pos_error = std::sqrt(std::pow(desired_state.x - current_state.x, 2) +
                              std::pow(desired_state.y - current_state.y, 2));
  
  // Orientation error
  double angle_diff = desired_state.theta - current_state.theta;
  while (angle_diff > M_PI) angle_diff -= 2 * M_PI;
  while (angle_diff < -M_PI) angle_diff += 2 * M_PI;
  double orient_error = std::abs(angle_diff);
  
  // Velocity error
  double vel_error = std::abs(desired_state.v - current_state.v) +
                    std::abs(desired_state.omega - current_state.omega);
  
  // Control penalty
  double control_cost = std::pow(control.linear_accel, 2) + std::pow(control.angular_accel, 2);
  
  // Compute reward (negative error + penalty)
  double reward = -params_.position_weight * pos_error
                 -params_.orientation_weight * orient_error
                 -params_.velocity_weight * vel_error
                 -params_.control_penalty * control_cost;
  
  return reward;
}

double RLController::evaluatePolicy(DualWheelRobot& robot, 
                                   std::shared_ptr<TrajectoryBase> trajectory,
                                   const SimpleNetwork& network) {
  // Reset robot and trajectory
  DualWheelRobot::RobotState initial_state;
  initial_state.x = 0;
  initial_state.y = 0;
  initial_state.theta = 0;
  initial_state.v = 0;
  initial_state.omega = 0;
  initial_state.timestamp = 0;
  
  robot.initialize(initial_state);
  trajectory->reset();
  
  double total_reward = 0;
  double dt = 0.05; // 20 Hz control
  double max_time = trajectory->getDuration();
  
  for (double time = 0; time < max_time; time += dt) {
    DualWheelRobot::RobotState current_state = robot.getCurrentState();
    TrajectoryPoint desired_state = trajectory->getDesiredState(time);
    
    // Get action from network
    Eigen::VectorXd state_vec = constructStateVector(current_state, desired_state);
    Eigen::VectorXd action = network.forward(state_vec);
    DualWheelRobot::ControlInput control = actionToControl(action);
    
    // Apply control and get reward
    robot.update(control, dt);
    double reward = computeReward(current_state, desired_state, control);
    total_reward += reward;
  }
  
  return total_reward;
}

SimpleNetwork RLController::mutateNetwork(const SimpleNetwork& network) {
  SimpleNetwork mutated = network;
  
  std::normal_distribution<double> mutation_dist(0.0, params_.mutation_strength);
  
  // Get parameters
  Eigen::VectorXd params = mutated.getParameters();
  
  // Mutate parameters
  for (int i = 0; i < params.size(); ++i) {
    if (std::uniform_real_distribution<double>(0.0, 1.0)(rng_) < params_.mutation_rate) {
      params(i) += mutation_dist(rng_);
    }
  }
  
  // Set mutated parameters
  mutated.setParameters(params);
  
  return mutated;
}

} // namespace ov_msckf