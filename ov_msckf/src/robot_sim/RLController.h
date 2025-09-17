/*
 * T-ESKF: Reinforcement Learning Controller for Robot Trajectory Tracking
 * Copyright (C) 2024 T-ESKF Contributors
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 */

#ifndef OV_MSCKF_RL_CONTROLLER_H
#define OV_MSCKF_RL_CONTROLLER_H

#include "robot_sim/DualWheelRobot.h"
#include "robot_sim/Trajectory.h"
#include <Eigen/Eigen>
#include <random>
#include <vector>
#include <deque>

namespace ov_msckf {

/**
 * @brief Simple neural network for RL controller
 */
class SimpleNetwork {
public:
  /**
   * @brief Constructor
   * @param input_size Number of input neurons
   * @param hidden_size Number of hidden neurons
   * @param output_size Number of output neurons
   */
  SimpleNetwork(int input_size, int hidden_size, int output_size);

  /**
   * @brief Forward pass through network
   * @param input Input vector
   * @return Output vector
   */
  Eigen::VectorXd forward(const Eigen::VectorXd& input) const;

  /**
   * @brief Get network parameters
   * @return Flattened parameter vector
   */
  Eigen::VectorXd getParameters() const;

  /**
   * @brief Set network parameters
   * @param params Flattened parameter vector
   */
  void setParameters(const Eigen::VectorXd& params);

  /**
   * @brief Get number of parameters
   * @return Total parameter count
   */
  int getParameterCount() const;

private:
  Eigen::MatrixXd W1_, W2_;  // Weight matrices
  Eigen::VectorXd b1_, b2_;  // Bias vectors
  int input_size_, hidden_size_, output_size_;

  /**
   * @brief Tanh activation function
   * @param x Input
   * @return Activated output
   */
  double tanh_activation(double x) const;
};

/**
 * @brief Experience replay buffer for RL training
 */
struct Experience {
  Eigen::VectorXd state;
  Eigen::VectorXd action;
  double reward;
  Eigen::VectorXd next_state;
  bool done;
  
  Experience() : reward(0), done(false) {}
};

/**
 * @brief Simple RL Controller using evolutionary strategy
 */
class RLController {
public:
  /**
   * @brief RL Controller parameters
   */
  struct RLParams {
    int population_size;        ///< Population size for evolutionary strategy
    double mutation_rate;       ///< Mutation rate for evolution
    double mutation_strength;   ///< Strength of mutations
    int max_generations;        ///< Maximum training generations
    double learning_rate;       ///< Learning rate for parameter updates
    
    // State and action space dimensions
    int state_dim;             ///< State space dimension
    int action_dim;            ///< Action space dimension
    int hidden_size;           ///< Hidden layer size
    
    // Reward parameters
    double position_weight;     ///< Weight for position error in reward
    double velocity_weight;     ///< Weight for velocity error in reward
    double orientation_weight;  ///< Weight for orientation error in reward
    double control_penalty;     ///< Penalty for large control inputs
    
    RLParams() : population_size(50), mutation_rate(0.1), mutation_strength(0.1),
                max_generations(100), learning_rate(0.01), state_dim(10), action_dim(2),
                hidden_size(32), position_weight(1.0), velocity_weight(0.5),
                orientation_weight(0.8), control_penalty(0.1) {}
  };

public:
  /**
   * @brief Constructor
   * @param params RL parameters
   */
  RLController(const RLParams& params = RLParams());

  /**
   * @brief Compute control input using trained policy
   * @param current_state Current robot state
   * @param desired_state Desired robot state from trajectory
   * @return Control input for robot
   */
  DualWheelRobot::ControlInput computeControl(const DualWheelRobot::RobotState& current_state,
                                             const TrajectoryPoint& desired_state);

  /**
   * @brief Train the RL controller using evolutionary strategy
   * @param robot Robot simulator for training
   * @param trajectory Training trajectory
   * @param episodes Number of training episodes
   */
  void train(DualWheelRobot& robot, std::shared_ptr<TrajectoryBase> trajectory, int episodes);

  /**
   * @brief Reset controller state
   */
  void reset();

  /**
   * @brief Get current parameters
   * @return RL parameters
   */
  RLParams getParams() const { return params_; }

  /**
   * @brief Set parameters
   * @param params New RL parameters
   */
  void setParams(const RLParams& params) { params_ = params; }

  /**
   * @brief Save trained policy to file
   * @param filename File to save to
   */
  void savePolicy(const std::string& filename);

  /**
   * @brief Load trained policy from file
   * @param filename File to load from
   * @return True if successful
   */
  bool loadPolicy(const std::string& filename);

  /**
   * @brief Get training statistics
   * @return Vector of episode rewards during training
   */
  std::vector<double> getTrainingStats() const { return training_rewards_; }

private:
  RLParams params_;                    ///< RL parameters
  std::unique_ptr<SimpleNetwork> policy_; ///< Policy network
  std::mt19937 rng_;                   ///< Random number generator
  std::vector<double> training_rewards_; ///< Training episode rewards
  
  // Experience replay (for future extensions)
  std::deque<Experience> replay_buffer_;
  static const size_t MAX_BUFFER_SIZE = 10000;

  /**
   * @brief Convert robot state and trajectory point to RL state vector
   * @param current_state Current robot state
   * @param desired_state Desired trajectory state
   * @return State vector for RL
   */
  Eigen::VectorXd constructStateVector(const DualWheelRobot::RobotState& current_state,
                                      const TrajectoryPoint& desired_state);

  /**
   * @brief Convert RL action vector to robot control input
   * @param action RL action vector
   * @return Robot control input
   */
  DualWheelRobot::ControlInput actionToControl(const Eigen::VectorXd& action);

  /**
   * @brief Compute reward for current state and action
   * @param current_state Current robot state
   * @param desired_state Desired trajectory state
   * @param control Applied control input
   * @return Reward value
   */
  double computeReward(const DualWheelRobot::RobotState& current_state,
                      const TrajectoryPoint& desired_state,
                      const DualWheelRobot::ControlInput& control);

  /**
   * @brief Evaluate policy on trajectory
   * @param robot Robot simulator
   * @param trajectory Evaluation trajectory
   * @param network Policy network to evaluate
   * @return Episode reward
   */
  double evaluatePolicy(DualWheelRobot& robot, 
                       std::shared_ptr<TrajectoryBase> trajectory,
                       const SimpleNetwork& network);

  /**
   * @brief Mutate network parameters
   * @param network Network to mutate
   * @return Mutated network
   */
  SimpleNetwork mutateNetwork(const SimpleNetwork& network);
};

} // namespace ov_msckf

#endif // OV_MSCKF_RL_CONTROLLER_H