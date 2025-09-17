/*
 * T-ESKF: Dual-Wheel Robot Simulation with PID and RL Controllers
 * Copyright (C) 2024 T-ESKF Contributors
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 */

#include "robot_sim/DualWheelRobot.h"
#include "robot_sim/Trajectory.h"
#include "robot_sim/PIDController.h"
#include "robot_sim/RLController.h"

#include <iostream>
#include <fstream>
#include <memory>
#include <chrono>
#include <iomanip>

using namespace ov_msckf;

/**
 * @brief Write simulation results to CSV file
 */
void writeResultsToCSV(const std::string& filename,
                      const std::vector<DualWheelRobot::RobotState>& robot_states,
                      const std::vector<TrajectoryPoint>& trajectory_states,
                      const std::vector<DualWheelRobot::ControlInput>& controls) {
  std::ofstream file(filename);
  if (!file.is_open()) {
    std::cerr << "Could not open file: " << filename << std::endl;
    return;
  }
  
  // Write header
  file << "timestamp,robot_x,robot_y,robot_theta,robot_v,robot_omega,"
       << "desired_x,desired_y,desired_theta,desired_v,desired_omega,"
       << "linear_accel,angular_accel\n";
  
  // Write data
  size_t min_size = std::min({robot_states.size(), trajectory_states.size(), controls.size()});
  for (size_t i = 0; i < min_size; ++i) {
    const auto& robot = robot_states[i];
    const auto& traj = trajectory_states[i];
    const auto& ctrl = controls[i];
    
    file << std::fixed << std::setprecision(6)
         << robot.timestamp << ","
         << robot.x << "," << robot.y << "," << robot.theta << ","
         << robot.v << "," << robot.omega << ","
         << traj.x << "," << traj.y << "," << traj.theta << ","
         << traj.v << "," << traj.omega << ","
         << ctrl.linear_accel << "," << ctrl.angular_accel << "\n";
  }
  
  file.close();
  std::cout << "Results written to: " << filename << std::endl;
}

/**
 * @brief Compute tracking errors
 */
void computeTrackingErrors(const std::vector<DualWheelRobot::RobotState>& robot_states,
                          const std::vector<TrajectoryPoint>& trajectory_states) {
  if (robot_states.empty() || trajectory_states.empty()) {
    std::cout << "No data for error computation" << std::endl;
    return;
  }
  
  double total_pos_error = 0;
  double total_orient_error = 0;
  double max_pos_error = 0;
  double max_orient_error = 0;
  
  size_t min_size = std::min(robot_states.size(), trajectory_states.size());
  
  for (size_t i = 0; i < min_size; ++i) {
    const auto& robot = robot_states[i];
    const auto& traj = trajectory_states[i];
    
    // Position error
    double pos_error = std::sqrt(std::pow(traj.x - robot.x, 2) + 
                                std::pow(traj.y - robot.y, 2));
    total_pos_error += pos_error;
    max_pos_error = std::max(max_pos_error, pos_error);
    
    // Orientation error
    double angle_diff = traj.theta - robot.theta;
    while (angle_diff > M_PI) angle_diff -= 2 * M_PI;
    while (angle_diff < -M_PI) angle_diff += 2 * M_PI;
    double orient_error = std::abs(angle_diff);
    total_orient_error += orient_error;
    max_orient_error = std::max(max_orient_error, orient_error);
  }
  
  double avg_pos_error = total_pos_error / min_size;
  double avg_orient_error = total_orient_error / min_size;
  
  std::cout << std::fixed << std::setprecision(4);
  std::cout << "Tracking Performance:" << std::endl;
  std::cout << "  Average position error: " << avg_pos_error << " m" << std::endl;
  std::cout << "  Maximum position error: " << max_pos_error << " m" << std::endl;
  std::cout << "  Average orientation error: " << avg_orient_error << " rad" << std::endl;
  std::cout << "  Maximum orientation error: " << max_orient_error << " rad" << std::endl;
}

/**
 * @brief Run simulation with PID controller
 */
void runPIDSimulation(PIDController& controller,
                     DualWheelRobot& robot,
                     std::shared_ptr<TrajectoryBase> trajectory,
                     double dt = 0.05) {
  
  std::cout << "\n=== Running PID Controller Simulation ===" << std::endl;
  
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
  controller.reset();
  
  // Storage for results
  std::vector<DualWheelRobot::RobotState> robot_states;
  std::vector<TrajectoryPoint> trajectory_states;
  std::vector<DualWheelRobot::ControlInput> controls;
  
  double max_time = trajectory->getDuration();
  std::cout << "Trajectory duration: " << max_time << " seconds" << std::endl;
  
  auto start_time = std::chrono::high_resolution_clock::now();
  
  // Main simulation loop
  for (double time = 0; time < max_time; time += dt) {
    DualWheelRobot::RobotState current_state = robot.getCurrentState();
    current_state.timestamp = time;
    
    TrajectoryPoint desired_state = trajectory->getDesiredState(time);
    
    // Compute control
    DualWheelRobot::ControlInput control = controller.computeControl(current_state, desired_state, dt);
    
    // Update robot
    robot.update(control, dt);
    
    // Store results
    robot_states.push_back(current_state);
    trajectory_states.push_back(desired_state);
    controls.push_back(control);
    
    // Progress indicator
    if (static_cast<int>(time * 10) % static_cast<int>(max_time) == 0) {
      std::cout << "Progress: " << std::fixed << std::setprecision(1) 
               << (time / max_time * 100) << "%" << std::endl;
    }
  }
  
  auto end_time = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
  
  std::cout << "Simulation completed in " << duration.count() << " ms" << std::endl;
  
  // Compute and display tracking errors
  computeTrackingErrors(robot_states, trajectory_states);
  
  // Write results to file
  writeResultsToCSV("simulation_results_PID.csv", robot_states, trajectory_states, controls);
}

/**
 * @brief Run simulation with RL controller
 */
void runRLSimulation(RLController& controller,
                    DualWheelRobot& robot,
                    std::shared_ptr<TrajectoryBase> trajectory,
                    double dt = 0.05) {
  
  std::cout << "\n=== Running RL Controller Simulation ===" << std::endl;
  
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
  controller.reset();
  
  // Storage for results
  std::vector<DualWheelRobot::RobotState> robot_states;
  std::vector<TrajectoryPoint> trajectory_states;
  std::vector<DualWheelRobot::ControlInput> controls;
  
  double max_time = trajectory->getDuration();
  std::cout << "Trajectory duration: " << max_time << " seconds" << std::endl;
  
  auto start_time = std::chrono::high_resolution_clock::now();
  
  // Main simulation loop
  for (double time = 0; time < max_time; time += dt) {
    DualWheelRobot::RobotState current_state = robot.getCurrentState();
    current_state.timestamp = time;
    
    TrajectoryPoint desired_state = trajectory->getDesiredState(time);
    
    // Compute control
    DualWheelRobot::ControlInput control = controller.computeControl(current_state, desired_state);
    
    // Update robot
    robot.update(control, dt);
    
    // Store results
    robot_states.push_back(current_state);
    trajectory_states.push_back(desired_state);
    controls.push_back(control);
    
    // Progress indicator
    if (static_cast<int>(time * 10) % static_cast<int>(max_time) == 0) {
      std::cout << "Progress: " << std::fixed << std::setprecision(1) 
               << (time / max_time * 100) << "%" << std::endl;
    }
  }
  
  auto end_time = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
  
  std::cout << "Simulation completed in " << duration.count() << " ms" << std::endl;
  
  // Compute and display tracking errors
  computeTrackingErrors(robot_states, trajectory_states);
  
  // Write results to file
  writeResultsToCSV("simulation_results_RL.csv", robot_states, trajectory_states, controls);
}

/**
 * @brief Run simulation with specified controller
 */
template<typename ControllerType>
void runSimulation(const std::string& controller_name,
                  ControllerType& controller,
                  DualWheelRobot& robot,
                  std::shared_ptr<TrajectoryBase> trajectory,
                  double dt = 0.05) {
  // This template is now replaced by specific functions above
}

/**
 * @brief Create sample trajectories
 */
std::vector<std::shared_ptr<TrajectoryBase>> createSampleTrajectories() {
  std::vector<std::shared_ptr<TrajectoryBase>> trajectories;
  
  // 1. Circle trajectory
  auto circle = std::make_shared<CircleTrajectory>(2.0, 2.0, 1.5, 0.5, 20.0);
  trajectories.push_back(circle);
  
  // 2. Figure-8 trajectory
  auto figure8 = std::make_shared<Figure8Trajectory>(0.0, 0.0, 2.0, 0.3, 25.0);
  trajectories.push_back(figure8);
  
  // 3. Line trajectory
  auto line = std::make_shared<LineTrajectory>(0.0, 0.0, 5.0, 3.0, 1.0);
  trajectories.push_back(line);
  
  // 4. Waypoint trajectory
  std::vector<TrajectoryPoint> waypoints;
  waypoints.emplace_back(0.0, 0.0, 0.0, 0.0, 0.5, 0.0);
  waypoints.emplace_back(5.0, 2.0, 1.0, M_PI/4, 0.8, 0.2);
  waypoints.emplace_back(10.0, 3.0, 3.0, M_PI/2, 0.6, -0.1);
  waypoints.emplace_back(15.0, 1.0, 4.0, M_PI, 0.4, 0.3);
  waypoints.emplace_back(20.0, 0.0, 2.0, 0.0, 0.0, 0.0);
  
  auto waypoint_traj = std::make_shared<WaypointTrajectory>(waypoints);
  trajectories.push_back(waypoint_traj);
  
  return trajectories;
}

/**
 * @brief Main function
 */
int main(int argc, char** argv) {
  std::cout << "=== Dual-Wheel Robot Simulation with PID and RL Controllers ===" << std::endl;
  
  // Parse command line arguments
  std::string mode = "all"; // default: run all simulations
  int trajectory_idx = 0;   // default: circle trajectory
  
  if (argc >= 2) {
    mode = argv[1]; // "pid", "rl", "train", or "all"
  }
  if (argc >= 3) {
    trajectory_idx = std::atoi(argv[2]); // 0: circle, 1: figure8, 2: line, 3: waypoint
  }
  
  // Create robot
  DualWheelRobot::RobotParams robot_params;
  robot_params.wheelbase = 0.3;
  robot_params.max_linear_vel = 2.0;
  robot_params.max_angular_vel = 3.14;
  robot_params.max_linear_accel = 1.0;
  robot_params.max_angular_accel = 3.14;
  
  DualWheelRobot robot(robot_params);
  
  // Create trajectories
  auto trajectories = createSampleTrajectories();
  const std::vector<std::string> trajectory_names = {"Circle", "Figure-8", "Line", "Waypoint"};
  
  if (trajectory_idx >= static_cast<int>(trajectories.size())) {
    trajectory_idx = 0;
  }
  
  auto trajectory = trajectories[trajectory_idx];
  std::cout << "Using " << trajectory_names[trajectory_idx] << " trajectory" << std::endl;
  
  if (mode == "pid" || mode == "all") {
    // Create and tune PID controller
    PIDController::PIDParams pid_params;
    pid_params.kp_x = 2.0; pid_params.ki_x = 0.1; pid_params.kd_x = 0.5;
    pid_params.kp_y = 2.0; pid_params.ki_y = 0.1; pid_params.kd_y = 0.5;
    pid_params.kp_theta = 3.0; pid_params.ki_theta = 0.2; pid_params.kd_theta = 0.8;
    pid_params.kp_v = 1.5; pid_params.ki_v = 0.05; pid_params.kd_v = 0.3;
    pid_params.kp_omega = 2.0; pid_params.ki_omega = 0.1; pid_params.kd_omega = 0.4;
    
    PIDController pid_controller(pid_params);
    runPIDSimulation(pid_controller, robot, trajectory);
  }
  
  if (mode == "rl" || mode == "all") {
    // Create RL controller
    RLController::RLParams rl_params;
    rl_params.population_size = 30;
    rl_params.mutation_rate = 0.1;
    rl_params.mutation_strength = 0.05;
    
    RLController rl_controller(rl_params);
    
    // Try to load pre-trained policy
    if (!rl_controller.loadPolicy("rl_policy.txt")) {
      std::cout << "No pre-trained policy found. Training new policy..." << std::endl;
      rl_controller.train(robot, trajectory, 50);
      rl_controller.savePolicy("rl_policy.txt");
    } else {
      std::cout << "Loaded pre-trained policy" << std::endl;
    }
    
    runRLSimulation(rl_controller, robot, trajectory);
  }
  
  if (mode == "train") {
    // Train RL controller
    RLController::RLParams rl_params;
    rl_params.population_size = 50;
    rl_params.mutation_rate = 0.1;
    rl_params.mutation_strength = 0.1;
    
    RLController rl_controller(rl_params);
    
    std::cout << "Training RL controller..." << std::endl;
    rl_controller.train(robot, trajectory, 100);
    rl_controller.savePolicy("rl_policy.txt");
    
    // Display training statistics
    auto stats = rl_controller.getTrainingStats();
    std::cout << "\nTraining statistics:" << std::endl;
    for (size_t i = 0; i < stats.size(); i += 10) {
      std::cout << "Episode " << i << ": " << stats[i] << std::endl;
    }
  }
  
  std::cout << "\nSimulation complete!" << std::endl;
  
  return 0;
}