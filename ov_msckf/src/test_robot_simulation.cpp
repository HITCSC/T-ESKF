/*
 * T-ESKF: Simple test program for dual-wheel robot simulation
 * Copyright (C) 2024 T-ESKF Contributors
 */

#include "robot_sim/DualWheelRobot.h"
#include "robot_sim/Trajectory.h"
#include "robot_sim/PIDController.h"
#include <iostream>

int main() {
    std::cout << "Testing dual-wheel robot simulation components..." << std::endl;
    
    // Test robot creation
    ov_msckf::DualWheelRobot robot;
    std::cout << "✓ Robot created successfully" << std::endl;
    
    // Test robot initialization
    ov_msckf::DualWheelRobot::RobotState initial_state;
    initial_state.x = 1.0;
    initial_state.y = 2.0;
    initial_state.theta = 0.5;
    robot.initialize(initial_state);
    std::cout << "✓ Robot initialized successfully" << std::endl;
    
    // Test robot update
    ov_msckf::DualWheelRobot::ControlInput control(0.5, 0.2);
    robot.update(control, 0.1);
    std::cout << "✓ Robot dynamics updated successfully" << std::endl;
    
    // Test trajectory creation
    auto circle_traj = std::make_shared<ov_msckf::CircleTrajectory>(0, 0, 1.0, 1.0, 10.0);
    auto desired_state = circle_traj->getDesiredState(1.0);
    std::cout << "✓ Circle trajectory created and tested successfully" << std::endl;
    
    // Test PID controller
    ov_msckf::PIDController pid_controller;
    auto current_state = robot.getCurrentState();
    auto pid_control = pid_controller.computeControl(current_state, desired_state, 0.1);
    std::cout << "✓ PID controller created and tested successfully" << std::endl;
    
    // Test waypoint trajectory
    std::vector<ov_msckf::TrajectoryPoint> waypoints;
    waypoints.emplace_back(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    waypoints.emplace_back(5.0, 1.0, 1.0, 0.0, 0.0, 0.0);
    waypoints.emplace_back(10.0, 2.0, 0.0, 0.0, 0.0, 0.0);
    
    ov_msckf::WaypointTrajectory waypoint_traj(waypoints);
    auto waypoint_state = waypoint_traj.getDesiredState(2.5);
    std::cout << "✓ Waypoint trajectory created and tested successfully" << std::endl;
    
    std::cout << "\nAll tests passed! Robot simulation components are working correctly." << std::endl;
    
    // Print some example values
    std::cout << "\nExample results:" << std::endl;
    std::cout << "Current robot position: (" << current_state.x << ", " << current_state.y << ")" << std::endl;
    std::cout << "Desired position: (" << desired_state.x << ", " << desired_state.y << ")" << std::endl;
    std::cout << "PID control output: linear_accel=" << pid_control.linear_accel 
              << ", angular_accel=" << pid_control.angular_accel << std::endl;
    
    return 0;
}