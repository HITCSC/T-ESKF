# Dual-Wheel Robot Simulator with PID and RL Controllers

This implementation extends the T-ESKF framework with a complete dual-wheel differential drive robot simulator featuring both classical PID and modern reinforcement learning controllers for trajectory tracking.

## Features

### Robot Dynamics
- **Accurate Physics Model**: Differential drive kinematics with configurable robot parameters
- **RK4 Integration**: High-precision numerical integration for smooth dynamics
- **Control Inputs**: Angular and linear acceleration commands
- **State Representation**: Position (x, y), orientation (θ), linear velocity (v), angular velocity (ω)

### Trajectory Generation
- **Circle Trajectory**: Configurable radius and angular velocity
- **Figure-8 Trajectory**: Lemniscate pattern with adjustable scale
- **Line Trajectory**: Straight-line motion between waypoints
- **Waypoint Trajectory**: Smooth interpolation between multiple waypoints

### Controllers

#### PID Controller
- **Multi-variable Control**: Separate PID loops for position, orientation, and velocity
- **Anti-windup Protection**: Integral term clamping to prevent windup
- **Tunable Parameters**: All gains configurable for optimization

#### RL Controller
- **Neural Network Policy**: Simple feedforward network with tanh activation
- **Evolutionary Strategy**: Population-based training without gradients
- **State Space**: 10D state including current and desired robot states
- **Action Space**: 2D continuous control (linear and angular acceleration)

### Data Logging & Analysis
- **CSV Export**: Complete simulation data for external analysis
- **Performance Metrics**: Automatic calculation of tracking errors
- **Visualization**: Python script for plotting trajectories and performance

## Building

### Standalone Build (Recommended for Testing)
```bash
# Install dependencies
sudo apt-get install libeigen3-dev libboost-all-dev libopencv-dev

# Create build directory
mkdir build && cd build

# Configure and build
cmake /path/to/T-ESKF/ov_msckf/src/robot_sim -DSTANDALONE_BUILD=ON
make
```

### Integration with T-ESKF
The robot simulation is automatically built with the main T-ESKF project:
```bash
cd ~/catkin_ws
catkin build ov_msckf
```

## Usage

### Basic Testing
Test all components work correctly:
```bash
./test_robot_simulation
```

### Running Simulations

#### PID Controller
```bash
# Circle trajectory (trajectory 0)
./robot_simulation pid 0

# Figure-8 trajectory (trajectory 1)  
./robot_simulation pid 1

# Line trajectory (trajectory 2)
./robot_simulation pid 2

# Waypoint trajectory (trajectory 3)
./robot_simulation pid 3
```

#### RL Controller
```bash
# Run with existing or train new policy
./robot_simulation rl 0

# Force training of new policy
./robot_simulation train 0

# Run both controllers for comparison
./robot_simulation all 0
```

### Trajectory Types
- **0**: Circle (radius=1.5m, duration=20s)
- **1**: Figure-8 (scale=2.0, duration=25s)  
- **2**: Line (start=(0,0), end=(5,3), velocity=1.0m/s)
- **3**: Waypoint (5 waypoints over 20s)

## Configuration

### Robot Parameters
```cpp
DualWheelRobot::RobotParams params;
params.wheelbase = 0.3;           // Distance between wheels (m)
params.max_linear_vel = 2.0;      // Maximum linear velocity (m/s)
params.max_angular_vel = 3.14;    // Maximum angular velocity (rad/s)  
params.max_linear_accel = 1.0;    // Maximum linear acceleration (m/s²)
params.max_angular_accel = 3.14;  // Maximum angular acceleration (rad/s²)
```

### PID Tuning
```cpp
PIDController::PIDParams pid_params;
// Position control
pid_params.kp_x = 2.0; pid_params.ki_x = 0.1; pid_params.kd_x = 0.5;
pid_params.kp_y = 2.0; pid_params.ki_y = 0.1; pid_params.kd_y = 0.5;
// Orientation control  
pid_params.kp_theta = 3.0; pid_params.ki_theta = 0.2; pid_params.kd_theta = 0.8;
// Velocity control
pid_params.kp_v = 1.5; pid_params.ki_v = 0.05; pid_params.kd_v = 0.3;
pid_params.kp_omega = 2.0; pid_params.ki_omega = 0.1; pid_params.kd_omega = 0.4;
```

### RL Training
```cpp
RLController::RLParams rl_params;
rl_params.population_size = 50;        // Population size for evolution
rl_params.mutation_rate = 0.1;         // Probability of parameter mutation
rl_params.mutation_strength = 0.1;     // Standard deviation of mutations
rl_params.max_generations = 100;       // Training episodes
rl_params.learning_rate = 0.01;        // Parameter update rate
```

## Output Files

### Simulation Results
- `simulation_results_PID.csv`: PID controller performance data
- `simulation_results_RL.csv`: RL controller performance data  
- `rl_policy.txt`: Trained RL policy parameters

### Data Format
Each CSV contains timestamped data:
- Robot state: x, y, θ, v, ω
- Desired state: x_d, y_d, θ_d, v_d, ω_d  
- Control inputs: linear_accel, angular_accel

## Visualization

Generate plots from simulation results:
```bash
python3 scripts/visualize_results.py simulation_results_PID.csv --title "PID Controller" --save
```

This creates:
- Trajectory comparison plot
- Control signal plots
- Velocity tracking plots
- Performance statistics

## Performance Examples

### PID Controller (Circle Trajectory)
- Average position error: ~5.3m (needs tuning)
- Average orientation error: ~0.24rad
- Simulation time: ~3ms for 20s trajectory

### RL Controller
- Training: 50-100 episodes typically sufficient
- Policy size: ~1000 parameters
- Inference: Real-time capable

## Implementation Details

### Robot Dynamics
The differential drive robot follows standard kinematics:
```
ẋ = v cos(θ)
ẏ = v sin(θ)  
θ̇ = ω
v̇ = u_linear
ω̇ = u_angular
```

### Control Architecture
1. **Trajectory Generator**: Produces time-indexed reference states
2. **Controller**: Computes control inputs from state error
3. **Robot Model**: Integrates dynamics with control inputs
4. **Logger**: Records all signals for analysis

### RL Training Process
1. **Population Initialization**: Random neural network parameters
2. **Evaluation**: Each individual runs full trajectory
3. **Selection**: Best performers become parents
4. **Mutation**: Add Gaussian noise to parameters
5. **Iteration**: Repeat until convergence

## Extending the Framework

### Adding New Trajectories
Inherit from `TrajectoryBase` and implement:
```cpp
class CustomTrajectory : public TrajectoryBase {
  TrajectoryPoint getDesiredState(double time) override;
  bool isFinished(double time) override;
  double getDuration() override;
  void reset() override;
};
```

### Adding New Controllers  
Implement the control interface:
```cpp
class CustomController {
  DualWheelRobot::ControlInput computeControl(
    const DualWheelRobot::RobotState& current_state,
    const TrajectoryPoint& desired_state
  );
  void reset();
};
```

## Troubleshooting

### Common Issues
1. **Build Errors**: Ensure Eigen3, OpenCV, and Boost are installed
2. **Poor Tracking**: Tune PID gains or increase RL training episodes  
3. **Unstable Control**: Check control limits and reduce gains
4. **Slow Convergence**: Increase RL population size or mutation rate

### Debug Output
Enable verbose output by modifying the simulation loop to print intermediate values.

## References
- Differential Drive Kinematics: Siegwart & Nourbakhsh, "Introduction to Autonomous Mobile Robots"
- PID Control: Åström & Hägglund, "PID Controllers: Theory, Design, and Tuning"  
- Evolutionary Strategies: Hansen & Ostermeier, "Completely Derandomized Self-Adaptation in Evolution Strategies"