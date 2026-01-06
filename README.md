# 🚲 Autonomous Bicycle Parking

<div align="center">
  
**A comprehensive autonomous parking system using bicycle kinematic model with multiple control strategies**

*Field and Service Robotics - Final Project*

*Università degli Studi di Napoli Federico II*

</div>

---

## 📋 Table of Contents

- [Overview](#-overview)
- [Features](#-features)
- [System Architecture](#-system-architecture)
- [Controllers Implemented](#-controllers-implemented)
- [Project Structure](#-project-structure)
- [Installation & Requirements](#-installation--requirements)
- [Usage](#-usage)
- [Performance Comparison](#-performance-comparison)
- [Results](#-results)
- [References](#-references)

---

## 🎯 Overview

This project develops, implements, and critically analyzes a complete **autonomous parking system** based on the bicycle kinematic model. The system demonstrates the ability to:

- Generate admissible trajectories from initial pose to parking position
- Navigate through obstacle-free space in a parking lot environment
- Track trajectories with minimum error using multiple control approaches
- Achieve precision parking with centimeter-level accuracy

The bicycle model provides the optimal compromise between computational simplicity and accuracy for vehicles operating at moderate speeds, typical of parking maneuvers.

---

## ✨ Features

- **RRT\* Path Planning**: Optimal path generation with obstacle avoidance
- **Multiple Control Strategies**: Four different controllers for comparative analysis
- **Adaptive Control**: Dynamic parameter adjustment based on operational context
- **Costmap Generation**: Vehicle-aware environment representation
- **Velocity Profiling**: Curvature-adaptive speed management
- **Real-time Visualization**: Animated parking simulations

---

## 🏗 System Architecture

### Vehicle Parameters

| Parameter | Value | Description |
|-----------|-------|-------------|
| Length | 5.0 m | Total vehicle length |
| Width | 2.5 m | Vehicle width |
| Wheelbase | 2.8 m | Distance between axles |
| Rear Overhang | 1.0 m | Rear overhang distance |
| Cell Size | 0.1 m | Map resolution |
| Inflation Radius | 0.5 m | Safety buffer around obstacles |

### RRT* Planner Configuration

| Parameter | Value |
|-----------|-------|
| MinTurningRadius | 4.0 m |
| ConnectionDistance | 30.0 m |
| GoalTolerance | [0.5 m, 0.5 m, 10°] |
| MinIterations | 2000 |

### Velocity Constraints

- **Speed Range**: 0.0 - 2.0 m/s
- **Max Lateral Acceleration**: 3.0 m/s²
- **Steering Limit**: ±45°

---

## 🎮 Controllers Implemented

### 1. Pure Pursuit Controller
A geometric path tracking controller using adaptive look-ahead distance.

**Key Features:**
- Adaptive look-ahead (2.0 - 15.0 m based on speed)
- Multi-phase control logic (startup, cruise, approach, final)
- Variable smoothing coefficients
- Achieves **8 cm position accuracy** and **0.02 rad orientation accuracy**

### 2. Model Predictive Control (MPC)
An optimization-based controller using quadratic programming.

**Key Features:**
- 6-step prediction horizon (0.6s anticipation)
- Error coordinate transformation
- Adaptive weight matrices for different phases
- Constraint handling for velocity and steering

**Weight Configuration:**
- Q = diag(2000, 200, 400) - State error weights
- R = diag(0.5, 2) - Control input weights

### 3. Hybrid Multi-Mode Controller
Combines multiple control strategies for robust performance.

**Components:**
- Feedforward-feedback longitudinal control
- Stanley-type lateral control
- Heading control for orientation correction
- Supervisory logic for phase management

**Operating Modes:**
- Normal navigation
- Approaching target (position_error > 0.5 m)
- Parking complete (position_error < 0.3 m)

### 4. Stanley Controller
Front-axle referenced path following with dynamic model.

**Control Law:**
```
δ = ψ + arctan(k · e_lateral / (v + v_offset))
```

**Features:**
- Differentiated gains for forward (2.0) and reverse (2.5) motion
- PI longitudinal control with anti-windup
- Dynamic bicycle model integration

---

## 📁 Project Structure

```
FINAL_PROJECT_FSR/
│
├── PURE_PURSUIT/
│   ├── pure_pursuit_controller.m
│   └── BICYCLE_PARKING_ANIMATION_PURE_PURSUIT.mp4
│
├── MPC_CONTROLLER/
│   ├── mpc_controller.m
│   └── BICYCLE_PARKING_ANIMATION_MPC_CONTROLLER.mp4
│
├── Hybrid_Multi_Mode_Controller/
│   ├── hybrid_controller.m
│   └── BICYCLE_PARKING_ANIMATION_HYBRID_CONTROLLER.mp4
│
├── STANLEY/
│   ├── stanley_controller.m
│   └── BICYCLE_PARKING_ANIMATION_STANLEY_CONTROLLER.mp4
│
├── common/
│   ├── my_map.bmp
│   ├── costmap_generator.m
│   └── rrt_star_planner.m
│
├── Final_Project_FSR.pdf
├── Presentation.pptx
└── README.md
```

---

## 💻 Installation & Requirements

### Prerequisites

- **MATLAB R2024a** or later
- **Simulink**
- **Robotics System Toolbox**
- **Automated Driving Toolbox** (for Stanley controller)

### Setup

1. Clone the repository:
```bash
git clone https://github.com/yourusername/autonomous-bicycle-parking.git
cd autonomous-bicycle-parking
```

2. Open MATLAB and navigate to the project directory

3. Run the desired controller simulation:
```matlab
run('PURE_PURSUIT/pure_pursuit_controller.m')
```

---

## 🚀 Usage

### Running a Simulation

1. **Generate the costmap** (if not already generated):
```matlab
costmap = generateCostmap('my_map.bmp', vehicleParams);
```

2. **Plan the path using RRT\***:
```matlab
[path, trajectory] = planPath(startPose, goalPose, costmap);
```

3. **Run the controller**:
```matlab
% Choose one of the controllers
runPurePursuitController(trajectory);
% or
runMPCController(trajectory);
% or
runHybridController(trajectory);
% or
runStanleyController(trajectory);
```

### Customizing Parameters

Edit the configuration section in each controller file to adjust:
- Velocity limits
- Control gains
- Look-ahead distances
- Prediction horizons

---

## 📊 Performance Comparison

| Metric | Pure Pursuit | MPC | Hybrid | Stanley |
|--------|-------------|-----|--------|---------|
| Max Position Error (m) | 1.15 | 3.97 | **0.50** | 4.40 |
| RMS Position Error (m) | 0.26 | 2.17 | **0.18** | 2.93 |
| Mean Position Error (m) | 0.17 | 2.06 | **0.14** | 2.90 |
| Final Position Error (m) | **0.02** | 2.16 | 0.23 | 2.20 |
| Max Heading Error (°) | 0.58 | 0.83 | **0.14** | 4.34 |
| Final Heading Error (°) | **0.01** | 0.03 | 0.02 | 1.51 |
| Control Saturation (%) | 3.10 | **0.00** | 1.60 | **0.00** |

### Key Insights

- **Best Overall Tracking**: Hybrid Controller
- **Smoothest Control**: MPC and Stanley (0% saturation)
- **Best Final Positioning**: Pure Pursuit
- **Best Trade-off**: Hybrid Controller

---

## 📈 Results

### Trajectory Tracking Performance

All controllers successfully navigate the parking environment with obstacles. The animations demonstrate:

- Smooth navigation through narrow corridors
- Effective obstacle avoidance
- Precise final parking alignment

### Controller Characteristics

| Controller | Strengths | Weaknesses |
|------------|-----------|------------|
| **Pure Pursuit** | Excellent final positioning, intuitive | Higher control saturation |
| **MPC** | Smooth control, constraint handling | Larger transient errors |
| **Hybrid** | Best overall accuracy, adaptive | Moderate complexity |
| **Stanley** | Zero saturation, robust | Larger heading errors |

---

## 📚 References

1. Aggor, J. (2023). *Exploring path planning with RRT\* and visualization in Python*. Medium.

2. MathWorks. (2025). *Bicycle Kinematic Model - Robotics Toolbox*. MATLAB Documentation.

3. Ding, Y. (2020). *Three methods of vehicle lateral control: Pure pursuit, Stanley and MPC*. Medium.

4. Hoffmann, G. M., & Tomlin, C. J. (2007). *Autonomous automobile trajectory tracking for off-road driving: Controller design, experimental validation and racing*. American Control Conference.

---

## 📄 License

This project is developed for academic purposes as part of the Field and Service Robotics course at Università degli Studi di Napoli Federico II.

---

<div align="center">


</div>
