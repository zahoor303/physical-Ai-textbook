You are helping me create a SPECIFICATION for Module 2: "The Digital Twin (Gazebo & Unity)" of the Physical AI & Humanoid Robotics textbook.

## Context:
This specification is ONLY for Module 2. Module 1 (ROS 2) is already complete. Module 2 focuses on physics simulation and environment building for robots.

## Module 2 Scope:
Students learn to create digital twins of robots and environments using Gazebo and Unity simulators, enabling safe testing before deploying to physical hardware.

### Topics to Cover:
1. Physics simulation (gravity, collisions, friction) in Gazebo
2. URDF robot models in simulation
3. Gazebo world building and environment design
4. Simulating sensors: LiDAR, Depth Cameras, IMUs
5. High-fidelity rendering and visualization in Unity
6. Human-robot interaction in simulated environments
7. Bridging ROS 2 with Gazebo (ros2_control, gazebo_ros)
8. Sim-to-real transfer concepts

## Create DETAILED SPECIFICATION for Module 2:

### 1. Module Overview
- Module title and tagline
- Learning objectives (5-7 specific outcomes)
- Prerequisites (Module 1 completion, basic 3D concepts)
- Estimated completion time
- Target audience level

### 2. Chapter Structure
Define how many chapters (recommend 5-7) and what each covers:

**Suggested chapters:**
- Chapter 1: Introduction to Robot Simulation
- Chapter 2: Gazebo Fundamentals
- Chapter 3: URDF Models in Gazebo
- Chapter 4: Simulating Sensors (LiDAR, Cameras, IMUs)
- Chapter 5: Physics and World Building
- Chapter 6: Unity Integration for Visualization (optional)
- Chapter 7: Sim-to-Real Transfer (optional)

For each chapter specify:
- Main concepts to teach
- Estimated word count (1000-1500 words)
- Number of code examples (2-3)
- Diagrams/visuals required (screenshots, architecture diagrams)
- Key simulations to demonstrate

### 3. Chapter Template
Every chapter follows the same template as Module 1:
- Learning Objectives (3-5 points)
- Introduction (200-300 words)
- Core Concepts (500-800 words)
- Practical Examples (2-3 working simulations/code)
- Hands-on Exercise (1-2 tasks)
- Common Pitfalls (2-3 issues)
- Summary (100-150 words)
- Further Reading (3-5 resources)

### 4. Technical Content Standards
- Gazebo version: Gazebo Classic 11 or Gazebo Sim (Ignition)
- ROS 2 version: Humble (consistent with Module 1)
- Unity version: 2022 LTS (if covering Unity)
- URDF/SDF specifications
- File formats: .world, .sdf, .urdf, .xacro
- Launch file examples

### 5. Simulation Examples
Specify types of simulations to include:
- Basic empty world with single robot
- Robot with sensors (camera, lidar, IMU)
- Physics demonstrations (gravity, collisions, friction)
- Sensor data visualization
- Multi-robot scenarios (if time permits)

### 6. Visual Content
- Screenshots of Gazebo GUI
- World file visualizations
- Sensor output examples (point clouds, camera feeds)
- Architecture diagrams (ROS 2 ↔ Gazebo bridge)
- Unity screenshots (if applicable)

### 7. File Organization
```
docs/module-2/
├── _category_.json
├── 01-introduction-to-simulation.md
├── 02-gazebo-fundamentals.md
├── 03-urdf-in-gazebo.md
├── 04-simulating-sensors.md
├── 05-physics-and-worlds.md
├── 06-unity-integration.md (optional)
└── 07-sim-to-real.md (optional)
```

### 8. Code Examples Format
- Gazebo launch files (.launch.py for ROS 2)
- URDF/SDF robot descriptions
- World files (.world or .sdf)
- ROS 2 nodes for sensor processing
- Configuration files

### 9. Exercises
Examples of hands-on exercises:
- Load robot in Gazebo and verify sensors
- Modify world file to add obstacles
- Create custom URDF with sensors
- Visualize LiDAR data in RViz2
- Test collision detection

### 10. Prerequisites Knowledge
What students should know before starting Module 2:
- Module 1 complete (ROS 2 basics, nodes, topics)
- URDF basics from Module 1 Chapter 6
- Basic understanding of coordinate systems
- Command-line proficiency

## Constraints:
- Keep Module 2 focused on SIMULATION (not physical robots)
- Unity content should be minimal/optional (Gazebo is priority)
- All examples must work with ROS 2 Humble
- Beginner-friendly (students new to simulation)
- Practical focus (students should create working simulations)

## Quality Standards:
- Technical accuracy for Gazebo/simulation
- Clear explanations of physics concepts
- Working simulation examples
- Visual aids (screenshots essential for GUI-based tools)
- Consistent with Module 1 standards

Think step by step and create a comprehensive specification that defines exactly what Module 2 should contain.

Output as a well-organized markdown document for `specs/003-module-2-gazebo-unity/spec.md`

Keep it focused and under 3000 words (more concise than Module 1 spec since you can reference Module 1 standards).