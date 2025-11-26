# Dual UR5e Configuration - ROS 2 Robotics Simulation

A comprehensive ROS 2 framework for simulating and controlling dual UR5e robotic arms with advanced motion planning, collision avoidance, and manipulation capabilities.<cite/>

## 🚀 Features

- **Dual Robot Coordination**: Simulate and control two UR5e arms working in shared workspaces
- **Advanced Motion Planning**: Integrated MoveIt framework with collision-free trajectory generation
- **Physics Simulation**: Ignition Gazebo (Gazebo Sim) for realistic physics and sensor simulation
- **Collision Avoidance**: Real-time inter-robot collision detection and avoidance algorithms
- **Manipulation Capabilities**: Robotiq 2f-85 gripper integration with pick-and-place operations
- **BIM Integration**: Building Information Modeling support for construction environments
- **Visualization**: RViz2 integration for 3D robot state visualization and planning

## 📁 Repository Structure

```
Dual_ur5e_configuration/
├── collision_avoidance_packages/     # Multi-robot safety systems
│   ├── add_collision_objects/        # Dynamic obstacle management
│   ├── bim_object_sequencer/         # BIM IFC file processing
│   └── robot_collision_avoidance/    # Inter-robot collision detection
├── dual_ur5e_simulation/             # Dual arm coordination
│   ├── dual_robot_combined_description/
│   └── dual_robot_combined_moveit_config/
└── single_ur5e_simulation/           # Single arm control
    ├── robot_controller/             # Joint control algorithms
    ├── robot_description/            # URDF/XACRO robot models
    ├── robot_moveit_config/          # Motion planning configuration
    ├── robot_path_planning/          # Manipulation algorithms
    └── robot_simulation_launch/      # Simulation orchestration
```

## 🛠️ Installation

### Prerequisites

- Ubuntu 22.04 LTS
- ROS 2 Humble
- Python 3.10+
- CMake 3.22+

### Dependencies

The project requires extensive ROS 2 packages including MoveIt, Ignition Gazebo, and specialized libraries:<cite/>

```bash
# Core ROS 2 and MoveIt
sudo apt install ros-humble-moveit ros-humble-moveit-ros-planning-interface
sudo apt install ros-humble-ros-gz-sim ros-humble-ros-gz-bridge

# BIM and Python dependencies
sudo apt install python3-ifcopenshell python3-numpy

# Visualization and tools
sudo apt install ros-humble-rviz2 ros-humble-warehouse-ros-mongo
```

### Build Instructions

```bash
# Clone the repository
git clone https://github.com/sumit280188/Dual_ur5e_configuration.git
cd Dual_ur5e_configuration

# Install dependencies
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# Build the workspace
colcon build --symlink-install

# Source the workspace
source install/setup.bash
```

## 🎮 Usage

### Single Robot Simulation

Launch a single UR5e arm with gripper in Gazebo:

```bash
ros2 launch robot_simulation_launch robot_simulation.launch.py
```

### Dual Robot Simulation

Launch coordinated dual UR5e arms with collision avoidance:

```bash
ros2 launch dual_robot_combined_description dual_robot_combined_simulation.launch.py
```

### Pick and Place Operations

Run pick-and-place demonstrations:

```bash
# Basic pick and place
ros2 launch robot_path_planning pick_and_place.launch.py

# With trajectory publishing
ros2 launch robot_path_planning pick_and_place_with_publisher.launch.py

# With gripper feedback control
ros2 launch robot_path_planning robot_with_gripper_feedback.launch.py
```

### Motion Planning Interface

Start MoveIt with RViz visualization:

```bash
# Single robot
ros2 launch robot_moveit_config demo.launch.py

# Dual robot
ros2 launch dual_robot_combined_moveit_config demo.launch.py
```

### Adding Objects to Simulation

Spawn objects for grasping using Ignition services:<cite/>

```bash
ign service -s /world/default/create \
  --reqtype ignition.msgs.EntityFactory \
  --reptype ignition.msgs.Boolean \
  --req 'sdf_filename: "path/to/object.sdf", name: "grasp_box", pose: {position: {x: 0.34, y: 0.13, z: 0.1}}'
```

## 🐳 Docker Support

A comprehensive Dockerfile is included for containerized deployment:

```bash
# Build the Docker image
docker build -t dual-ur5e-simulation .

# Run with X11 forwarding for GUI applications
docker run -it --rm \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  dual-ur5e-simulation
```

## 🔄 CI/CD Pipeline

The project includes GitHub Actions workflows for:

- **Build and Test**: Automated compilation and testing across ROS 2 packages
- **Code Quality**: Ament linting and style checking
- **Docker Build**: Automated container building and pushing
- **Simulation Tests**: Headless Gazebo and MoveIt validation

## 📊 Key Components

### Robot Configuration

The UR5e robots are configured with Robotiq 2f-85 grippers and use Ignition Gazebo for simulation:<cite/>

```xml
<!-- sim_ignition set to true for Ignition Gazebo -->
<xacro:arg name="sim_ignition" default="true" />
```

### Motion Planning Groups

- **Single Robot**: `ur_manipulator` (arm) and `gripper` groups<cite/>
- **Dual Robot**: `alice_arm`, `bob_arm`, `alice_gripper`, `bob_gripper` groups<cite/>

### Control Interface

ROS 2 Control manages joint trajectories through action servers:<cite/>

```bash
# Robot arm control
ros2 action send_goal /robot_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory "{...}"

# Gripper control  
ros2 action send_goal /robotiq_gripper_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory "{...}"
```

## 🧪 Testing

Run the test suite:

```bash
# Build with tests
colcon build --symlink-install

# Run all tests
colcon test --return-code-on-test-failure

# View test results
colcon test-result --verbose
```

## 🤝 Contributing

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/amazing-feature`)
3. Follow ROS 2 coding standards and ament linting
4. Commit your changes (`git commit -m 'Add amazing feature'`)
5. Push to the branch (`git push origin feature/amazing-feature`)
6. Open a Pull Request

## 📄 License

This project is licensed under the BSD License - see the package.xml files for details.<cite/>

## 🙏 Acknowledgments

- **Universal Robots**: For the UR5e robot description and drivers
- **Robotiq**: For the 2f-85 gripper models and integration
- **MoveIt Team**: For the motion planning framework
- **ROS 2 Community**: For the robust robotics middleware

## 📞 Support

For questions and support:
- Open an issue on GitHub
- Check the ROS 2 documentation
- Review the MoveIt tutorials

---

**Note**: This project requires ROS 2 Humble and is tested on Ubuntu 22.04 LTS. For other platforms, please refer to the ROS 2 installation guides.

## Notes

The repository uses Ignition Gazebo (Gazebo Sim) rather than classic Gazebo, which is configured through the `sim_ignition` parameter in the URDF files.<cite/> The BIM object sequencer requires `python3-ifcopenshell` for processing IFC building files.<cite/> All launch files support headless operation for CI/CD and automated testing scenarios.
