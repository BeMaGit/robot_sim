# ROS 2 Implementation Walkthrough

This document guides you through building and running the ROS 2 package for your RC Rover.

## Prerequisites
- ROS 2 (Humble or Jazzy recommended) installed.
- `colcon` build tool.
- `pyserial` installed in your ROS environment.

## Build Instructions
1.  Navigate to the workspace root:
    ```bash
    cd ~/Documents/robot_sim/ros_ws
    ```
2.  Build the package:
    ```bash
    colcon build --symlink-install
    ```
3.  Source the overlay:
    ```bash
    source install/setup.bash
    ```

## Running the Hardware Interface
To launch the hardware interface and the robot state publisher:

```bash
ros2 launch rc_rover_pkg bringup.launch.py serial_port:=/dev/ttyUSB0
```

### Mock Mode
If no Arduino is connected, the node will automatically fall back to MOCK mode, logging commands to the console instead of sending them to the serial port.

## Controlling the Robot
You can control the robot by publishing to the `joint_commands` topic:

```bash
ros2 topic pub /joint_commands sensor_msgs/msg/JointState "{name: ['waist_joint', 'shoulder_joint'], position: [0.5, -0.5]}"
```

## Visualizing
To visualize the robot in Rviz2:
```bash
ros2 run rviz2 rviz2
```
Add a "RobotModel" display and set the "Fixed Frame" to `base_link`.
