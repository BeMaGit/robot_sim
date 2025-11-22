# RC Rover ROS 2 Workspace

This workspace contains the ROS 2 packages for the RC Rover project.

## Prerequisites

- ROS 2 (Humble, Iron, or Rolling recommended)
- `colcon` build tool
- Python 3

## Build Instructions

1.  Source the main ROS 2 installation:
    ```bash
    source /opt/ros/jazzy/setup.bash
    ```

2.  Navigate to the workspace directory:
    ```bash
    cd ~/Documents/robot_sim/ros_ws
    ```

3.  Build the packages:
    ```bash
    colcon build --symlink-install
    ```

3.  Source the setup script:
    ```bash
    source install/setup.bash
    ```

## Running the Robot

To launch the robot's hardware interface and state publisher, run:

```bash
ros2 launch rc_rover_pkg bringup.launch.py serial_port:=/dev/ttyUSB0
```

-   **Note**: If the Arduino is not connected to the specified `serial_port`, the node will automatically fall back to **MOCK mode**, allowing you to test the system without hardware.

## Visualization (Rviz)

The launch file starts Rviz automatically with a pre-configured view.

To visualize the robot model, simply run the launch command mentioned above. RViz will open and display the robot model.

## Interaction

The robot exposes the following topics for control:

-   **`/cmd_vel`** (`geometry_msgs/msg/Twist`): Controls the base movement (linear/angular velocity).
-   **`/joint_commands`** (`sensor_msgs/msg/JointState`): Controls the arm joints. Publish a message with `name` and `position` arrays to move the arm.

The robot publishes its state to:

-   **`/joint_states`** (`sensor_msgs/msg/JointState`): Current joint positions (feedback from hardware or mock simulation).
