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

The launch file does not start Rviz automatically. To visualize the robot model:

1.  Open a new terminal.
2.  Source the workspace:
    ```bash
    cd ~/Documents/robot_sim/ros_ws
    source install/setup.bash
    ```
3.  Run Rviz 2:
    ```bash
    rviz2
    ```

### Configuring Rviz

1.  In the **Displays** panel (left side), click **Add**.
2.  Select **RobotModel** and click **OK**.
3.  Set the **Fixed Frame** (in Global Options) to `base_link` (or the root link of your URDF).
4.  You should now see the 3D model of the rover.

## Interaction

The robot exposes the following topics for control:

-   **`/cmd_vel`** (`geometry_msgs/msg/Twist`): Controls the base movement (linear/angular velocity).
-   **`/joint_commands`** (`sensor_msgs/msg/JointState`): Controls the arm joints. Publish a message with `name` and `position` arrays to move the arm.

The robot publishes its state to:

-   **`/joint_states`** (`sensor_msgs/msg/JointState`): Current joint positions (feedback from hardware or mock simulation).
