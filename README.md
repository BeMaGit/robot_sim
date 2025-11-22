# RC Rover Robot Simulation

This project contains two distinct implementations for simulating and controlling an RC Rover with a 6-DOF robot arm.

## Project Structure

### 1. ROS 2 Workspace (`ros_ws/`)
A full ROS 2 (Robot Operating System) implementation. This is the recommended approach for robust robot control, visualization, and hardware interfacing.

-   **Features**:
    -   URDF model with visual and collision geometry.
    -   `robot_state_publisher` for TF broadcasting.
    -   RViz 2 visualization (pre-configured).
    -   Hardware interface node for Arduino communication.
    -   Launch files for easy startup.
-   **Location**: [`ros_ws/`](ros_ws/)
-   **Instructions**: See [`ros_ws/README.md`](ros_ws/README.md)

### 2. Python Simulation (`robsim/`)
A standalone Python simulation using PyBullet. This is useful for testing kinematics, physics, and control logic without the overhead of ROS.

-   **Features**:
    -   PyBullet physics engine.
    -   Inverse Kinematics (IK) implementation.
    -   Keyboard control for base and arm.
    -   Direct Arduino interfacing scripts.
-   **Location**: [`robsim/`](robsim/)
-   **Instructions**: See [`robsim/README.md`](robsim/README.md)

## Hardware
Both implementations target the same hardware platform:
-   **Chassis**: 4WD RC Rover (Skid Steer).
-   **Arm**: 6-DOF Robot Arm + Gripper.
-   **Electronics**: Raspberry Pi (High-level control) + Arduino (Low-level servo driver).
