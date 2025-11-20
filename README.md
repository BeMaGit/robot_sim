# 3D Robot Simulation

A PyBullet-based simulation of an RC Rover equipped with a 6-DOF robot arm. This project simulates a differential drive mobile base and a manipulator arm, controllable via keyboard inputs.

## Features

- **Mobile Base**: Differential drive simulation with 4 wheels (2 driven, 2 passive).
- **Robot Arm**: 6-DOF arm + gripper (7 joints total) simulation.
- **Interactive Control**: Real-time keyboard control for both the rover and the arm.
- **Physics Engine**: Powered by [PyBullet](https://pybullet.org/) for realistic physics and collision detection.

## Requirements

- Python 3.x
- `pybullet`

## Installation

1.  Clone the repository (if applicable) or download the source code.
2.  Install the required dependencies:

    ```bash
    pip install -r requirements.txt
    ```

![Robot Simulation](img/simulation_screenshot.png)

## Usage

To start the simulation, run the `simulation.py` script:

```bash
python simulation.py
```

To run the automated demo script:

```bash
python demo.py
```

To run the robot control script (for Raspberry Pi):

```bash
python robot_control.py
```

*Note: `robot_control.py` contains placeholder functions for hardware interfaces. You will need to adapt `read_rc_input` and `set_servo_angle` to match your specific motor driver and RC receiver libraries.*

A PyBullet GUI window will open showing the robot on a ground plane.

## Controls

Click on the PyBullet window to ensure it has focus before using the keyboard controls.

### Rover Movement
- **Arrow Up**: Move Forward
- **Arrow Down**: Move Backward
- **Arrow Left**: Turn Left
- **Arrow Right**: Turn Right

### Arm Control (Joint Mode)
Toggle Mode: **T**

You can control the arm joints using the **Side Panel Sliders** (Params) or the keyboard.

| Joint | Increase | Decrease |
|-------|----------|----------|
| **Waist** | J | L |
| **Shoulder** | 3 | 4 |
| **Elbow** | 5 | 6 |
| **Forearm Roll** | 7 | 8 |
| **Wrist Pitch** | 9 | 0 |
| **Wrist Roll** | O | U |
| **Gripper** | [ | ] |

*Note: The Side Panel Sliders allow for direct control of each joint. If you use the keyboard or IK mode, the sliders will not update to match the robot's position due to PyBullet limitations.*

### Arm Control (IK Mode)
Toggle Mode: **T**

-   **I / K**: Move Forward / Backward (X-axis)
-   **J / L**: Move Left / Right (Y-axis)
-   **U / O**: Move Up / Down (Z-axis)
-   **N / M**: Rotate Yaw
-   **Space**: Toggle Gripper

## File Structure

- `simulation.py`: Main entry point. Sets up the PyBullet environment, loads the robot, and runs the simulation loop.
- `controller.py`: Contains the `RobotController` class which handles the logic for wheel velocities and arm joint angles.
- `robot_model.urdf`: The URDF (Unified Robot Description Format) file defining the robot's physical structure, joints, and visual/collision meshes.
- `test_simulation.py`: Unit tests for the simulation setup.
- `test_controller.py`: Unit tests for the controller logic.

## Testing

To run the tests:

```bash
python -m unittest test_controller.py
python -m unittest test_simulation.py
```
