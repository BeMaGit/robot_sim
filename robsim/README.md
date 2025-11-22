# 3D Robot Simulation

A PyBullet-based simulation of an RC Rover equipped with a 6-DOF robot arm. This project simulates a differential drive mobile base and a manipulator arm, controllable via keyboard inputs.

The hardware is based on the [RC Rover with Robot Arm 6DOF](https://makerworld.com/en/models/1342319-rc-rover-with-robot-arm-6-dof#profileId-1383072) from [MakerWorld](https://makerworld.com/) by [Emre Kalem](https://makerworld.com/en/@emrekalem).

* [Documentation](https://www.youtube.com/watch?v=IE_Y0Bq97aE&t=24s)

** ATTENTION - THE CODE IS NOT TESTED ON HARDWARE YET! **

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
2.  Create and activate a virtual environment:

    ```bash
    python3 -m venv venv
    source venv/bin/activate  # On Windows use: venv\Scripts\activate
    ```

3.  Install the required dependencies:

    ```bash
    venv/bin/pip install -r requirements.txt
    ```

![Robot Simulation](../img/robot_sim_v00.png)

## Usage

To start the simulation, run the `simulation.py` script:

```bash
venv/bin/python simulation.py
```

To run the automated demo script:

```bash
venv/bin/python demo.py
```

To run the robot control script (for Raspberry Pi):

```bash
venv/bin/python robot_control.py
```

*Note: `robot_control.py` contains placeholder functions for hardware interfaces. You will need to adapt `read_rc_input` and `set_servo_angle` to match your specific motor driver and RC receiver libraries.*

## Hardware Control Schema

The robot control logic is split between the Raspberry Pi (High-level logic, IK) and an Arduino (Low-level servo driver).

![Control Schema](img/robot_control_schema.svg)

### Setup Instructions

1.  **Arduino**:
    *   Open `arduino_driver/arduino_driver.ino` in the Arduino IDE.
    *   Connect your Arduino and upload the sketch.
    *   Note the serial port (e.g., `/dev/ttyUSB0` or `/dev/ttyACM0`).

2.  **Raspberry Pi**:
    *   Ensure the Arduino is connected via USB.
    *   Run the control script:
        ```bash
        python robot_control.py
        ```
    *   The script will attempt to connect to `/dev/ttyUSB0` by default. You can modify the port in `robot_control.py` if needed.

A PyBullet GUI window will open showing the robot on a ground plane.

## Controls

Click on the PyBullet window to ensure it has focus before using the keyboard controls.

### Rover Movement

- **Arrow Up**: Move Forward
- **Arrow Down**: Move Backward
- **Arrow Left**: Turn Left
- **Arrow Right**: Turn Right

### Arm Movement

- **I / K**: Arm Forward / Backward (X-axis)
- **Z / H**: Arm Up / Down (Z-axis)
- **J / L**: Rotate Arm
- **U / O**: Rotate Gripper
- **N / M***: Gripper Open / Close

## File Structure

- `simulation.py`: Main entry point. Sets up the PyBullet environment, loads the robot, and runs the simulation loop.
- `controller.py`: Contains the `RobotController` class which handles the logic for wheel velocities and arm joint angles.
- `robot_model.urdf`: The URDF (Unified Robot Description Format) file defining the robot's physical structure, joints, and visual/collision meshes.
- `test_simulation.py`: Unit tests for the simulation setup.
- `test_controller.py`: Unit tests for the controller logic.

## Testing

To run the tests:

```bash
venv/bin/python -m unittest test_controller.py
venv/bin/python -m unittest test_simulation.py
```
