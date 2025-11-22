import pybullet as p
import pybullet_data
import time
import math

import serial

# --- Hardware Interface ---

class ArduinoInterface:
    def __init__(self, port='/dev/ttyUSB0', baudrate=115200):
        self.ser = None
        try:
            self.ser = serial.Serial(port, baudrate, timeout=1)
            time.sleep(2) # Wait for Arduino reset
            print(f"Connected to Arduino on {port}")
        except Exception as e:
            print(f"Error connecting to Arduino: {e}")
            print("Running in MOCK mode (no hardware output)")

    def send_command(self, angles):
        """
        Send angles to Arduino.
        angles: list or tuple of 7 floats (degrees)
        """
        if self.ser and self.ser.is_open:
            # Format: CMD:ang0,ang1,ang2,ang3,ang4,ang5,ang6\n
            cmd_str = "CMD:" + ",".join([f"{a:.2f}" for a in angles]) + "\n"
            try:
                self.ser.write(cmd_str.encode('utf-8'))
            except Exception as e:
                print(f"Serial write error: {e}")
        else:
            # Mock output
            # print(f"MOCK SEND: {angles}")
            pass

    def close(self):
        if self.ser and self.ser.is_open:
            self.ser.close()

# Global interface instance
arduino = None

def setup_hardware():
    """Initialize motor drivers and RC receiver."""
    global arduino
    # Try to connect to Arduino. Adjust port as needed (e.g., /dev/ttyACM0)
    arduino = ArduinoInterface(port='/dev/ttyUSB0')

def read_rc_input():
    """
    Read inputs from the Radio Controller.
    Returns a dictionary of normalized values (-1.0 to 1.0) for each channel.
    """
    # MOCK: Simulating some input. 
    # In a real scenario, read from your RC receiver (SBUS, PPM, etc.)
    # Example mapping:
    # Ch1: Left/Right (Yaw)
    # Ch2: Forward/Backward (X/Y)
    # Ch3: Up/Down (Z)
    # Ch4: Rotate (Wrist)
    # Ch5: Gripper
    
    # For testing without hardware, we return 0s (no movement)
    # Change these values to test IK response
    return {
        'ch1': 0.0, # Yaw
        'ch2': 0.0, # Fwd/Back
        'ch3': 0.0, # Up/Down
        'ch4': 0.0, # Wrist
        'ch5': 0.0  # Gripper
    }

def send_all_servos(arm_angles, gripper_val):
    """
    Send all servo angles to Arduino in one go.
    arm_angles: list of angles for joints 4-9 (6 items) in RADIANS
    gripper_val: 0.0 to 0.02 (approx)
    """
    if arduino:
        # Convert radians to degrees for Arduino
        # URDF Joints: Waist, Shoulder, Elbow, WristP, WristR
        # Note: Check your URDF zero positions vs Servo zero positions!
        # For now, assuming 1:1 mapping with some offset (e.g. 90 deg center)
        
        # Map -PI..PI to 0..180 (approx, needs tuning)
        # Let's assume 0 radians = 90 degrees
        
        degrees = []
        for rad in arm_angles:
            deg = math.degrees(rad) + 90.0
            degrees.append(deg)
            
        # Gripper: 0.0(Open) -> 0.02(Closed). Map to servo 0..180?
        # Let's say 0.0 -> 0 deg, 0.02 -> 180 deg (just an example)
        # Or maybe 0.0 -> 90, 0.02 -> 110
        grip_deg = 90.0 + (gripper_val / 0.02) * 90.0 
        degrees.append(grip_deg)
        
        arduino.send_command(degrees)

# --- Robot Control Logic ---

def main():
    # 1. Setup PyBullet for IK (No GUI needed for the Pi controller)
    # Use p.DIRECT for non-graphical version, or p.GUI to visualize on a monitor
    physicsClient = p.connect(p.DIRECT) 
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)

    # Load Robot
    # Ensure the URDF path is correct relative to this script
    robotId = p.loadURDF("robot_model.urdf", [0, 0, 0], useFixedBase=True)

    # Joint Indices (based on URDF)
    # 0-3: Wheels (Ignored for arm control, but could be used for base)
    # 4: Waist
    # 5: Shoulder
    # 6: Elbow
    # 7: Forearm Roll
    # 8: Wrist Pitch
    # 9: Wrist Roll
    # 10: Gripper
    
    # End Effector Link Index (The part we want to control)
    # Usually the last link in the chain. Let's check the URDF.
    # gripper_link is the child of gripper_joint.
    # We need the index of the link. p.getNumJoints() can help verify.
    # For now, let's assume the gripper link index matches the gripper joint index (10)
    ee_link_index = 10 

    # 2. Setup Hardware
    setup_hardware()

    # Initial Target State (Cartesian Position & Orientation)
    target_pos = [0.3, 0.0, 0.2] # X, Y, Z (meters)
    target_orn = p.getQuaternionFromEuler([0, 0, 0]) # Roll, Pitch, Yaw
    
    gripper_state = 0.0 # 0.0 (Open) to 0.02 (Closed)

    print("Robot Control Started. Press Ctrl+C to exit.")

    try:
        while True:
            # 3. Read Input
            rc_data = read_rc_input()
            
            # 4. Update Target State based on Input
            # Adjust sensitivity as needed
            speed = 0.005 
            
            # Map RC channels to Cartesian movement
            # Ch2 (Fwd/Back) -> X axis
            target_pos[0] += rc_data['ch2'] * speed
            
            # Ch1 (Left/Right) -> Y axis (or Yaw of base, depending on preference)
            # Let's map it to Y for strafing the end effector
            target_pos[1] += rc_data['ch1'] * speed
            
            # Ch3 (Up/Down) -> Z axis
            target_pos[2] += rc_data['ch3'] * speed
            
            # Ch4 -> Wrist Rotation (Yaw of end effector)
            # This is a bit complex with Quaternions. 
            # For simplicity, let's just keep orientation fixed or simple rotation.
            # current_euler = p.getEulerFromQuaternion(target_orn)
            # new_yaw = current_euler[2] + rc_data['ch4'] * 0.05
            # target_orn = p.getQuaternionFromEuler([0, 0, new_yaw])

            # Ch5 -> Gripper
            # Simple Open/Close toggle or proportional
            if rc_data['ch5'] > 0.5:
                gripper_state = 0.02 # Closed
            else:
                gripper_state = 0.0 # Open

            # 5. Calculate Inverse Kinematics
            # calculateInverseKinematics returns a list of joint positions
            joint_poses = p.calculateInverseKinematics(
                robotId, 
                ee_link_index, 
                target_pos, 
                target_orn,
                maxNumIterations=100,
                residualThreshold=1e-5
            )
            
            # joint_poses contains angles for ALL movable joints (excluding fixed base)
            # The returned list corresponds to the non-fixed joints in the URDF.
            # We need to map these back to our motor indices.
            
            # 6. Send to Servos
            # Collect angles for the arm joints
            # Arm Joints (Waist, Shoulder, Elbow, WristP, WristR, Gripper)
            current_arm_angles = []
            arm_joint_indices = [4, 5, 6, 7, 8, 9] # Exclude gripper for this list
            
            for joint_idx in arm_joint_indices:
                # joint_poses includes wheels (0-3), so index 4 is at joint_poses[4]
                angle = joint_poses[joint_idx]
                current_arm_angles.append(angle)
            
            # Send batch command
            send_all_servos(current_arm_angles, gripper_state)

            # Debug Print (Optional)
            # print(f"Target: {target_pos}, Joints: {[f'{j:.2f}' for j in joint_poses[4:]]}")

            time.sleep(0.02) # 50Hz loop

    except KeyboardInterrupt:
        print("\nExiting...")
        p.disconnect()

if __name__ == "__main__":
    main()
