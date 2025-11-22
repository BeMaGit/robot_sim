import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
import serial
import time
import math

def constrain(val, min_val, max_val):
    return min(max_val, max(min_val, val))

class HardwareInterface(Node):
    def __init__(self):
        super().__init__('hardware_interface')
        
        # Parameters
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        
        self.port = self.get_parameter('serial_port').get_parameter_value().string_value
        self.baud = self.get_parameter('baudrate').get_parameter_value().integer_value
        
        # Serial Connection
        self.ser = None
        self.connect_serial()
        
        # Publishers
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        
        # Subscribers
        self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        # We can subscribe to a specific topic for arm commands, or just use joint_states if we had a controller.
        # For this simple interface, let's listen to 'joint_commands' (custom or standard)
        # But typically, we might want to be compatible with MoveIt or similar, which output JointTrajectory.
        # For simplicity, let's listen to 'joint_commands' as a JointState message for direct control.
        self.create_subscription(JointState, 'joint_commands', self.joint_command_callback, 10)
        
        # State
        self.joint_names = [
            'waist_joint', 'shoulder_joint', 'elbow_joint', 
            'wrist_pitch_joint', 'wrist_roll_joint', 'gripper_joint',
            'front_left_wheel_joint', 'front_right_wheel_joint',
            'rear_left_wheel_joint', 'rear_right_wheel_joint'
        ]
        self.current_positions = {name: 0.0 for name in self.joint_names}
        self.left_motor_cmd = 0.0
        self.right_motor_cmd = 0.0
        
        # Timer for publishing state (loop)
        self.create_timer(0.1, self.timer_callback) # 10Hz
        
        self.get_logger().info("Hardware Interface Started")

    def connect_serial(self):
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=1)
            time.sleep(2) # Wait for Arduino reset
            self.get_logger().info(f"Connected to Arduino on {self.port}")
        except Exception as e:
            self.get_logger().warn(f"Could not connect to Arduino: {e}. Running in MOCK mode.")
            self.ser = None

    def cmd_vel_callback(self, msg):
        # Differential Drive Logic
        # Wheel separation and radius should be parameters, but hardcoding for now
        # based on URDF (approximate)
        wheel_sep = 0.18 # Distance between left and right wheels
        wheel_radius = 0.04
        
        linear = msg.linear.x
        angular = msg.angular.z
        
        # Calculate wheel velocities (m/s)
        left_vel = linear - (angular * wheel_sep / 2)
        right_vel = linear + (angular * wheel_sep / 2)
        
        # Convert to percentage (-100 to 100) for Arduino PWM mapping
        # Assuming max speed is roughly 0.5 m/s
        max_speed = 0.5
        
        self.left_motor_cmd = constrain(left_vel / max_speed * 100, -100, 100)
        self.right_motor_cmd = constrain(right_vel / max_speed * 100, -100, 100)
        
        # Update wheel joint states for visualization (approximate integration)
        # In a real system, we would read encoders. Here we just integrate command.
        dt = 0.1 # Timer period
        self.current_positions['front_left_wheel_joint'] += (left_vel / wheel_radius) * dt
        self.current_positions['rear_left_wheel_joint'] += (left_vel / wheel_radius) * dt
        self.current_positions['front_right_wheel_joint'] += (right_vel / wheel_radius) * dt
        self.current_positions['rear_right_wheel_joint'] += (right_vel / wheel_radius) * dt

    def joint_command_callback(self, msg):
        # Update internal state with commanded positions
        for i, name in enumerate(msg.name):
            if name in self.current_positions:
                self.current_positions[name] = msg.position[i]
        
        # Send to Hardware
        self.send_to_arduino()

    def send_to_arduino(self):
        # Map joint positions (radians) to degrees/servo commands
        # Order: Waist, Shoulder, Elbow, WristP, WristR, Gripper
        
        angles = []
        
        # Waist
        angles.append(math.degrees(self.current_positions['waist_joint']) + 90)
        # Shoulder
        angles.append(math.degrees(self.current_positions['shoulder_joint']) + 90)
        # Elbow
        angles.append(math.degrees(self.current_positions['elbow_joint']) + 90)
        # Wrist Pitch
        angles.append(math.degrees(self.current_positions['wrist_pitch_joint']) + 90)
        # Wrist Roll
        angles.append(math.degrees(self.current_positions['wrist_roll_joint']) + 90)
        
        # Gripper (0.0 to 0.02) -> Map to servo
        # 0.0 -> 90, 0.02 -> 110 (Example from robot_control.py)
        grip_val = self.current_positions['gripper_joint']
        grip_deg = 90.0 + (grip_val / 0.02) * 20.0 # Assuming 20 deg range
        angles.append(grip_deg)
        
        # Motors
        motors = [
            self.left_motor_cmd,  # M1 (FL)
            self.right_motor_cmd, # M2 (FR)
            self.left_motor_cmd,  # M3 (RL)
            self.right_motor_cmd  # M4 (RR)
        ]
        
        # Format: CMD:s1,s2,s3,s4,s5,s6,m1,m2,m3,m4\n
        cmd_str = "CMD:" + ",".join([f"{a:.2f}" for a in angles]) + "," + ",".join([f"{m:.0f}" for m in motors]) + "\n"
        
        if self.ser and self.ser.is_open:
            try:
                self.ser.write(cmd_str.encode('utf-8'))
            except Exception as e:
                self.get_logger().error(f"Serial write error: {e}")
        else:
            # Mock Mode
            # self.get_logger().info(f"MOCK CMD: {cmd_str.strip()}")
            pass

    def timer_callback(self):
        # Publish current state (Feedback)
        # In a real system, we might read back from Arduino. 
        # Here we just echo the commands (Open Loop).
        
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self.current_positions.keys())
        msg.position = list(self.current_positions.values())
        
        self.joint_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = HardwareInterface()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.ser:
            node.ser.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
