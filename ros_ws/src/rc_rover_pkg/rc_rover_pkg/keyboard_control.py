#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
import sys, select, termios, tty
import math
import threading

# Key Mappings
msg = """
Reading from the keyboard  and Publishing to Twist!
---------------------------
Moving around:
   Arrow Keys

Arm Control (Unified):
   Waist: J/L
   Extension: I/K (IK)
   Height: P/; (IK)
   Wrist Pitch: Z/H (IK target pitch)
   Wrist Roll: U/O
   Gripper: M/,

Locked Mode: . (Counteract Base Turn)
CTRL-C to quit
"""

class KeyboardControlNode(Node):
    def __init__(self):
        super().__init__('keyboard_control')
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.joint_pub = self.create_publisher(JointState, 'joint_commands', 10)
        
        # Subscribers
        self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        
        # Parameters / State
        self.settings = termios.tcgetattr(sys.stdin)
        self.speed = 0.5
        self.turn = 1.0
        
        # Robot State
        self.joint_names = [
            'waist_joint', 'shoulder_joint', 'elbow_joint', 
            'wrist_pitch_joint', 'wrist_roll_joint', 'gripper_joint'
        ]
        self.joint_positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] # Initial angles
        
        # IK State
        self.target_radius = 0.15 # Initial estimate
        self.target_height = 0.20 # Initial estimate
        self.target_pitch = 0.0
        
        # Locked Mode
        self.locked_mode = False
        self.current_yaw = 0.0
        self.prev_yaw = 0.0
        
        # Link Lengths (from URDF)
        self.L1 = 0.10 # Shoulder to Elbow
        self.L2 = 0.08 # Elbow to Wrist Pitch
        # Shoulder offset from Base: Z=0.08 (0.055 Waist + 0.025 Shoulder)
        self.SHOULDER_Z_OFFSET = 0.08
        
        # Timer for control loop (input processing)
        self.create_timer(0.05, self.control_loop)
        
        print(msg)

    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
            if key == '\x1b':
                key = sys.stdin.read(2)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def odom_callback(self, msg):
        # Extract Yaw from Quaternion
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

    def solve_ik(self, r, z, target_pitch):
        # Geometric IK for 2-link planar arm (Shoulder, Elbow)
        # Target (r, z) is relative to Shoulder Joint
        
        # r is horizontal distance from shoulder axis
        # z is vertical distance from shoulder axis
        
        # Adjust z to be relative to shoulder
        z_rel = z - self.SHOULDER_Z_OFFSET
        
        # Distance to target
        d_sq = r**2 + z_rel**2
        d = math.sqrt(d_sq)
        
        if d > (self.L1 + self.L2):
            d = self.L1 + self.L2 # Clamp to reach
            # Recompute r, z to be on the boundary? 
            # Or just let the math fail/clamp?
            # Let's just clamp d for the angle calculation, but r/z might be inconsistent.
            # Better to scale r, z_rel
            scale = (self.L1 + self.L2) / d
            r *= scale
            z_rel *= scale
            d = self.L1 + self.L2

        # Law of Cosines for Elbow Angle (theta2)
        # d^2 = L1^2 + L2^2 - 2*L1*L2*cos(pi - theta2)
        # d^2 = L1^2 + L2^2 + 2*L1*L2*cos(theta2)
        # cos(theta2) = (d^2 - L1^2 - L2^2) / (2*L1*L2)
        
        cos_theta2 = (d**2 - self.L1**2 - self.L2**2) / (2 * self.L1 * self.L2)
        cos_theta2 = max(-1.0, min(1.0, cos_theta2)) # Clamp for numerical stability
        theta2 = -math.acos(cos_theta2) # Elbow up/down? Sim usually has elbow up or down. 
        # In sim, Elbow limit is -2.5 to 2.5. 
        # Let's try negative (standard elbow up configuration often requires specific sign)
        # Actually, let's check sim behavior. 
        # If I look at the robot, elbow usually bends "forward" or "up".
        # Let's stick with one solution.
        
        # Shoulder Angle (theta1)
        # theta1 = atan2(z, r) - atan2(L2*sin(theta2), L1 + L2*cos(theta2))
        # Wait, standard formula:
        # alpha = atan2(z, r)
        # beta = acos((L1^2 + d^2 - L2^2) / (2*L1*d))
        # theta1 = alpha + beta (or alpha - beta)
        
        alpha = math.atan2(z_rel, r)
        cos_beta = (self.L1**2 + d**2 - self.L2**2) / (2 * self.L1 * d)
        cos_beta = max(-1.0, min(1.0, cos_beta))
        beta = math.acos(cos_beta)
        
        theta1 = alpha + beta # Try this solution
        
        # Wrist Pitch
        # Global pitch = theta1 + theta2 + theta3
        # We want global pitch = target_pitch
        # theta3 = target_pitch - theta1 - theta2
        theta3 = target_pitch - theta1 - theta2
        
        return theta1, theta2, theta3

    def control_loop(self):
        key = self.getKey()
        
        # Base Control
        linear = 0.0
        angular = 0.0
        
        # Arm Control Deltas
        waist_delta = 0.0
        wrist_roll_delta = 0.0
        gripper_delta = 0.0
        
        ik_change = False
        
        if key == '[A': # Up Arrow
            linear = self.speed
        elif key == '[B': # Down Arrow
            linear = -self.speed
        elif key == '[C': # Right Arrow
            angular = -self.turn
        elif key == '[D': # Left Arrow
            angular = self.turn
            
        elif key == 'j': waist_delta = 0.05
        elif key == 'l': waist_delta = -0.05
        
        elif key == 'u': wrist_roll_delta = -0.1
        elif key == 'o': wrist_roll_delta = 0.1
        
        elif key == 'm': gripper_delta = 0.001
        elif key == ',': gripper_delta = -0.001
        
        elif key == 'i': 
            self.target_radius += 0.005
            ik_change = True
        elif key == 'k': 
            self.target_radius -= 0.005
            ik_change = True
            
        elif key == 'p': 
            self.target_height += 0.005
            ik_change = True
        elif key == ';': 
            self.target_height -= 0.005
            ik_change = True
            
        elif key == 'z': 
            self.target_pitch += 0.05
            ik_change = True
        elif key == 'h': 
            self.target_pitch -= 0.05
            ik_change = True
            
        elif key == '.':
            self.locked_mode = not self.locked_mode
            self.get_logger().info(f"Locked Mode: {self.locked_mode}")
            
        elif key == '\x03': # CTRL-C
            rclpy.shutdown()
            return

        # Publish Twist
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        self.cmd_vel_pub.publish(twist)
        
        # Update Arm State
        # 1. Waist
        self.joint_positions[0] += waist_delta
        
        # Locked Mode Correction
        if self.locked_mode:
            delta_yaw = self.current_yaw - self.prev_yaw
            # Counteract base rotation
            self.joint_positions[0] -= delta_yaw
            
        self.prev_yaw = self.current_yaw
        
        # 2. Wrist Roll
        self.joint_positions[4] += wrist_roll_delta
        
        # 3. Gripper
        self.joint_positions[5] += gripper_delta
        
        # 4. IK Update
        if ik_change or waist_delta != 0: # Update IK if target changed
             # We need to maintain consistency. 
             # If we use IK, we overwrite Shoulder, Elbow, WristPitch
             pass
             
        # Always run IK based on current targets?
        # In sim, IK runs every frame.
        theta1, theta2, theta3 = self.solve_ik(self.target_radius, self.target_height, self.target_pitch)
        self.joint_positions[1] = theta1
        self.joint_positions[2] = theta2
        self.joint_positions[3] = theta3
        
        # Clamp Limits (Approximate)
        self.joint_positions[0] = max(-3.14, min(3.14, self.joint_positions[0])) # Waist
        self.joint_positions[5] = max(0.0, min(0.02, self.joint_positions[5]))   # Gripper
        
        # Publish Joint Commands
        joint_msg = JointState()
        joint_msg.header.stamp = self.get_clock().now().to_msg()
        joint_msg.name = self.joint_names
        joint_msg.position = self.joint_positions
        self.joint_pub.publish(joint_msg)

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardControlNode()
    try:
        rclpy.spin(node)
    except Exception as e:
        print(e)
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, node.settings)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
