class RobotController:
    def __init__(self):
        # Rover parameters
        self.max_speed = 20.0
        self.turn_speed = 10.0
        
        # Arm state (joint angles in radians)
        self.arm_joints = [0.0] * 6
        self.joint_limits = [
            (-3.14, 3.14), # Waist
            (-2.0, 2.0),   # Shoulder
            (-2.5, 2.5),   # Elbow
            (-1.57, 1.57), # Wrist Pitch
            (-3.14, 3.14), # Wrist Roll
            (0.0, 0.02)    # Gripper
        ]
        self.joint_step = 0.05

    def compute_wheel_velocities(self, throttle, steering):
        """
        Compute left and right wheel velocities based on throttle and steering inputs.
        throttle: -1.0 to 1.0
        steering: -1.0 to 1.0
        """
        left_vel = (throttle * self.max_speed) - (steering * self.turn_speed)
        right_vel = (throttle * self.max_speed) + (steering * self.turn_speed)
        return left_vel, right_vel

    def update_arm_joint(self, joint_index, direction):
        """
        Update the target angle for a specific joint.
        joint_index: 0-6
        direction: 1 (increase), -1 (decrease), 0 (no change)
        """
        if 0 <= joint_index < len(self.arm_joints):
            new_pos = self.arm_joints[joint_index] + (direction * self.joint_step)
            # Clamp to limits
            lower, upper = self.joint_limits[joint_index]
            self.arm_joints[joint_index] = max(lower, min(new_pos, upper))
        return self.arm_joints[joint_index]

    def get_arm_targets(self):
        return self.arm_joints

    def solve_ik(self, robot_id, end_effector_index, target_pos, target_orn=None, pybullet_module=None):
        """
        Calculate joint angles for the given target position and orientation.
        """
        if target_orn is not None:
            joint_poses = pybullet_module.calculateInverseKinematics(
                robot_id,
                end_effector_index,
                target_pos,
                targetOrientation=target_orn,
                maxNumIterations=100,
                residualThreshold=1e-5
            )
        else:
            joint_poses = pybullet_module.calculateInverseKinematics(
                robot_id,
                end_effector_index,
                target_pos,
                maxNumIterations=100,
                residualThreshold=1e-5
            )
        return joint_poses
