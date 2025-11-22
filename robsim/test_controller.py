import unittest
from controller import RobotController

class TestRobotController(unittest.TestCase):
    def setUp(self):
        self.controller = RobotController()

    def test_differential_drive_straight(self):
        # Full throttle, no steering
        left, right = self.controller.compute_wheel_velocities(1.0, 0.0)
        self.assertEqual(left, 20.0)
        self.assertEqual(right, 20.0)

    def test_differential_drive_turn(self):
        # No throttle, full right turn
        left, right = self.controller.compute_wheel_velocities(0.0, 1.0)
        self.assertEqual(left, -10.0)
        self.assertEqual(right, 10.0)

    def test_differential_drive_mix(self):
        # Full throttle, full right turn
        left, right = self.controller.compute_wheel_velocities(1.0, 1.0)
        self.assertEqual(left, 10.0) # 20 - 10
        self.assertEqual(right, 30.0) # 20 + 10

    def test_arm_joint_update(self):
        # Initial pos is 0
        initial = self.controller.arm_joints[0]
        self.assertEqual(initial, 0.0)
        
        # Increase
        new_pos = self.controller.update_arm_joint(0, 1)
        self.assertAlmostEqual(new_pos, 0.05)
        
        # Decrease
        new_pos = self.controller.update_arm_joint(0, -1)
        self.assertAlmostEqual(new_pos, 0.0)

    def test_arm_joint_limits(self):
        # Try to go beyond limit
        # Joint 6 (Gripper) limit is 0.02
        for _ in range(10):
            self.controller.update_arm_joint(6, 1)
        
        self.assertLessEqual(self.controller.arm_joints[6], 0.02)

if __name__ == '__main__':
    unittest.main()
