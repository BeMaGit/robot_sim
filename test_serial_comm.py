import unittest
from unittest.mock import MagicMock, patch
import sys
import os

# Add current directory to path so we can import robot_control
sys.path.append(os.getcwd())

# Mock serial and pybullet before importing robot_control
sys.modules['serial'] = MagicMock()
sys.modules['pybullet'] = MagicMock()
sys.modules['pybullet_data'] = MagicMock()

from robot_control import ArduinoInterface, send_all_servos
import robot_control

class TestArduinoInterface(unittest.TestCase):
    def setUp(self):
        # Reset the global arduino instance
        robot_control.arduino = None
        
    @patch('robot_control.serial.Serial')
    def test_initialization(self, mock_serial):
        # Test successful connection
        interface = ArduinoInterface(port='/dev/ttyUSB0')
        mock_serial.assert_called_with('/dev/ttyUSB0', 115200, timeout=1)
        self.assertIsNotNone(interface.ser)

    @patch('robot_control.serial.Serial')
    def test_send_command(self, mock_serial):
        # Setup mock
        mock_ser_instance = MagicMock()
        mock_ser_instance.is_open = True
        mock_serial.return_value = mock_ser_instance
        
        interface = ArduinoInterface()
        
        # Test sending angles
        angles = [90.0, 45.5, 120.0, 0.0, 180.0, 90.0, 100.0]
        interface.send_command(angles)
        
        expected_str = "CMD:90.00,45.50,120.00,0.00,180.00,90.00,100.00\n"
        mock_ser_instance.write.assert_called_with(expected_str.encode('utf-8'))

    @patch('robot_control.serial.Serial')
    def test_send_all_servos_integration(self, mock_serial):
        # Setup global arduino
        mock_ser_instance = MagicMock()
        mock_ser_instance.is_open = True
        mock_serial.return_value = mock_ser_instance
        
        robot_control.arduino = ArduinoInterface()
        
        # Test send_all_servos
        # arm_angles in radians. 0 rad -> 90 deg.
        arm_angles = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] 
        gripper_val = 0.0 # -> 90 deg
        
        send_all_servos(arm_angles, gripper_val)
        
        # Expected: 7 angles of 90.0
        expected_str = "CMD:90.00,90.00,90.00,90.00,90.00,90.00,90.00\n"
        mock_ser_instance.write.assert_called_with(expected_str.encode('utf-8'))

if __name__ == '__main__':
    unittest.main()
