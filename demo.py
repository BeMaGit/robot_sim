import pybullet as p
import pybullet_data
import time
import math
from controller import RobotController

def main():
    # Connect to PyBullet
    physicsClient = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)

    # Load Ground
    planeId = p.loadURDF("plane.urdf")

    # Load Robot
    startPos = [0, 0, 0.2]
    startOrientation = p.getQuaternionFromEuler([0, 0, 0])
    robotId = p.loadURDF("robot_model.urdf", startPos, startOrientation)

    # Camera setup
    p.resetDebugVisualizerCamera(cameraDistance=2.0, cameraYaw=45, cameraPitch=-30, cameraTargetPosition=[0,0,0])

    # Initialize Controller
    controller = RobotController()
    
    # Joint Indices (based on URDF order)
    wheel_indices = [0, 1, 2, 3] # FL, FR, RL, RR
    arm_indices = [4, 5, 6, 7, 8, 9, 10]

    print("Demo started.")
    print("Press Ctrl+C to exit.")

    # Demo Sequence Definition
    # Each step: (duration_seconds, throttle, steering, arm_action_func)
    
    def arm_wave(t):
        # Simple wave: oscillate forearm
        return {
            3: math.sin(t * 5) * 0.5 # Forearm
        }

    def arm_lift(t):
        # Lift shoulder and elbow
        return {
            1: -0.5, # Shoulder
            2: -1.0  # Elbow
        }
    
    def arm_reset(t):
        return {}

    # (Duration, Throttle, Steering, Arm Function)
    sequence = [
        (2.0, 1.0, 0.0, None),      # Drive Forward
        (2.0, -1.0, 0.0, None),     # Drive Backward
        (2.0, 0.0, 1.0, None),      # Turn Right
        (2.0, 0.0, -1.0, None),     # Turn Left
        (3.0, 0.0, 0.0, arm_lift),  # Lift Arm
        (3.0, 0.0, 0.0, arm_wave),  # Wave
        (2.0, 0.0, 0.0, arm_reset), # Reset
    ]

    start_time = time.time()
    current_step_idx = 0
    step_start_time = start_time

    try:
        while True:
            current_time = time.time()
            elapsed_in_step = current_time - step_start_time
            
            if current_step_idx >= len(sequence):
                # Restart sequence
                current_step_idx = 0
                step_start_time = current_time
                elapsed_in_step = 0
            
            duration, throttle, steering, arm_func = sequence[current_step_idx]
            
            if elapsed_in_step > duration:
                current_step_idx += 1
                step_start_time = current_time
                continue

            # --- Drive Control ---
            left_vel, right_vel = controller.compute_wheel_velocities(throttle, steering)
            
            # Apply Wheel Velocities
            p.setJointMotorControl2(robotId, 0, p.VELOCITY_CONTROL, targetVelocity=left_vel, force=10)
            p.setJointMotorControl2(robotId, 2, p.VELOCITY_CONTROL, targetVelocity=left_vel, force=10)
            p.setJointMotorControl2(robotId, 1, p.VELOCITY_CONTROL, targetVelocity=right_vel, force=10)
            p.setJointMotorControl2(robotId, 3, p.VELOCITY_CONTROL, targetVelocity=right_vel, force=10)

            # --- Arm Control ---
            # Get current targets from controller (holds state)
            # For the demo, we might want to directly set targets or update the controller state
            # Let's directly set targets based on the arm_func for smooth animation
            
            current_arm_targets = list(controller.get_arm_targets()) # Copy current state
            
            if arm_func:
                overrides = arm_func(elapsed_in_step)
                for joint_offset, value in overrides.items():
                    # Map relative joint index (0-6) to actual joint index
                    # But wait, controller.arm_joints is 0-6.
                    # Let's just update the specific indices in our temporary target list
                    if 0 <= joint_offset < len(current_arm_targets):
                         current_arm_targets[joint_offset] = value
            else:
                # Reset to 0 if no func? Or keep last? 
                # For simplicity in this demo, let's default to 0s if not specified, or keep controller state
                # Let's just use 0s for "rest"
                current_arm_targets = [0.0] * 7

            # Apply Arm Targets
            for i, target in enumerate(current_arm_targets):
                joint_idx = arm_indices[i]
                p.setJointMotorControl2(robotId, joint_idx, p.POSITION_CONTROL, targetPosition=target, force=50)

            # Update Camera to follow robot
            pos, orn = p.getBasePositionAndOrientation(robotId)
            p.resetDebugVisualizerCamera(cameraDistance=2.0, cameraYaw=45, cameraPitch=-30, cameraTargetPosition=pos)

            p.stepSimulation()
            time.sleep(1./240.)
            
    except KeyboardInterrupt:
        pass
    
    p.disconnect()

if __name__ == "__main__":
    main()
