import pybullet as p
import pybullet_data
import time
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
    p.resetDebugVisualizerCamera(cameraDistance=1.5, cameraYaw=45, cameraPitch=-30, cameraTargetPosition=[0,0,0])

    # Initialize Controller
    controller = RobotController()
    
    # Joint Indices (based on URDF order)
    # 0-3: Wheels (FL, FR, RL, RR)
    # 4: Waist
    # 5: Shoulder
    # 6: Elbow
    # 7: Forearm Roll
    # 8: Wrist Pitch
    # 9: Wrist Roll
    # 10: Gripper
    
    wheel_indices = [0, 1, 2, 3] # FL, FR, RL, RR
    arm_indices = [4, 5, 6, 7, 8, 9, 10]

    print("Simulation started.")
    print("Controls:")
    print("  Drive: Arrow Keys")
    print("  Arm (Joint): 1/2, 3/4, 5/6, 7/8, 9/0, -/=, [/]")
    print("  Arm (IK): I/K (X), J/L (Y), U/O (Z), N/M (Yaw), Space (Grip)")
    print("  Toggle Mode: T (Joint <-> IK)")

    # IK State
    ik_mode = False
    ee_index = 10
    target_pos_local = [0.3, 0.0, 0.2]
    target_orn_local = p.getQuaternionFromEuler([0, 0, 0])
    gripper_state = 0.0

    # Simulation Loop
    try:
        while True:
            # Get Keyboard Events
            keys = p.getKeyboardEvents()
            
            # Drive Inputs
            throttle = 0
            steering = 0
            if p.B3G_UP_ARROW in keys and keys[p.B3G_UP_ARROW] & p.KEY_IS_DOWN:
                throttle += 1
            if p.B3G_DOWN_ARROW in keys and keys[p.B3G_DOWN_ARROW] & p.KEY_IS_DOWN:
                throttle -= 1
            if p.B3G_LEFT_ARROW in keys and keys[p.B3G_LEFT_ARROW] & p.KEY_IS_DOWN:
                steering -= 1
            if p.B3G_RIGHT_ARROW in keys and keys[p.B3G_RIGHT_ARROW] & p.KEY_IS_DOWN:
                steering += 1
            
            left_vel, right_vel = controller.compute_wheel_velocities(throttle, steering)
            
            # Apply Wheel Velocities
            # Assuming FL and RL are left, FR and RR are right
            p.setJointMotorControl2(robotId, 0, p.VELOCITY_CONTROL, targetVelocity=left_vel, force=10)
            p.setJointMotorControl2(robotId, 2, p.VELOCITY_CONTROL, targetVelocity=left_vel, force=10)
            p.setJointMotorControl2(robotId, 1, p.VELOCITY_CONTROL, targetVelocity=right_vel, force=10)
            p.setJointMotorControl2(robotId, 3, p.VELOCITY_CONTROL, targetVelocity=right_vel, force=10)

            # Arm Inputs
            # Map keys to joint index and direction
            # Toggle Mode
            if ord('t') in keys and keys[ord('t')] & p.KEY_WAS_TRIGGERED:
                ik_mode = not ik_mode
                print(f"Control Mode: {'IK' if ik_mode else 'Joint'}")
                
                if ik_mode:
                    # Sync IK target to current actual gripper position (Local Frame)
                    # 1. Get Gripper World Pose
                    link_state = p.getLinkState(robotId, ee_index)
                    gripper_world_pos = link_state[0]
                    gripper_world_orn = link_state[1]
                    
                    # 2. Get Base World Pose
                    base_pos, base_orn = p.getBasePositionAndOrientation(robotId)
                    
                    # 3. Convert Gripper World -> Gripper Local (relative to Base)
                    inv_base_pos, inv_base_orn = p.invertTransform(base_pos, base_orn)
                    local_pos, local_orn = p.multiplyTransforms(inv_base_pos, inv_base_orn, gripper_world_pos, gripper_world_orn)
                    
                    # 4. Update Target
                    target_pos_local = list(local_pos)
                    target_orn_local = list(local_orn)
                    print(f"IK Synced to Local Pos: {target_pos_local}")

            if not ik_mode:
                # Joint Control Mode
                # Map keys to joint index and direction
                arm_controls = {
                    ord('1'): (0, 1), ord('2'): (0, -1), # Waist
                    ord('3'): (1, 1), ord('4'): (1, -1), # Shoulder
                    ord('5'): (2, 1), ord('6'): (2, -1), # Elbow
                    ord('7'): (3, 1), ord('8'): (3, -1), # Forearm
                    ord('9'): (4, 1), ord('0'): (4, -1), # Wrist Pitch
                    ord('-'): (5, 1), ord('='): (5, -1), # Wrist Roll
                    ord('['): (6, 1), ord(']'): (6, -1), # Gripper
                }

                for key, (idx, direction) in arm_controls.items():
                    if key in keys and keys[key] & p.KEY_IS_DOWN:
                        controller.update_arm_joint(idx, direction)
                
                # Apply Arm Targets (Joint Mode)
                targets = controller.get_arm_targets()
                for i, target in enumerate(targets):
                    joint_idx = arm_indices[i]
                    p.setJointMotorControl2(robotId, joint_idx, p.POSITION_CONTROL, targetPosition=target, force=50)

            else:
                # IK Control Mode
                speed = 0.005
                if ord('i') in keys and keys[ord('i')] & p.KEY_IS_DOWN: target_pos_local[0] += speed
                if ord('k') in keys and keys[ord('k')] & p.KEY_IS_DOWN: target_pos_local[0] -= speed
                if ord('j') in keys and keys[ord('j')] & p.KEY_IS_DOWN: target_pos_local[1] += speed
                if ord('l') in keys and keys[ord('l')] & p.KEY_IS_DOWN: target_pos_local[1] -= speed
                if ord('u') in keys and keys[ord('u')] & p.KEY_IS_DOWN: target_pos_local[2] += speed
                if ord('o') in keys and keys[ord('o')] & p.KEY_IS_DOWN: target_pos_local[2] -= speed
                
                # Gripper
                if ord(' ') in keys and keys[ord(' ')] & p.KEY_WAS_TRIGGERED:
                    gripper_state = 0.02 if gripper_state == 0.0 else 0.0
                
                # Transform Local Target to World Target
                base_pos, base_orn = p.getBasePositionAndOrientation(robotId)
                target_pos_world, target_orn_world = p.multiplyTransforms(base_pos, base_orn, target_pos_local, target_orn_local)

                # Solve IK
                joint_poses = controller.solve_ik(robotId, ee_index, target_pos_world, target_orn_world, p)
                
                # Apply IK Targets
                # joint_poses contains all movable joints. 
                # Arm joints are indices 4-10 in URDF, which correspond to indices 4-10 in joint_poses if wheels are 0-3.
                for i, joint_idx in enumerate(arm_indices):
                    # joint_poses has values for all joints.
                    # We need to be careful about mapping.
                    # calculateInverseKinematics returns a tuple of positions for each degree of freedom.
                    # If wheels are movable, they are included.
                    angle = joint_poses[joint_idx]
                    p.setJointMotorControl2(robotId, joint_idx, p.POSITION_CONTROL, targetPosition=angle, force=50)
                
                # Override Gripper (IK might not solve it perfectly or we want manual toggle)
                p.setJointMotorControl2(robotId, 10, p.POSITION_CONTROL, targetPosition=gripper_state, force=50)

            p.stepSimulation()
            time.sleep(1./240.)
            
    except KeyboardInterrupt:
        pass
    
    p.disconnect()

if __name__ == "__main__":
    main()
