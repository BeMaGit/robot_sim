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
    # 7: Wrist Pitch
    # 8: Wrist Roll
    # 9: Gripper
    
    wheel_indices = [0, 1, 2, 3] # FL, FR, RL, RR
    arm_indices = [4, 5, 6, 7, 8, 9]

    print("Simulation started.")
    print("Controls:")
    print("  Drive: Arrow Keys")
    print("  Arm (Unified - German Layout):")
    print("    Waist: J/L")
    print("    Extension: I/K")
    print("    Height: P/Ö")
    print("    Wrist Pitch: Z/H")
    print("    Wrist Roll: U/O")
    print("    Gripper: M/,")
    print("  Locked Mode: . (Counteract Base Turn)")
    print("  Camera: C (Toggle Gripper FPV)")

    # Control State
    target_radius = 0.3
    target_height = 0.2
    target_pitch = 0.0
    ee_index = 7
    camera_mode = False
    
    # Locked Mode State
    locked_mode = False
    # Get initial yaw
    _, base_orn = p.getBasePositionAndOrientation(robotId)
    prev_base_yaw = p.getEulerFromQuaternion(base_orn)[2]

    # --- Debug Sliders ---
    # Add sliders for arm joints
    # Limits based on controller.joint_limits or standard ranges
    # Waist: -1.57 to 1.57
    # Shoulder: -1.57 to 1.57
    # Elbow: -1.57 to 1.57
    # Wrist Pitch: -1.57 to 1.57
    # Wrist Roll: -3.14 to 3.14
    # Gripper: 0 to 0.02
    
    slider_waist = p.addUserDebugParameter("Waist", -1.57, 1.57, 0)
    slider_shoulder = p.addUserDebugParameter("Shoulder", -1.57, 1.57, 0)
    slider_elbow = p.addUserDebugParameter("Elbow", -1.57, 1.57, 0)
    slider_wrist_pitch = p.addUserDebugParameter("Wrist Pitch", -1.57, 1.57, 0)
    slider_wrist_roll = p.addUserDebugParameter("Wrist Roll", -3.14, 3.14, 0)
    slider_gripper = p.addUserDebugParameter("Gripper", 0, 0.02, 0)
    
    # Store previous slider values to detect changes
    prev_slider_values = {
        'waist': 0, 'shoulder': 0, 'elbow': 0, 
        'wrist_pitch': 0, 'wrist_roll': 0, 'gripper': 0
    }

    # Simulation Loop
    try:
        while True:
            # Get Keyboard Events
            keys = p.getKeyboardEvents()
            
            # --- Drive Control ---
            throttle = 0
            steering = 0
            if p.B3G_UP_ARROW in keys and keys[p.B3G_UP_ARROW] & p.KEY_IS_DOWN:
                throttle += 1
            if p.B3G_DOWN_ARROW in keys and keys[p.B3G_DOWN_ARROW] & p.KEY_IS_DOWN:
                throttle -= 1
            if p.B3G_LEFT_ARROW in keys and keys[p.B3G_LEFT_ARROW] & p.KEY_IS_DOWN:
                steering += 1 # Turn Left
            if p.B3G_RIGHT_ARROW in keys and keys[p.B3G_RIGHT_ARROW] & p.KEY_IS_DOWN:
                steering -= 1 # Turn Right
            
            left_vel, right_vel = controller.compute_wheel_velocities(throttle, steering)
            
            # Apply Wheel Velocities
            p.setJointMotorControl2(robotId, 0, p.VELOCITY_CONTROL, targetVelocity=left_vel, force=10)
            p.setJointMotorControl2(robotId, 2, p.VELOCITY_CONTROL, targetVelocity=left_vel, force=10)
            p.setJointMotorControl2(robotId, 1, p.VELOCITY_CONTROL, targetVelocity=right_vel, force=10)
            p.setJointMotorControl2(robotId, 3, p.VELOCITY_CONTROL, targetVelocity=right_vel, force=10)

            # --- Locked Mode Logic ---
            # Toggle
            if ord('.') in keys and keys[ord('.')] & p.KEY_WAS_TRIGGERED:
                locked_mode = not locked_mode
                print(f"Locked Mode: {'ON' if locked_mode else 'OFF'}")
            
            # Calculate Base Rotation Delta
            _, base_orn = p.getBasePositionAndOrientation(robotId)
            current_base_yaw = p.getEulerFromQuaternion(base_orn)[2]
            delta_yaw = current_base_yaw - prev_base_yaw
            prev_base_yaw = current_base_yaw
            
            # Apply Correction if Locked
            if locked_mode:
                current_waist = controller.arm_joints[0]
                new_waist = current_waist - delta_yaw
                lower, upper = controller.joint_limits[0]
                controller.arm_joints[0] = max(lower, min(new_waist, upper))

            # --- Slider Inputs ---
            # Read sliders
            val_waist = p.readUserDebugParameter(slider_waist)
            val_shoulder = p.readUserDebugParameter(slider_shoulder)
            val_elbow = p.readUserDebugParameter(slider_elbow)
            val_wrist_pitch = p.readUserDebugParameter(slider_wrist_pitch)
            val_wrist_roll = p.readUserDebugParameter(slider_wrist_roll)
            val_gripper = p.readUserDebugParameter(slider_gripper)
            
            # Check for changes and update controller if changed
            # We use a small epsilon for float comparison
            epsilon = 0.0001
            
            if abs(val_waist - prev_slider_values['waist']) > epsilon:
                controller.arm_joints[0] = val_waist
                prev_slider_values['waist'] = val_waist
                
            if abs(val_shoulder - prev_slider_values['shoulder']) > epsilon:
                controller.arm_joints[1] = val_shoulder
                prev_slider_values['shoulder'] = val_shoulder
                
            if abs(val_elbow - prev_slider_values['elbow']) > epsilon:
                controller.arm_joints[2] = val_elbow
                prev_slider_values['elbow'] = val_elbow
                
            if abs(val_wrist_pitch - prev_slider_values['wrist_pitch']) > epsilon:
                controller.arm_joints[3] = val_wrist_pitch
                prev_slider_values['wrist_pitch'] = val_wrist_pitch
                
            if abs(val_wrist_roll - prev_slider_values['wrist_roll']) > epsilon:
                controller.arm_joints[4] = val_wrist_roll
                prev_slider_values['wrist_roll'] = val_wrist_roll
                
            if abs(val_gripper - prev_slider_values['gripper']) > epsilon:
                controller.arm_joints[5] = val_gripper
                prev_slider_values['gripper'] = val_gripper

            # --- Arm Control (Unified) ---
            
            # 1. Manual Inputs (Waist, Wrist Roll, Gripper) - Keyboard Overrides
            # Map keys to controller joint index (0-5) and direction
            manual_controls = {
                ord('j'): (0, 1), ord('l'): (0, -1),   # Waist (Ctrl 0) -> J/L
                ord('u'): (4, -1), ord('o'): (4, 1),   # Wrist Roll (Ctrl 4) -> U/O (Inverted)
                ord('m'): (5, 1), ord(','): (5, -1),   # Gripper (Ctrl 5) -> M/,
            }

            for key, (idx, direction) in manual_controls.items():
                if key in keys and keys[key] & p.KEY_IS_DOWN:
                    controller.update_arm_joint(idx, direction)

            # 2. IK Inputs (Shoulder & Elbow via Extension/Height, Wrist Pitch via target_pitch)
            speed = 0.005
            # Extension: I / K
            if ord('i') in keys and keys[ord('i')] & p.KEY_IS_DOWN: target_radius += speed
            if ord('k') in keys and keys[ord('k')] & p.KEY_IS_DOWN: target_radius -= speed
            
            # Height: P / Ö (using ; for Ö as fallback/standard mapping)
            # We check for both 'ö' (if supported) and ';' (US mapping for that key)
            if ord('p') in keys and keys[ord('p')] & p.KEY_IS_DOWN: target_height += speed
            if (ord(';') in keys and keys[ord(';')] & p.KEY_IS_DOWN) or \
               (246 in keys and keys[246] & p.KEY_IS_DOWN): # 246 is ord('ö')
                target_height -= speed
            
            # Wrist Pitch Control (Target Pitch): Z / H
            if ord('z') in keys and keys[ord('z')] & p.KEY_IS_DOWN: target_pitch += speed * 2
            if ord('h') in keys and keys[ord('h')] & p.KEY_IS_DOWN: target_pitch -= speed * 2

            # 3. Calculate IK Target based on Waist Angle
            # Get current manual waist angle
            waist_angle = controller.arm_joints[0]
            
            # Calculate Target X/Y in Base Frame
            import math
            target_x = target_radius * math.cos(waist_angle)
            target_y = target_radius * math.sin(waist_angle)
            target_z = target_height
            
            target_pos_world = [target_x, target_y, target_z] 
            
            base_pos, base_orn = p.getBasePositionAndOrientation(robotId)
            # Local target relative to robot base
            target_pos_local = [target_x, target_y, target_z]
            
            # Calculate Target Orientation
            # We want the wrist pitch link to be pitched by target_pitch relative to the arm plane.
            # Arm plane is defined by waist_angle (Yaw).
            # Euler: [Roll, Pitch, Yaw] -> [0, target_pitch, waist_angle]
            target_orn_local = p.getQuaternionFromEuler([0, target_pitch, waist_angle]) 
            
            target_pos_world, target_orn_world = p.multiplyTransforms(base_pos, base_orn, target_pos_local, target_orn_local)

            # 4. Solve IK
            # ee_index = 7 (Wrist Pitch Link)
            # Pass target_orn to control pitch
            # Only solve IK if we are NOT moving sliders manually for Shoulder/Elbow?
            # Actually, let's let IK run. If sliders for Shoulder/Elbow are moved, they override.
            # But if IK runs, it overwrites the controller state for those joints.
            
            # We need to decide: Does IK always run?
            # If we want sliders to work for Shoulder/Elbow, we should probably disable IK if sliders moved.
            # But for now, let's assume IK is active if keys are pressed? No, IK is always calculating based on target_radius/height.
            
            # Let's apply IK results to controller state, BUT only if we haven't just moved the sliders.
            # This is getting tricky. 
            # Simplified: IK updates the "target" state. 
            # Let's just apply IK to the joints 1, 2, 3 (Shoulder, Elbow, Wrist Pitch).
            
            joint_poses = controller.solve_ik(robotId, ee_index, target_pos_world, target_orn=target_orn_world, pybullet_module=p)
            
            # Apply IK to controller state (so it persists)
            # Note: This will overwrite slider inputs for these joints if we are not careful.
            # But since we want "Show angles", maybe we just let the simulation drive the robot and we don't sync back to sliders (impossible).
            # If the user moves the slider, it will jump.
            
            # Let's only apply IK if we are pressing IK keys? 
            # Or just let IK be the dominant mode for these joints, and sliders are for "manual override" which might fight IK.
            # Given the user request "show them in the right panel", and my plan "use sliders for control", 
            # I will prioritize the slider if it changed.
            
            # If slider changed, we already updated controller.arm_joints.
            # Now we run IK. IK produces new angles.
            # We should probably NOT apply IK if we just moved the slider.
            
            # For now, let's apply IK.
            # Shoulder
            # p.setJointMotorControl2(robotId, 5, p.POSITION_CONTROL, targetPosition=joint_poses[5], force=50)
            # Elbow
            # p.setJointMotorControl2(robotId, 6, p.POSITION_CONTROL, targetPosition=joint_poses[6], force=50)
            # Wrist Pitch
            # p.setJointMotorControl2(robotId, 7, p.POSITION_CONTROL, targetPosition=joint_poses[7], force=50)
            
            # Update controller state to match IK (so we are consistent)
            # controller.arm_joints[1] = joint_poses[5]
            # controller.arm_joints[2] = joint_poses[6]
            # controller.arm_joints[3] = joint_poses[7]
            
            # WAIT. If I update controller state here, the slider check in next frame will see "no change" (because slider didn't move, but internal val did).
            # That's fine.
            # But if I move slider, I update controller.arm_joints.
            # Then IK overwrites it immediately here?
            # Yes. So Slider for Shoulder/Elbow won't work if IK is running.
            
            # Fix: Only apply IK if we didn't just move the slider for that joint?
            # Or better: Just apply the controller.arm_joints to the motors.
            # And let IK update controller.arm_joints ONLY if IK keys are pressed?
            # That would be cleaner.
            
            ik_keys = [ord('i'), ord('k'), ord('p'), ord(';'), 246, ord('z'), ord('h')]
            ik_active = any(k in keys and keys[k] & p.KEY_IS_DOWN for k in ik_keys)
            
            if ik_active:
                 # Update controller state from IK
                 controller.arm_joints[1] = joint_poses[5]
                 controller.arm_joints[2] = joint_poses[6]
                 controller.arm_joints[3] = joint_poses[7]

            # 5. Apply Targets from Controller State
            # Waist (Sim 4 / Ctrl 0)
            p.setJointMotorControl2(robotId, 4, p.POSITION_CONTROL, targetPosition=controller.arm_joints[0], force=50)
            
            # Shoulder (Sim 5 / Ctrl 1)
            p.setJointMotorControl2(robotId, 5, p.POSITION_CONTROL, targetPosition=controller.arm_joints[1], force=50)
            
            # Elbow (Sim 6 / Ctrl 2)
            p.setJointMotorControl2(robotId, 6, p.POSITION_CONTROL, targetPosition=controller.arm_joints[2], force=50)
            
            # Wrist Pitch (Sim 7 / Ctrl 3)
            p.setJointMotorControl2(robotId, 7, p.POSITION_CONTROL, targetPosition=controller.arm_joints[3], force=50)
            
            # Wrist Roll (Sim 8 / Ctrl 4)
            p.setJointMotorControl2(robotId, 8, p.POSITION_CONTROL, targetPosition=controller.arm_joints[4], force=50)
            
            # Gripper (Sim 9 / Ctrl 5)
            p.setJointMotorControl2(robotId, 9, p.POSITION_CONTROL, targetPosition=controller.arm_joints[5], force=50)

            # Camera Control
            if ord('c') in keys and keys[ord('c')] & p.KEY_WAS_TRIGGERED:
                camera_mode = not camera_mode
                if not camera_mode:
                    # Reset to default view
                    p.resetDebugVisualizerCamera(cameraDistance=1.5, cameraYaw=45, cameraPitch=-30, cameraTargetPosition=[0,0,0])
            
            if camera_mode:
                # FPV from Wrist Pitch Link (Index 7)
                # We want the camera to be "mounted" on the wrist pitch link (where the rectangle is).
                # This implies a small offset in the local Z direction (up) and maybe X (forward/back).
                
                link_state = p.getLinkState(robotId, 7)
                link_pos = link_state[0]
                link_orn = link_state[1]
                
                # Calculate Camera Position (The "Rectangle" spot)
                # Offset: 0.05m Up (Z), 0.0m Forward (X) relative to link frame
                # Adjust these values to match the visual "rectangle"
                camera_offset_local = [0, 0, 0.08] 
                
                # Rotate offset to world frame
                import pybullet_utils.bullet_client as bc
                # We can use p.multiplyTransforms to apply the offset
                # Identity quaternion for the offset rotation (we just want the point)
                cam_pos_world, _ = p.multiplyTransforms(link_pos, link_orn, camera_offset_local, [0,0,0,1])
                
                euler = p.getEulerFromQuaternion(link_orn)
                yaw_deg = euler[2] * 180 / 3.14159
                pitch_deg = euler[1] * 180 / 3.14159
                
                # We want to look FORWARD from this position.
                # resetDebugVisualizerCamera orbits a target.
                # To simulate FPV, we set the target to be slightly in FRONT of the camera position,
                # and the camera distance to be very small (or 0 if allowed, but usually needs >0).
                # Alternatively, we set the target to the camera position (the rectangle) and pull the camera back?
                # No, user wants camera *at* the rectangle.
                
                # Let's try setting the target to a point 1m in front of the camera.
                # Forward vector in local frame is [1, 0, 0] (assuming X is forward along arm)
                look_at_offset_local = [1.0, 0, 0.08] # 1m forward, same height
                look_at_pos_world, _ = p.multiplyTransforms(link_pos, link_orn, look_at_offset_local, [0,0,0,1])
                
                # Set camera to look at that point, from the camera position.
                # Distance = 1.0
                # Yaw/Pitch should match the link's orientation.
                
                # Note: cameraYaw is relative to world Z? Or view?
                # PyBullet cameraYaw/Pitch are somewhat absolute but centered on target.
                # Yaw: Rotation around Z axis.
                # Pitch: Angle with XY plane.
                
                # We use the link's Yaw and Pitch directly.
                # We might need to adjust offsets (e.g. -90) depending on model orientation.
                
                p.resetDebugVisualizerCamera(cameraDistance=1.0, cameraYaw=yaw_deg, cameraPitch=pitch_deg, cameraTargetPosition=look_at_pos_world)

            # --- Debug Info ---
            # Removed 3D text as requested.
            # Sliders in "Params" panel show the values (inputs).
            
            p.stepSimulation()
            time.sleep(1./240.)
            
    except KeyboardInterrupt:
        pass
    
    p.disconnect()

if __name__ == "__main__":
    main()
