import pybullet as p
import pybullet_data
import time

def main():
    p.connect(p.DIRECT)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    robotId = p.loadURDF("robot_model.urdf", useFixedBase=True)

    num_joints = p.getNumJoints(robotId)
    print(f"Total Joints: {num_joints}")
    for i in range(num_joints):
        info = p.getJointInfo(robotId, i)
        # info[0] is index, info[1] is name
        print(f"Joint {i}: {info[1].decode('utf-8')} (Type: {info[2]})")

    # Test IK
    ee_index = 10
    target_pos = [0.3, 0.0, 0.2]
    target_orn = p.getQuaternionFromEuler([0, 0, 0])
    
    print(f"\nTesting IK for Target: {target_pos}")
    joint_poses = p.calculateInverseKinematics(robotId, ee_index, target_pos, target_orn)
    
    print(f"IK Result Length: {len(joint_poses)}")
    print(f"IK Result: {joint_poses}")

    p.disconnect()

if __name__ == "__main__":
    main()
