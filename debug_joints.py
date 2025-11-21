import pybullet as p
import pybullet_data

p.connect(p.DIRECT)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
robotId = p.loadURDF("robot_model.urdf", useFixedBase=True)

num_joints = p.getNumJoints(robotId)
print(f"Total joints: {num_joints}")
for i in range(num_joints):
    info = p.getJointInfo(robotId, i)
    print(f"Joint {i}: {info[1].decode('utf-8')} (Type: {info[2]})")

p.disconnect()
