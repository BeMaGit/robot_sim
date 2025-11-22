import pybullet as p
import pybullet_data
import time
import sys

def main():
    try:
        # Connect to PyBullet in DIRECT mode (headless)
        physicsClient = p.connect(p.DIRECT)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)

        # Load Ground
        planeId = p.loadURDF("plane.urdf")
        print("Ground loaded")

        # Load Robot
        startPos = [0, 0, 0.2]
        startOrientation = p.getQuaternionFromEuler([0, 0, 0])
        robotId = p.loadURDF("robot_model.urdf", startPos, startOrientation)
        print(f"Robot loaded with ID: {robotId}")

        # Check joints
        numJoints = p.getNumJoints(robotId)
        print(f"Number of joints: {numJoints}")
        for i in range(numJoints):
            info = p.getJointInfo(robotId, i)
            print(f"Joint {i}: {info[1].decode('utf-8')}")

        # Run a few steps
        for i in range(100):
            p.stepSimulation()
        
        print("Simulation test passed successfully.")
        p.disconnect()
        sys.exit(0)
    except Exception as e:
        print(f"Simulation failed: {e}")
        sys.exit(1)

if __name__ == "__main__":
    main()
