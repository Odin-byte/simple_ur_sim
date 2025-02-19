import pybullet as p
import time
import pybullet_data
import math

# Step 1: Connect to PyBullet
p.connect(p.GUI)

# Step 2: Load URDF files
# Load the ground plane (this is typically available in pybullet_data)
p.setAdditionalSearchPath(pybullet_data.getDataPath())  # Add search path
plane_id = p.loadURDF("plane.urdf", basePosition=[0, 0, 0])  # Load a flat ground plane

# Load the UR5e robot model
ur5e_id = p.loadURDF("/ur_sim/src/ur5pybullet/urdf/universal_robots/ur_description/urdf/ur5e.urdf", basePosition=[0, 0, 0])

# Step 3: Start the simulation
p.setGravity(0, 0, -9.81)  # Set gravity
p.setTimeStep(0.01)
p.setRealTimeSimulation(0)
# Get the number of joints in the UR5e robot
num_joints = p.getNumJoints(ur5e_id)
print(f"Number of joints in the UR5e: {num_joints}")

# Get the joint information for each joint
for joint_index in range(num_joints):
    joint_info = p.getJointInfo(ur5e_id, joint_index)
    print(f"Joint {joint_index}: {joint_info}")

# Set joint positions (in radians)
joint_positions = [-1.5, 1.5, 0.0, 0, 0, 0]  # Example joint angles for the UR5e

# Lets calculate the needed joint positions for the endeffector to point straight up
needed_positions = p.calculateInverseKinematics(ur5e_id, endEffectorLinkIndex=7, targetPosition=[0.5, 0.0, 0.2], targetOrientation=[1, 0, 0, 0])

print(needed_positions)


# Step 4: Run the simulation loop
steps = 0
try:
    while True:

        # Move the robot to these joint positions
        p.setJointMotorControlArray(ur5e_id, jointIndices=list(range(1,7)), controlMode=p.POSITION_CONTROL, targetPositions=joint_positions)

        # Change positions after given steps
        if steps >= 500:
            joint_positions = [0.17251942004895773, 0.4735918889004609, 2.5406568044033517, 0.260341677189998, -0.8896989644346109, 0.0]
            p.setJointMotorControlArray(ur5e_id, jointIndices=list(range(2,8)), controlMode=p.POSITION_CONTROL, targetPositions=p.calculateInverseKinematics(ur5e_id, endEffectorLinkIndex=9, targetPosition=[0.6, 0.0, 0.5], targetOrientation=p.getQuaternionFromEuler([0, -math.pi, 0])))



        # Step the simulation to apply the control
        p.stepSimulation()
        steps += 1
        time.sleep(0.01)  # Simulation time step
except KeyboardInterrupt:
    pass

# Disconnect at the end
p.disconnect()