import pybullet as p
import time
import pybullet_data
import math
import sys
import os

# Add the parent directory to sys.path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))


from fts_driver_sim.axia_socket_server import FtnAxiaServer

# Constants for joint indices and target positions
JOINT_INDICES = list(range(2, 8))
TARGET_POSITIONS = [
    [1.5, -1.5, 0.0, 0, 0, 0],
    [
        0.17251942004895773,
        0.4735918889004609,
        2.5406568044033517,
        0.260341677189998,
        -0.8896989644346109,
        0.0,
    ],
]


# Step 1: Connect to PyBullet
def connect_to_pybullet():
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())  # Add search path
    p.setGravity(0, 0, -0.981)  # Set gravity
    p.setTimeStep(0.01)


# Step 2: Load URDF files
def load_urdf_models():
    plane_id = p.loadURDF(
        "plane.urdf", basePosition=[0, 0, 0]
    )  # Load a flat ground plane
    table_id = p.loadURDF(
        "/ur_sim/src/ur5pybullet/urdf/cell/table.urdf", basePosition=[-0.5, -0.3, 0]
    )
    ur5e_id = p.loadURDF(
        "/ur_sim/src/ur5pybullet/urdf/universal_robots/ur_description/urdf/ur5e_with_peg.urdf",
        basePosition=[0, 0, 0],
    )
    return ur5e_id


# Step 3: Initialize Force-Torque Sensor Server
def init_ft_sensor_server():
    ft_server = FtnAxiaServer()
    ft_server.start()
    return ft_server


# Step 4: Enable Force-Torque Sensors
def enable_ft_sensor(ur5e_id):
    p.enableJointForceTorqueSensor(ur5e_id, jointIndex=9)


# Step 5: Set Initial Joint Positions
def set_initial_joint_positions(ur5e_id):
    joint_positions = TARGET_POSITIONS[0]
    p.setJointMotorControlArray(
        ur5e_id,
        jointIndices=JOINT_INDICES,
        controlMode=p.POSITION_CONTROL,
        targetPositions=joint_positions,
    )


# Step 6: Move Robot to Target Position Using Inverse Kinematics
def move_robot_to_target_position(ur5e_id, ft_server, steps):
    if steps >= 500:
        target_pos = [-0.8, -0.2, 0.65]
        target_orientation = p.getQuaternionFromEuler([0, -math.pi, 0])
        p.setJointMotorControlArray(
            ur5e_id,
            jointIndices=JOINT_INDICES,
            controlMode=p.POSITION_CONTROL,
            targetPositions=p.calculateInverseKinematics(
                ur5e_id,
                endEffectorLinkIndex=10,
                targetPosition=target_pos,
                targetOrientation=target_orientation,
            ),
        )

    if steps >= 700:
        target_pos = [-0.8, -0.2, 0.633]
        p.setJointMotorControlArray(
            ur5e_id,
            jointIndices=JOINT_INDICES,
            controlMode=p.POSITION_CONTROL,
            targetPositions=p.calculateInverseKinematics(
                ur5e_id,
                endEffectorLinkIndex=10,
                targetPosition=target_pos,
                targetOrientation=target_orientation,
            ),
        )

    ft_server.set_values(p.getJointState(ur5e_id, 9))


# Main loop to run the simulation
def run_simulation():
    steps = 0
    try:
        connect_to_pybullet()
        ur5e_id = load_urdf_models()
        ft_server = init_ft_sensor_server()
        enable_ft_sensor(ur5e_id)
        set_initial_joint_positions(ur5e_id)

        while True:
            move_robot_to_target_position(ur5e_id, ft_server, steps)

            p.stepSimulation()  # Step the simulation to apply the control
            time.sleep(0.01)  # Simulation time step
            steps += 1

    except KeyboardInterrupt:
        print("Simulation interrupted.")
    finally:
        p.disconnect()
        ft_server.stop()
        print("PyBullet disconnected.")


if __name__ == "__main__":
    run_simulation()
