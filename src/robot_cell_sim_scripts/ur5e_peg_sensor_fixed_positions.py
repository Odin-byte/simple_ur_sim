import pybullet as p
import numpy as np
from scipy.spatial.transform import Rotation as R
import time
import pybullet_data
import math
import sys
import os

# Add the parent directory to sys.path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))


from fts_driver_sim.axia_socket_server import FtnAxiaServer
from rtde_interface_sim.ur_rtde_socket_server import UrRtdeServer

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

# Transformation from world to robot
TRANSLATION_VECTOR = [0.0, 0.0, 0.6]
ROTATION_VECTOR = [0.0, 0.0, 0.0, 1.0]

STEPS_PER_POSE = 50

# Example Positions
example_pos_list = [
    # Position, Orientation
    [  # Pre press position in the center of the table
        [-0.8, -0.2, 0.65],
        p.getQuaternionFromEuler([0, math.pi, 0]),
    ],
    [  # Press position center of table
        [-0.8, -0.2, 0.633],
        p.getQuaternionFromEuler([0, math.pi, 0]),
    ],
    [
        [-0.5, -0.2, 0.70],
        p.getQuaternionFromEuler([0, math.pi, 0]),
    ],
    [
        [-0.5, 0.4, 0.70],
        p.getQuaternionFromEuler([0, math.pi, 0]),
    ],
    [
        [-0.5, 0.4, 0.61],
        p.getQuaternionFromEuler([0, math.pi, 0]),
    ],
    [  # Contact with the long edge of the table Main Torque around X-Axis
        [-0.5, 0.3, 0.61],
        p.getQuaternionFromEuler([0, math.pi, 0]),
    ],
    [
        [-0.5, 0.2, 0.61],
        p.getQuaternionFromEuler([0, math.pi, 0]),
    ],
    [  # Post Conctact Position Long Edge
        [-0.5, 0.3, 0.70],
        p.getQuaternionFromEuler([0, math.pi, 0]),
    ],
    [
        [0.0, 0.3, 0.70],
        p.getQuaternionFromEuler([0, math.pi, 0]),
    ],
    [
        [0.4, 0.3, 0.70],
        p.getQuaternionFromEuler([0, math.pi, 0]),
    ],
    [
        [0.5, -0.2, 0.70],
        p.getQuaternionFromEuler([0, math.pi, 0]),
    ],
    [
        [0.5, -0.2, 0.61],
        p.getQuaternionFromEuler([0, math.pi, 0]),
    ],
    [   # Contact Pose Short Edge # Main Torque around Y-Axis
        [0.25, -0.2, 0.61],
        p.getQuaternionFromEuler([0, math.pi, 0]),
    ],
    [
        [0.5, -0.2, 0.70],
        p.getQuaternionFromEuler([0, math.pi, 0]),
    ],
    [
        [0.4, 0.3, 0.70],
        p.getQuaternionFromEuler([0, math.pi, 0]),
    ],
    [
        [0.0, 0.3, 0.70],
        p.getQuaternionFromEuler([0, math.pi, 0]),
    ],
    [
        [-0.4, 0.0, 0.65],
        p.getQuaternionFromEuler([0, math.pi, 0]),
    ],

]


def connect_to_pybullet():
    """Setup the pybullet simulation, connecting to the GUI, searching for the needed models and setting the simulated gravity and timesteps."""
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())  # Add search path
    p.setGravity(0, 0, -0.981)  # Set gravity
    p.setTimeStep(0.01)


def load_urdf_models():
    """Load in all needed URDF files, describing the pybullet world. This includes the a plane, a table and the robot itself.

    Returns:
        pybullet body id(int): Integer which is used to refer to the robot when getting and setting values
    """
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


def transform_tcp_to_base(tcp_pose_world):
    """Given the translation and quaternion rotation defined within the constants of the sim script,
    return the given TCP pose transformed into the robot base frame.

    Args:
        tcp_pose_world (tuple[list[float, float, float], list[float, float, float, float]]):
            Position (x, y, z) and orientation (qx, qy, qz, qw) of the TCP within the world frame.

    Returns:
        tuple[list[float, float, float], list[float, float, float]]:
            Position and orientation of the TCP within the robot base frame. Rotation is represented as euler rotation with the axis
            as XYZ.
    """

    tcp_position_world, tcp_quaternion_world = tcp_pose_world

    # Convert world-to-base rotation from quaternion to rotation matrix
    rotation_matrix = R.from_quat(ROTATION_VECTOR).as_matrix()

    # Compute inverse rotation (quaternion conjugate -> inverse rotation matrix)
    inv_rotation_matrix = rotation_matrix.T  # Transpose = inverse for rotation matrices
    inv_quaternion = (
        R.from_quat(ROTATION_VECTOR).inv().as_quat()
    )  # Quaternion conjugate

    # Transform TCP position from world to base frame
    tcp_position_base = np.dot(
        inv_rotation_matrix, np.array(tcp_position_world) - TRANSLATION_VECTOR
    )

    # Transform TCP orientation from world to base frame (quaternion multiplication)
    tcp_quaternion_base = R.from_quat(inv_quaternion) * R.from_quat(
        tcp_quaternion_world
    )

    return (
        tcp_position_base.tolist()
        + tcp_quaternion_base.as_euler("xyz", degrees=False).tolist()
    )


def init_ft_sensor_server():
    """Initialize a TCP Server that sends the sensor data simulated by pybullet as if it where a Schunk FT-Axia sensor.

    Returns:
        FtnAxiaServer Object: Server object handling the external TCP communication
    """
    ft_server = FtnAxiaServer()
    ft_server.start()
    return ft_server


# TODO: Right now the TCP link is hardcoded!
def enable_ft_sensor(ur5e_id):
    """Wrapper function enabling a FT sensor at the tool joint of the simulated UR robot

    Args:
        ur5e_id (int): Integer used to refer to the kinematic chain of the simulated UR robot
    """
    p.enableJointForceTorqueSensor(ur5e_id, jointIndex=9)


def set_initial_joint_positions(ur5e_id):
    """Wrapper function setting the initial joint positions of the simulated UR robot:

    Args:
        ur5e_id (int): Integer used to refer to the kinematic chain of the simulated UR robot
    """
    joint_positions = TARGET_POSITIONS[0]
    p.setJointMotorControlArray(
        ur5e_id,
        jointIndices=JOINT_INDICES,
        controlMode=p.POSITION_CONTROL,
        targetPositions=joint_positions,
    )


# DEBUG
def get_time_based_robot_position(ur5e_id, steps):
    """Move the robot based on function internal timesteps and positions

    Args:
        ur5e_id (int): Integer used to refer to the kinematic chain of the simulated UR robot
        steps (int): Current timestep of the simulation
    """
    # Reset time variable if out of bounds
    if steps // STEPS_PER_POSE >= len(example_pos_list):
        steps = 0

    # Get current example pose based on the current timestep
    current_pose = example_pos_list[steps // STEPS_PER_POSE]

    joint_positions = p.calculateInverseKinematics(
        ur5e_id,
        endEffectorLinkIndex=10,
        targetPosition=current_pose[0],
        targetOrientation=current_pose[1],
    )

    return joint_positions, steps


# Main loop to run the simulation
def run_simulation():
    """Run the pybullet simulation."""
    steps = 0
    try:
        connect_to_pybullet()
        ur5e_id = load_urdf_models()
        ft_server = init_ft_sensor_server()
        enable_ft_sensor(ur5e_id)

        set_initial_joint_positions(ur5e_id)
        p.stepSimulation()

        while True:
            # Get control signal based on the current time step
            joint_positions, steps = get_time_based_robot_position(ur5e_id, steps)

            # Send control signal based on joint positions
            p.setJointMotorControlArray(
                ur5e_id,
                jointIndices=JOINT_INDICES,
                controlMode=p.POSITION_CONTROL,
                targetPositions=joint_positions,
            )
            p.stepSimulation()  # Step the simulation to apply the control
            # Read the current sensor value and print it to the console
            reaction_forces = p.getJointState(ur5e_id, 9)[2]
            print(f"Current sensor values: {reaction_forces}")
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
