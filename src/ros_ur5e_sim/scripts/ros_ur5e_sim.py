#!/usr/bin/env python3
import rospy
from threading import Lock
import actionlib
from geometry_msgs.msg import Pose, PoseStamped, Wrench
from ros_ur_msgs.msg import ap_movePoseAction, ap_movePoseResult

import pybullet as p
import numpy as np
from scipy.spatial.transform import Rotation as R
import pybullet_data

# Transformation from world to robot
TRANSLATION_VECTOR = [0.0, 0.0, 0.6]
ROTATION_VECTOR = [0.0, 0.0, 0.0, 1.0]

# PID values for torque control
KP = 200.0
KD = 10


class Ur_PyBullet_Ros_Sim:
    def __init__(self):
        # ROS Action Server
        self.requested_position = None
        self.requested_orientation = None
        self._lock = Lock()  # Thread-safe lock for accessing position and orientation

        self._feedback_pose = Pose()
        self._result_msg = ap_movePoseResult()

        self._action_name = "move_pose"
        rospy.init_node("ros_ur_control_simulation", anonymous=True)

        # Init Action server for robot position control
        self._action_server = actionlib.SimpleActionServer(
            self._action_name,
            ap_movePoseAction,
            execute_cb=self.execute_cb,
            auto_start=True,
        )

        # Init ROS publisher for the FT-Axia force-torque sensor
        self._ft_sensor_pub = rospy.Publisher(
            "forces_and_torques", Wrench, queue_size=10
        )
        # Init ROS publisher for the current tool pose
        self._pose_pub = rospy.Publisher("robot_pose", PoseStamped, queue_size=10)

        # PyBullet Simulation Variables
        self.current_joint_positions = None
        self.current_tcp_pose = None
        self.ur5e_id = None
        self.JOINT_INDICES = list(range(2, 8))

        self.previous_joint_command = [0.0] * len(self.JOINT_INDICES)
        self.alpha = 0.05  # Smoothing factor: closer to 0 = more lag

        # Initialize PyBullet
        self.connect_to_pybullet()
        self.load_urdf_models()

    def connect_to_pybullet(self):
        """Setup the PyBullet simulation."""
        p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())  # Add search path
        p.setGravity(0, 0, -9.81)  # Set gravity
        p.setTimeStep(0.01)

    def load_urdf_models(self):
        """Load URDF models into the PyBullet simulation."""
        p.loadURDF("plane.urdf", basePosition=[0, 0, 0])  # Load a flat ground plane
        p.loadURDF(
            "/home/docker/ros_ws/src/ur5pybullet/urdf/cell/table.urdf",
            basePosition=[-0.5, -0.3, 0],
        )
        self.ur5e_id = p.loadURDF(
            "/home/docker/ros_ws/src/ur5pybullet/urdf/universal_robots/ur_description/urdf/ur5e_with_peg.urdf",
            basePosition=[0, 0, 0.0],
        )

    def execute_cb(self, goal):
        """ROS Action Server callback to handle incoming goals."""
        rospy.loginfo(f"Received goal: {goal}")

        # Transform the recieved position into the global frame
        goal.pose.position.x += TRANSLATION_VECTOR[0]
        goal.pose.position.y += TRANSLATION_VECTOR[1]
        goal.pose.position.z += TRANSLATION_VECTOR[2]

        # Store the requested goal
        with self._lock:
            self.requested_position = [
                goal.pose.position.x,
                goal.pose.position.y,
                goal.pose.position.z,
            ]
            self.requested_orientation = [
                goal.pose.orientation.x,
                goal.pose.orientation.y,
                goal.pose.orientation.z,
                goal.pose.orientation.w,
            ]

        # Wait for the simulation to reach the goal
        success = False
        rate = rospy.Rate(100)  # 100 Hz
        timeout = rospy.Time.now() + rospy.Duration(10)  # 10-second timeout

        while rospy.Time.now() < timeout:
            with self._lock:
                if self.is_goal_reached():
                    success = True
                    break
            rate.sleep()

        if success:
            rospy.loginfo("Goal reached successfully.")
            self._result_msg.success = True  # Populate the result message
            self._action_server.set_succeeded(self._result_msg)
        else:
            rospy.logwarn("Goal not reached within timeout.")
            self._result_msg.success = False  # Populate the result message
            self._action_server.set_aborted(self._result_msg)

    def is_goal_reached(self, tolerance=0.7):
        """Check if the current TCP pose matches the requested pose within a tolerance."""
        if self.current_tcp_pose is None or self.requested_position is None:
            return False

        position_reached = all(
            abs(c - r) <= tolerance
            for c, r in zip(self.current_tcp_pose[0], self.requested_position)
        )
        orientation_reached = all(
            abs(abs(c) - abs(r)) <= tolerance
            for c, r in zip(self.current_tcp_pose[1], self.requested_orientation)
        )

        return position_reached and orientation_reached

    def transform_tcp_to_base(self, tcp_pose_world):
        """Given the translation and quaternion rotation defined within the constants of the sim script,
        return the given TCP pose transformed into the robot base frame.

        Args:
            tcp_pose_world (tuple[list[float, float, float], list[float, float, float, float]]):
                Position (x, y, z) and orientation (qx, qy, qz, qw) of the TCP within the world frame.

        Returns:
            tuple[list[float, float, float], list[float, float, float, float]]:
                Position and orientation of the TCP within the robot base frame. Rotation is represented as a quaternion.
        """

        tcp_position_world, tcp_quaternion_world = tcp_pose_world

        # Convert world-to-base rotation from quaternion to rotation matrix
        rotation_matrix = R.from_quat(ROTATION_VECTOR).as_matrix()

        # Compute inverse rotation (quaternion conjugate -> inverse rotation matrix)
        inv_rotation_matrix = (
            rotation_matrix.T
        )  # Transpose = inverse for rotation matrices
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

        return (tcp_position_base.tolist(), tcp_quaternion_base.as_quat().tolist())

    def publish_current_wrench(self, link_state: tuple):
        """Takes the jointstate of a pybullet joint as a tuple, extracts the forces and torques and publishes them using ROS

        Args:
            link_state (tuple): Tuple representing the current state of a pybullet joint (position, velocity, reaction forces, applied motor torque)
        """
        forces_and_torques = link_state[2]

        wrench_msg = Wrench()
        wrench_msg.force.x = forces_and_torques[0]
        wrench_msg.force.y = forces_and_torques[1]
        wrench_msg.force.z = forces_and_torques[2]

        wrench_msg.torque.x = forces_and_torques[3]
        wrench_msg.torque.y = forces_and_torques[4]
        wrench_msg.torque.z = forces_and_torques[5]

        self._ft_sensor_pub.publish(wrench_msg)

    def set_robot_status(self):
        """Update the robot's current state in the simulation."""
        self.current_joint_positions = [
            state[0] for state in p.getJointStates(self.ur5e_id, self.JOINT_INDICES)
        ]
        actual_TCP_pose = p.getLinkState(self.ur5e_id, 10, computeForwardKinematics=1)[
            :2
        ]
        self.current_tcp_pose = self.transform_tcp_to_base(actual_TCP_pose)

        # Publish the current pose to the ROS network
        pose_msg = PoseStamped()
        current_time = rospy.Time.now()

        pose_msg.header.stamp.secs = current_time.secs
        pose_msg.header.stamp.nsecs = current_time.nsecs
        pose_msg.header.frame_id = "base"

        pose_msg.pose.position.x = self.current_tcp_pose[0][0]
        pose_msg.pose.position.y = self.current_tcp_pose[0][1]
        pose_msg.pose.position.z = self.current_tcp_pose[0][2]

        pose_msg.pose.orientation.x = self.current_tcp_pose[1][0]
        pose_msg.pose.orientation.y = self.current_tcp_pose[1][1]
        pose_msg.pose.orientation.z = self.current_tcp_pose[1][2]
        pose_msg.pose.orientation.w = self.current_tcp_pose[1][3]

        self._pose_pub.publish(pose_msg)

    def run_simulation(self):
        """Main simulation loop."""
        rate = rospy.Rate(100)  # 100 Hz

        # Setup the FT-sensor
        p.enableJointForceTorqueSensor(self.ur5e_id, jointIndex=9)

        # Set initial joint positions
        initial_positions = [1.57, 0, 0, 0, 0, 0]
        p.setJointMotorControlArray(
            self.ur5e_id,
            jointIndices=self.JOINT_INDICES,
            controlMode=p.POSITION_CONTROL,
            targetPositions=initial_positions,
        )

        print("Initial Positions set!")
        p.stepSimulation()

        while not rospy.is_shutdown():
            # Get the requested control values from the action server
            with self._lock:
                control_values = (self.requested_position, self.requested_orientation)

            if control_values[0] is not None and control_values[1] is not None:
                # Calculate raw IK joint targets
                joint_values = list(p.calculateInverseKinematics(
                    self.ur5e_id,
                    endEffectorLinkIndex=10,
                    targetPosition=control_values[0],
                    targetOrientation=control_values[1],
                ))

                # Apply exponential smoothing to simulate actuator delay
                smoothed_joint_values = []
                for i in range(len(self.JOINT_INDICES)):
                    new_val = self.alpha * joint_values[i] + (1 - self.alpha) * self.previous_joint_command[i]
                    smoothed_joint_values.append(new_val)

                # Update previous command
                self.previous_joint_command = smoothed_joint_values

                # Send the smoothed command to the robot
                p.setJointMotorControlArray(
                    self.ur5e_id,
                    jointIndices=self.JOINT_INDICES,
                    controlMode=p.POSITION_CONTROL,
                    targetPositions=smoothed_joint_values,
                )

            p.stepSimulation()  # Step the simulation to apply the control
            self.set_robot_status()  # Update the robot's state
            self.publish_current_wrench(p.getJointState(self.ur5e_id, 9))
            rate.sleep()

    def start(self):
        """Start the simulation and ROS action server."""
        rospy.loginfo("Starting PyBullet simulation with ROS integration.")
        self.run_simulation()


if __name__ == "__main__":
    try:
        node = Ur_PyBullet_Ros_Sim()
        node.start()
    except rospy.ROSInterruptException:
        pass
