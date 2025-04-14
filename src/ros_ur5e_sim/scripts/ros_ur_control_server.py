#!/usr/bin/env python3
import rospy
from threading import Lock
import actionlib
from geometry_msgs.msg import Pose
from ros_interface_sim.msg import ap_movePoseAction, ap_movePoseResult

import pybullet as p
import numpy as np
from scipy.spatial.transform import Rotation as R
import time
import pybullet_data


class Ur_PyBullet_Ros_Sim:
    def __init__(self):
        self.requested_position = None
        self.requested_orientation = None
        self._lock = Lock()  # Thread-safe lock for accessing position and orientation

        self._feedback_pose = Pose()
        self._result_msg = ap_movePoseResult()

        self._action_name = "move_pose"
        rospy.init_node("ros_ur_control_server", anonymous=True)

        self._action_server = actionlib.SimpleActionServer(
            self._action_name, ap_movePoseAction, execute_cb=self.execute_cb, auto_start=True
        )

        # Variables to track the robot's state
        self.current_joint_positions = None
        self.current_tcp_pose = None

    def execute_cb(self, goal):
        rospy.loginfo(f"Received goal: {goal}")

        # Store the requested goal
        with self._lock:
            self.requested_position = [
                goal.pose.position.x, goal.pose.position.y, goal.pose.position.z
            ]
            self.requested_orientation = [
                goal.pose.orientation.x, goal.pose.orientation.y, goal.pose.orientation.z, goal.pose.orientation.w
            ]

        # Wait for the simulation to reach the goal
        success = False
        rate = rospy.Rate(10)  # 10 Hz
        timeout = rospy.Time.now() + rospy.Duration(10)  # 10-second timeout

        while rospy.Time.now() < timeout:
            with self._lock:
                if self.is_goal_reached():
                    success = True
                    break
            rate.sleep()

        if success:
            rospy.loginfo("Goal reached successfully.")
            self._action_server.set_succeeded(self._result_msg)
        else:
            rospy.logwarn("Goal not reached within timeout.")
            self._action_server.set_aborted(self._result_msg)

    def is_goal_reached(self, tolerance=0.01):
        """Check if the current TCP pose matches the requested pose within a tolerance."""
        if self.current_tcp_pose is None or self.requested_position is None:
            return False

        position_reached = all(
            abs(c - r) <= tolerance
            for c, r in zip(self.current_tcp_pose[0], self.requested_position)
        )
        orientation_reached = all(
            abs(c - r) <= tolerance
            for c, r in zip(self.current_tcp_pose[1], self.requested_orientation)
        )

        return position_reached and orientation_reached

    def set_robot_status(self, joint_positions, tcp_pose):
        """Update the robot's current state."""
        with self._lock:
            self.current_joint_positions = joint_positions
            self.current_tcp_pose = tcp_pose

    def start(self):
        
        print("Got it!")



if __name__ == "__main__":
    try:
        node = RosUrControlServer()
        node.start()
    except rospy.ROSInterruptException:
        pass
