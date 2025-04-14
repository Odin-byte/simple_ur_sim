#!/usr/bin/env python3
import rospy
import actionlib
from geometry_msgs.msg import Pose, PoseStamped, Wrench
from ros_interface_sim.msg import ap_movePoseAction, ap_movePoseGoal
from threading import Lock
from analog_api.lucidac_api import LucidacAPI

# Constant values for the simulation
QUAT_FACING_DOWN = [0.0, 1.0, 0.0, 6.123233995736766e-17]
PRE_PRESS_POS = [-0.5, -0.2, 0.05]

# Variables for the simulation
WANTED_FORCE = 5 #Newton

# Control values
M = 0.7
D = 0.5
K = 0.9

class RosUrControlClient:
    def __init__(self, api_host:str="localhost", api_port:int=5000):

        # Initialize the ROS node
        self._action_name = "move_pose"
        rospy.init_node("ros_ur_control_client", anonymous=True)

        # Create an action server to handle the movement commands sent by the action clients
        self._action_client = actionlib.SimpleActionClient(
            self._action_name, ap_movePoseAction
        )
        # Wait for the server to be ready
        self._action_client.wait_for_server()

        # Create a subcriber to the pose topic provided by the simulation
        self._robot_pose_sub = rospy.Subscriber(
            "robot_pose", PoseStamped, self.pose_callback
        )

        # Create a subscriber to the force torque sensor topic
        self._ft_sensor_sub = rospy.Subscriber(
            "forces_and_torques", Wrench, self.wrench_callback
        )

        # Store the latest values internally
        self.pose_msg = None
        self.wrench_msg = None
        self.data_lock = Lock()

        # Establish communication with the lucidac compiler API
        print("Establishing API connection!")
        self.luci_api = LucidacAPI(host=api_host, port=api_port)

    def pose_callback(self, msg):
        """Save the latest stamped pose msg to the class internal storage variable

        Args:
            msg (PoseStamped ROS msg object): ROS1 PoseStamped message object
        """
        with self.data_lock:
            self.pose_msg = msg

    def wrench_callback(self, msg):
        """Save the latest wrench msg to the class internal storage variable

        Args:
            msg (Wrench ROS msg object): ROS1 Wrench message object
        """
        with self.data_lock:
            self.wrench_msg = msg

    def send_goal(self, pos:list, quat:list):
        """Sends a ROS1 action request using the class internal action client to move the robot based on a given position and orientation.

        Args:
            pos (list): Position in relation to the robot base frame in X,Y,Z format
            quat (list): Orientation in relation to the robot base frame in X,Y,Z,W quaternion format

        Returns:
            Bool: Wether or not the requested pose was reached
        """
        # Send goal to action server!
        goal = ap_movePoseGoal()

        # Define a pose msg
        pose_msg = Pose()
        pose_msg.position.x = pos[0]
        pose_msg.position.y = pos[1]
        pose_msg.position.z = pos[2]

        pose_msg.orientation.x = quat[0]
        pose_msg.orientation.y = quat[1]
        pose_msg.orientation.z = quat[2]
        pose_msg.orientation.w = quat[3]

        goal.pose = pose_msg

        self._action_client.send_goal(goal)
        self._action_client.wait_for_result()

        return self._action_client.get_result()

    def run_sim(self):
        print("Running sim!")
        rate = rospy.Rate(500)  # 10 Hz loop rate

        # Move the robot to a pre press position using a fixed position command
        print(self.send_goal(PRE_PRESS_POS, QUAT_FACING_DOWN))
        
        while not rospy.is_shutdown():

            with self.data_lock:
                current_wrench = self.wrench_msg
                current_pose = self.pose_msg
            # Extract the Z-force from the current force values
            z_force = current_wrench.force.z
            # Extract the current z-position
            z_pos = current_pose.pose.position.z
            
            # Prepare the needed variable dict
            ode_vars = {
                "M_z" : M,
                "F_d" : WANTED_FORCE,
                "F_ext_z" : z_force,
                "D_z" : D,
                "K_z" : K,
                "Z(t: 0.0)" : z_pos,
                "diff[Z, t](t: 0.0)" : 0.0,
            }

            # Adapt the force control template ode
            json_ode = self.luci_api.adapt_json_template("force_control", ode_vars)

            # Compile the json ode
            self.luci_api.compile_json_ode(json_ode)

            # Calculate the ode for this control step
            print(self.luci_api.calculate_current_ode(1, 1))
            rate.sleep()


if __name__ == "__main__":

    try:
        node = RosUrControlClient()
        node.run_sim()
    except rospy.ROSInterruptException:
        pass
