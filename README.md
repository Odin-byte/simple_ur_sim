# Simple UR Sim (Pybullet based UR5e simulation)
This minimal simulation of a UR5e robot by Universal Robot allows the user to control a UR 5e robot using the [RTDE interface](https://www.universal-robots.com/articles/ur/interface-communication/real-time-data-exchange-rtde-guide/) provided by UR.
In the given configuration the robot is controlled by providing joint states. By changing a few lines of code the robot can also be controlled within the cartesian space. Additionally the robot is equipped with a Schunk FT-Axia-80 sensor. The messured forces and torques at the tool joint can also be read out over another TCP socket.

### RTDE Interface
Currently the simulation expects the joint position values as controll values provided by a controller connected using the RTDE protocol.
The joint position values are stored within the "double_input_registers" 0 to 5 for all 6 joints.
After the connection is established the simulated RTDE server/robot returns the current TCP pose and joint positions as a regular feedback. The feedback is sent in the feedback frequency the controller has requested when opening the connection. An additional feedback msg is sent every time a new control command is recieved.\
An example for a RTDE Client can be found here: https://github.com/Odin-byte/RTDE-URx-Bullet-Sim

### Schunk FT-Axia-80
Analog to the RTDE Interface, another TCP socket is simulating the hardware interface for the Schunk FT-Axia-80 sensor. The simulated forces and torques (N and Nm) are send over TCP protocol as specified within the hardware documentation of the sensor. A python based client implementation of the protocol can be found at https://github.com/IRAS-HKA/ros_ft_driver. 

## Usage
The entire simulation is dockerbased and runs uses Ubuntu 20.04. Apart from the URDF format, which is used to describe the structure of the robot no modules or functionalities of [ROS](https://www.ros.org/) are used. The container uses the localhost to enable the needed TCP communication.\
**CAUTION: Due to the minimal setup the robot / simulation does not check for collision with itself (tool) or the environment (table). If you provide new positions, ensure that the path and the endposition are free of unwanted collisions!**
To run the simulation clone this repo and execute the "build.sh" script found within the base directory.
```shell
./build.sh
```
After the container has been build simply run it using the provided "run.sh" script.
```shell
./run.sh
```
This scripts starts the container, mounts the needed directories and ensures X11 forwarding for the pybullet GUI.
Within the docker you can either run:
```shell
python3 src/robot_cell_sim_scripts/ur5e_peg_sensor_RTDE_control.py
```
This scripts setups a simple pybullet simulation of a UR 5e and opens two TCP sockets.\
The sockets use ports 30004 and 49151 for the RTDE and Schunk FT-Axia-80 communication.\
This simulations relies on external control input using the RTDE interface.

If you just want to test against the FT sensor run:
```shell
python3 src/robot_cell_sim_scripts/ur5e_peg_sensor_fixed_positions.py
```
This scripts setups a simple pybullet simulation of a UR 5e and opens one TCP socket.\
The socket uses port 49151 for the Schunk FT-Axia-80 communication.\
The robot loops between a predefined set of poses. It cycles between a direct push down position, creating mainly forces within the Z-axis of the sensor and two positions pushing against the edges of the table resulting in mainly torques around the X- and Y-axis of the sensor.

## Troubleshooting
In case the X11 forwarding does not work and no display can be opened make sure to allow X11 forwarding for docker by running:
```shell
xhost +local:docker
```



