# OnRobot_RG2FT_ROS2_Driver
This repository is based on the OnRobot-RG2FT-ROS driver: https://github.com/ian-chuang/OnRobot-RG2FT-ROS for ROS2 Humble. It was developed with the assistance of artificial intelligence (OpenAI GPT-5) for debugging and migration.

### Main Features:

+ Communication with the OnRobot RG2FT gripper via Modbus TCP

+ Continuous publishing of state data: forces, torques, and gripper width

+ Command reception through ROS 2 topics

+ Support for services (restart, calibration, proximity offsets)

+ Other packages from the original repository (MoveIt, Description, Action Server) were excluded for simplicity for now.

## Installation
1. Start by creating your workspace and cloning the repository:

<pre>cd ~/workspace/src

git clone https://github.com/A01637071/onrobot_rg2ft_ros2_driver.git</pre>

2. Install pymodbus 2.1.0:

<pre>pip3 install pymodbus==2.1.0</pre>

3. Compile your workspace

<pre>cd ~/workspace/

colcon build

source install/setup.bash</pre>

## Running the driver

This command connects to the gripper and prints a message once the connection is established. It also receives the other commands to physically manipulate it.

<pre>ros2 run onrobot_rg2ft_control OnRobotRG2FTDriver.py</pre>

Make sure your gripper's IP is 192.168.1.1. If it is different, change it on the ```OnRobotRG2FT.py``` and the ```OnRobotRG2FTDriver.py``` file.

Now that your gripper is connected, on another terminal you can close the gripper with this command:

<pre>ros2 topic pub /gripper/command onrobot_rg2ft_msgs/msg/RG2FTCommand \
  "{target_force: 40, target_width: 0, control: 1}"</pre>

And you can open it back with: 

<pre>ros2 topic pub /gripper/command onrobot_rg2ft_msgs/msg/RG2FTCommand \
  "{target_force: 40, target_width: 1000, control: 1}"</pre>

The parameter ```target_force``` should not be lesser than 10 for the gripper to move. The maximium value available is ... Meanwhile ```target_width```goes from 0 to 1000.


