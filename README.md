# Short Explanation:
1. The file "conditional_pybullet_replay_interpolation_Teleoperation_position" sends the desired position to the robot
2. The file "conditional_pybullet_replay_interpolation_Teleoperation_velocity" sends the desired velocity to the robot. Please make sure the output(return value) is correct for safety.
3. The file "conditional_pybullet_replay_interpolation_ROS_version_for_gazebo" is used for simulation in gazebo.
4. The file "ground_Truth" contains the ground truth data.
  
The code for master side can be found in repo: Teleoperation_System_Master_Alto

To start the whole simulation, run: 1. Master 2. Slave 3.Model

For Master:
```bash
cd to your workspace
source the workspace
roslaunch teleop_panda_controller teleop_panda_controller.launch leader_ip:=192.168.3.108
```

For Slave:
```bash
cd to your workspace
source the workspace
roslaunch teleop_panda_controller teleop_panda_controller.launch follower_ip:=192.168.3.107
```

For Model:

Direct with vscode

To run it in gazebo, please check the repo: Code_for_Finland
