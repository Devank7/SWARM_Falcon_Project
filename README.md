# SWARM_Falcon_Project
This repository consists of the workspace files for ROS2 (humble) simulation of team Falcon for SWARM Robotics project on Decentralized Swarming of Robots as Predator-Prey simulation. 

TEAM FALCON SWARMS
PREY-PREDATOR SWARM
HOW TO RUN OUR FILE.


Go to ROS2, I have already given the file of Workspace. Try importing it and run it directly.
After importing, Go to SRC->turtle_formation->Launch->swarm_predator_prey.launch.py. 

OR
Create a new workspace, create the same files which we submitted. Make sure you have a correct setup.py and package.xml file as there are rclpy dependencies inside it. Or else directly copy paste our submitted setup.py or package.xml file.

There are 4 code files.
1.	Prey_node.py
2.	Predator_swarm.py
3.	Spawn_turtles.py
4.	swarm_predator_prey.launch.py.

After doing all the steps:

Type this in terminal:
1) cd ~/ros2_swarm_ws
2) colcon build --packages-select turtle_formation --symlink-install
3) source install/setup.bash
4) ros2 launch turtle_formation swarm_predator_prey.launch.py
