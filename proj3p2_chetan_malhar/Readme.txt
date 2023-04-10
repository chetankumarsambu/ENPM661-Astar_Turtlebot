ENPM661 - Project 3 - Phase2

Team:
Name: Chetan Kumar
UID: 119112366
DirectoryID: chetan27

Name: Atre Malhar
UID: 119398258
DirectoryID: atre217

Dependencies:
1) Python 3
2) Numpy
3) OpenCV
4) geometry_msgs
5) roslaunch
6) ROS - Noetic(Ran on WSL)
7) nav_msgs


Build: 

cd into project directory
cd <project_directory>/part02

add the part02 to your catkin workspace
cd workspace/src
cp <project_directory>/part_2 workspace/src/.

Rename part02 to "project_3_phase_2_chetan_malhar"

set turtlebot3 to burger
export TURTLEBOT3_MODEL="BURGER"

build catkin workspace
cd ../
catkin_make

source the workspace
source ./devel/setup.bash

Make python script executable
chmod +x src/turtlebot_gazebo.py

launch the file
roslaunch project_3_phase_2_chetan_malhar final.launch

Enter x and y coordinates of the goal after all the rospy logs are finished

Videos:

Part 01: https://drive.google.com/drive/u/0/folders/17FiWeH-oVz65rQemPfE6BarKv6YdlZhE

  - Start Position: (50, 100, 0)
  - Goal Position: (550, 100)
  - Clearance: 5
  - RPM1: 5
  - RPM2: 6

Part 02: https://drive.google.com/drive/u/0/folders/17FiWeH-oVz65rQemPfE6BarKv6YdlZhE

  - Start Position: (50, 100, 0) 
  - Goal Position: (550, 100)
  - Clearance: 15

Github Repository: https://github.com/chetankumarsambu/ENPM661-Astar_Turtlebot

