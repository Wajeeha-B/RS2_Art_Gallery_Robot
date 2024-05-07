# RS2_Art_Gallery_Robot
Rover Project for Robotics Studio 1
## Naming structure
rs2_<packagename>
e.g. rs2_gazebo
     rs2_description

## Cloning the Repository
    cd ~/catkin_ws/src
    git clone git@github.com:Wajeeha-B/rs2_art_gallery_robot.git
    cd ~/catkin_ws
    catkin_make

## Launching the Simulation
### Terminal 1
    export TURTLEBOT3_MODEL=waffle
    roslaunch rs2_gazebo_world turtlebot3_marker_ver7.launch

### Terminal 2
    export TURTLEBOT3_MODEL=waffle
    roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=$HOME/catkin_ws/src/rs2_art_gallery_robot/examples/rs2_V3_map.yaml

### Terminal 3
    export TURTLEBOT3_MODEL=waffle
    roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch

## Running the Program
(This cannot be run at the same time as teleop)
### Terminal 3
     rosrun artbot_code artbot_code_node

### Terminal 4
     rosservice call /mission "data: true"
     
## Recording and Opening a bag:
**Open the simulation(gazebo)**
### Terminal 1 (Recording the bag)
     cd  ~/catkin_ws/src/rs2_art_gallery_robot/artbot_code/src/bags
     rosbag record /scan -l 10
**Close the simulation**
### Terminal 1 (Open Roscore)
     roscore
     
### Terminal 2 (Open the bag)
     cd  ~/catkin_ws/src/rs2_art_gallery_robot/artbot_code/src/bags
     rosbag play -r 0.1 --clock -l NAME_OF_BAG.bag

## Useful Commands
### Reset the Gazebo World to its initial state
     rosservice call /gazebo/reset_world "{}"
     
## Edits before starting the sim (Everyone)
     Make sure to change the directory of rs2_map_V3.yaml file from:
     home/wajeeha/catkin_ws/src/rs2_art_gallery_robot/examples
     to your own directory (Replace wajeeha to <your name>)

## Gmapping command
     Run this command alongside the TERMINAL 1 and TERMINAL 3 commands from 'Launching the Simulation'
     roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=gmapping
     
# Path Planning
	catkin_make
### Terminal 1
	roslaunch rs2_gazebo_world turtlebot3_marker_ver7.launch
### Terminal 2
	roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=$HOME/catkin_ws/src/rs2_art_gallery_robot/examples/rs2_V3_map.yaml
### Terminal 3 - preferably detach window so you can see waypoints and command outputs
	rostopic echo /thepath
### Terminal 4 
     rosrun subsystem_ppintg subsystem_ppintg_test
Once the above node is run, waypoints in the form of geometry_msgs::Point are published to the rostopic '/thepath'.  
