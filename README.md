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
    **Terminal 1**
    export TURTLEBOT3_MODEL=waffle
    roslaunch rs2_gazebo_world turtlebot3_marker_ver7.launch

    **Terminal 2**
    export TURTLEBOT3_MODEL=waffle
    roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=$HOME/catkin_ws/src/rs2_art_gallery_robot/examples/rs2_V3_map.yaml

    **Terminal 3**
    export TURTLEBOT3_MODEL=waffle
    roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch



## Notes before starting the sim
     Make sure to change the directory of rs2_map_V3.yaml file from:
     home\wajeeha\catkin_ws\src\rs2_art_gallery_robot\examples
     to your own directory (Replace wajeeha to <your name>)
