# Using Particle filter / Monte Carlo Localisation

## For TurtleBot3

### Grid Mapping:

### Edit parameters:
```Bash
gedit ~/catkin_ws/src/turtlebot3/turtlebot3_slam/config/gmapping_params.yaml
```
#### Replace with:
```Bash
map_update_interval: 3.0
maxUrange: 5.0
sigma: 0.05
kernelSize: 1
lstep: 0.05
astep: 0.05
iterations: 10
lsigma: 0.075
ogain: 3.0
lskip: 0
minimumScore: 50
srr: 0.1
srt: 0.2
str: 0.1
stt: 0.2
linearUpdate: 0.5
angularUpdate: 0.2
temporalUpdate: 1.0
resampleThreshold: 0.5
particles: 200
xmin: -5.0
ymin: -7.5
xmax: 5.0
ymax: 7.5
delta: 0.05
llsamplerange: 0.01
llsamplestep: 0.01
lasamplerange: 0.005
lasamplestep: 0.005
```

### Edit launch file configuration:
```Bash
gedit ~/catkin_ws/src/turtlebot3/turtlebot3_navigation/launch/amcl.launch
```

#### Replace with (subscribing to /noisy_odom and):
```
<param name="odom_frame_id"             value="noisy_odom"/>
```

#### Terminal:
```Bash
export TURTLEBOT3_MODEL=waffle
roslaunch rs2_gazebo_world turtlebot3_marker_ver7.launch

export TURTLEBOT3_MODEL=waffle
roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=gmapping

export TURTLEBOT3_MODEL=waffle
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch

rosrun map_server map_saver -f ~/map_name

```

### Using Particle Filter:
```Bash
export TURTLEBOT3_MODEL=waffle
roslaunch rs_gazebo_world turtlebot3_marker_V2.launch

export TURTLEBOT3_MODEL=waffle
roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=$HOME/catkin_ws/src/rs2_art_gallery_robot/examples/rs2_V3_map.yaml

export TURTLEBOT3_MODEL=waffle
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
```

#
# installing kalman filter (robot_pose_ekf)

### Clone robot_pose_ekf library:
```Bash
cd ~/catkin_ws/src
git clone https://github.com/ros-planning/robot_pose_ekf.git

cd ~/catkin_Ws
catkin_make
source devel/setup.bash
```

### Edit launch file configuration:
```Bash
gedit ~/catkin_ws/src/robot_pose_ekf/robot_pose_ekf.launch
```

#### Replace with (subscribing to /noisy_odom and /imu):
```Bash
<launch>

<node pkg="robot_pose_ekf" type="robot_pose_ekf" name="robot_pose_ekf">
  <param name="output_frame" value="odom_combined"/>
  <param name="base_footprint_frame" value="base_footprint_ekf"/>
  <param name="freq" value="30.0"/>
  <param name="sensor_timeout" value="1.0"/>  
  <param name="odom_used" value="true"/>
  <param name="imu_used" value="true"/>
  <param name="vo_used" value="false"/>


  <remap from="odom" to="/noisy_odom" />
  <remap from="imu" to="/imu" />
</node>

</launch>
```

### Test:
```Bash
export TURTLEBOT3_MODEL=waffle
roslaunch turtlebot3_gazebo turtlebot3_world.launch

export TURTLEBOT3_MODEL=waffle
roslaunch turtlebot3_gazebo turtlebot3_gazebo_rviz.launch

rosrun rs_odom_noise rs_odom_noise_create_noise

roslaunch robot_pose_ekf robot_pose_ekf.launch

rostopic echo /robot_pose_ekf/odom_combined
```
#
# installing kalman filter (robot_localization)
```Bash
cd ros_ws/src
git clone https://github.com/cra-ros-pkg/robot_localization.git --branch noetic-devel
cd ros_ws/
rosdep install --from-paths src --ignore-src -r -y
catkin_make -DCMAKE_BUILD_TYPE=Release
```
## Copy these files to the launch and params folders of the `robot_localization` library:
---
[config file](./robot_localization_library_configs/rs_ekf.yaml)
---
[launch file](./robot_localization_library_configs/rs_ekf.launch)
---


# References:
[Grid mapping Navigation Tuning](https://kaiyuzheng.me/documents/navguide.pdf)<br>
[Parameter Optimization Analysis of Gmapping](https://iopscience.iop.org/article/10.1088/1742-6596/1646/1/012004/pdf)


