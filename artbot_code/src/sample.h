#ifndef SAMPLE_H
#define SAMPLE_H

#include "ros/ros.h"
#include <atomic>
#include <mutex>

//ROS data types
#include "std_srvs/SetBool.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_datatypes.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseArray.h"
// #include "sensor_msgs/Image.h"
// #include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"

//We include header of another class we are developing
#include "laserprocessing.h"
#include "pathplanning.h"

/*!
 *  \brief     Sample Class
 *  \details
 *  This class is used for communicating with the simulator environment using ROS using the provided libraries.
 *  It is designed to use laserprocessing to interpret laser data and imageprocessing to interpret image data.
 *  This information is used to generate an input for the control or the Turtlebot to follow the AR tag.
 *  \author    Ashton Powell
 *  \version   1.00
 *  \date      2023-10-29
 */
class Sample
{
public:
  /// @brief Constructor of the Sample class.
  ///
  /// Sets the default values of variables such as the robot position, the goals, the running_ boolean, etc.
  /// Requires the NodeHandle input to communicate with ROS.
  Sample(ros::NodeHandle nh);

  /// @brief Destructor of the Sample class.
  ///
  /// Deletes the object pointers for laserprocessing and imageprocessing classes.
  ~Sample();
  
  /// @brief seperate thread.
  ///
  /// The main processing thread that will run continously and utilise the data.
  /// When data needs to be combined then running a thread seperate to callback will guarantee data is processed.
  /// The data is then used to publish input to move the TurtleBot, causing new data to be generated on its updated position and perspective.
  void seperateThread();

  //! Node handle for communication
  ros::NodeHandle nh_;
  //! Flag for whether the car is moving and the mission is active
  std::atomic<bool> running_;
};

#endif // SAMPLE_H