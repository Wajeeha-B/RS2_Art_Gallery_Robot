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
 *  \date      2024-XX-XX
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

  /// @brief request service callback for starting and stopping the mission and Turtlebot's movement.
  ///
  /// @param [in] req The request, a boolean value where true means the mission is in progress and false stops the mission.
  /// @param [in] res The response, a boolean and string value indicating if starting the mission was successful.
  ///
  /// @return bool - Will return true to indicate the request succeeded.
  bool request(std_srvs::SetBool::Request  &req,
               std_srvs::SetBool::Response &res);
  
  /// @brief Getter for distance to be travelled to reach goal, updates as the platform moves to current goal.
  ///
  /// @param [in] goal a geometry_msgs::Point that is the current goal in x,y,z for the Turtlebot.
  /// @param [in] robot a geometry_msgs::Pose that is the current position and orientation of the Turtlebot.
  ///
  /// @return distance of the goal to the Turtlebot in a straight line [m].
  double DistanceToGoal(geometry_msgs::Point goal, geometry_msgs::Pose robot);

  /// @brief Getter for distance between consecutive goals
  ///
  /// @param [in] goal1 a geometry_msgs::Point that is the first goal in x,y,z for the Turtlebot.
  /// @param [in] goal2 a geometry_msgs::Point that is the second goal in x,y,z for the Turtlebot.
  ///
  /// @return distance between consecutive goals in a straight line [m].
  double DistanceBetweenGoals(geometry_msgs::Point goal1, geometry_msgs::Point goal2);

  /// @brief Gets the angle of the required turn to follow a straight line towards the next goal.
  ///
  /// @param [in] goal a geometry_msgs::Point that is the current goal in x,y,z for the TurtleBot.
  /// @param [in] robot a geometry_msgs::Pose that is the current position and orientation of the TurtleBot.
  ///
  /// @return a double value for the angle of the required turn of the TurtleBot [rad].
  double GetGoalAngle(geometry_msgs::Point goal, geometry_msgs::Pose robot);

  double fabs(double x);

  /// @brief Laser Callback from the laser sensor's reference
  ///
  /// @param [in|out] msg sensor_msgs::LaserScanConstPtr - the laser scan data
  /// @note This function and the declaration are ROS specific
  void laserCallback(const sensor_msgs::LaserScanConstPtr& msg);

  /// @brief Odometry Callback from the world reference of the TurtleBot
  ///
  /// @param [in|out] msg nav_msgs::OdometryConstPtr - The odometry message
  /// @note This function and the declaration are ROS specific
  void odomCallback(const nav_msgs::OdometryConstPtr& msg);
  
private:
  //! Node handle for communication
  ros::NodeHandle nh_;
  //! Driving command publisher
  ros::Publisher pubDrive_;
  //! Laser scan subscriber, uses LaserCallback
  ros::Subscriber sub1_;
  //! Robot odometry subscriber, uses OdomCallback
  ros::Subscriber sub2_;
  //! Mission service, starts and stops the mission
  ros::ServiceServer service1_;
  
  //! Pointer to Laser Object
  LaserProcessing* laserProcessingPtr_;

  //! Stores the laser data from the LIDAR scanner
  sensor_msgs::LaserScan laserData_;
  //! Mutex to lock laserData_
  std::mutex laserDataMtx_;
  //! Stores the position and orientation of the robot
  geometry_msgs::Pose robotPose_;
  //! Mutex to lock robotPose_
  std::mutex robotPoseMtx_;

  //! Flag for whether the car is moving and the mission is active
  std::atomic<bool> running_;
  //! Flag to prevent the terminal from being flooded with messages
  bool stateChange_;
  //! Stores a goal for the robot to move towards
  geometry_msgs::Point goal_;
  //! Stores a series of goals in the order they occur
  std::vector<geometry_msgs::Point> goals_;
  //! The offset between the reference of the TurtleBot and the reference of the laser scanner
  double SENSOR_OFFSET_ = 0.12;
  //! The stop distance to stop the following TurtleBot before it collides with the guiding TurtleBot
  double STOP_DISTANCE_ = 0.3;
  //! Boolean for stopping the TurtleBot when it becomes too close to the guiding TurtleBot
  double STEERING_SENS_ = 1.0;
  bool tooClose_;

  int trajMode_ = 0;
};

#endif // SAMPLE_H