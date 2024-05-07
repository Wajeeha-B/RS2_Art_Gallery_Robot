#include <ros/ros.h>
#include "sample.h"
#include "laserprocessing.h"
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <iostream>
#include <random>
#include <thread>


int main(int argc, char** argv) {
   // calling node "a3_skeleton", ross::init() needs two arguments to perform any ROS argument.
  ros::init(argc, argv, "waypoints_producer");

  // Nodehandle is main access point to communications
  ros::NodeHandle nh;

  // create an object of type Sample and pass it a node handle; creating sample class
  std::shared_ptr<Sample> sample(new Sample(nh));

  // set and reach goals here
  // creating seperate thread to shared_ptr sample, calling seperateThread
  std::thread t1(&Sample::generateRandomGoals,sample);
  std::this_thread::sleep_for(std::chrono::milliseconds(500)); // wait for connection 
  std::thread t2(&Sample::seperateThread,sample);

  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
  ros::spin();

  // cleanup everything, shutdown ros and join the thread
  ros::shutdown();

  // wait for threads
  t1.join();
  t2.join();

  return 0;
}
