#ifndef PATHPLANNING_H
#define PATHPLANNING_H

#include <math.h>
#include <iostream>
#include "ros/ros.h"
#include "std_msgs/Header.h"
#include <geometry_msgs/PoseStamped.h>
// #include <geometry_msgs/Twist.h>

/*!
 *  \brief     Path Planning Class
 *  \details
 *  
 *  @sa Sample
 *  \author    
 *  \version   1.00
 *  \date      31/10/23
 */

class PathPlanning
{
public:
  /// @brief Constructor for path planning
  PathPlanning(int map_width, int map_height, double map_resolution,
                           double map_origin_x, double map_origin_y, std::vector<int8_t> map_data, double world_x, double world_y);

  ~PathPlanning();

  void generateRandomGoal(std::vector<geometry_msgs::PoseStamped>& unordered_goals, geometry_msgs::Pose robotPose);

  bool isGoalValid(double x, double y, std::vector<geometry_msgs::PoseStamped>& unordered_goals, geometry_msgs::Pose robotPose);

  double DistanceToGoal(double goal_x, double goal_y, geometry_msgs::Pose robot);

private:
  int map_width_;
  int map_height_;
  double map_resolution_;
  double map_origin_x_;
  double map_origin_y_;
  std::vector<int8_t> map_data_;
  double world_x_;
  double world_y_;
  // std::vector<geometry_msgs::PoseStamped> unordered_goals_;
  std::vector<geometry_msgs::PoseStamped> ordered_goals_;
};

#endif