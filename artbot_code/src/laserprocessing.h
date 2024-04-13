#ifndef LASERPROCESSING_H
#define LASERPROCESSING_H

#include <sensor_msgs/LaserScan.h>
#include <math.h>
#include <iostream>
#include "ros/ros.h"
#include <geometry_msgs/Twist.h>

/*!
 *  \brief     Laser Processing Class
 *  \details
 *  This class is designed to process laser data obtained from an LDS-01 sensor mounted 
 *  on a Turtlebot3 Waffle robot. The sensor captures readings in 360-degree increments 
 *  with a precision of 1 degree. The class provides counting valid readings, identifying 
 *  cone segments, finding distances based on angles, calculating turns, and determining 
 *  the magnitude of turns.
 *  @sa Sample
 *  \author    
 *  \version   1.00
 *  \date      XX/XX/24
 */

class LaserProcessing
{
public:
  /// @brief Constructor for laser processing
  /// @param [in] laserScan - laserScan to be processed
  LaserProcessing(sensor_msgs::LaserScan laserScan);

// 'Pass' test declaration
  /// @brief Getter for distance and angle from default pose
  ///
  /// @return a pair for the doubles distance and the corresponding angle
  std::pair<double, double> RangeAngle();


  /// @brief Getter for distance and angle to the nearest obstacle
  ///
  /// @return a pair for the doubles distance and the corresponding angle
  std::pair<double, double> MinDistAngle();


  /// @brief Getter for coordinates
  ///
  /// @return a pair for the double x.coordinate and the corresponding y.coordinate
  std::pair<double, double> Coordinate();

  /// @brief Normalises angle between -PI to PI, can only handle angles between -2PI to 4PI
  /// @param angle
  /// @return normalised angle
  double normaliseAngle(double theta);



private:
    //! Stores the laser scan data
    sensor_msgs::LaserScan laserScan_;
    
};
#endif