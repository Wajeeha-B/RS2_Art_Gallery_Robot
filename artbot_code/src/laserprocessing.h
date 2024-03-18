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
 *  \author    Ashton Powell, Jacinta Kuessner
 *  \version   1.00
 *  \date      31/10/23
 */

class LaserProcessing
{
public:
  /// @brief Constructor for laser processing
  /// @param [in] laserScan - laserScan to be processed
  LaserProcessing(sensor_msgs::LaserScan laserScan);

private:
    //! Stores the laser scan data
    sensor_msgs::LaserScan laserScan_;
    
};
#endif