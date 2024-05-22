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

  ~LaserProcessing();

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



/* Added 11/05/2024 */

  /// @brief Count number of readings belonging to objects
  /// @return the number of laser readings that are NOT at infinity, nan or max range
  unsigned int countObjectReadings();

  /// @brief Count number of high intensity segments
  /// @return the number of segments in the current laser scan
  unsigned int countSegments();

private:
    //! Stores the laser scan data
    sensor_msgs::LaserScan laserScan_;

    /* Added 11/05/2024 */

    /// @brief Returns the cartesian position of laser reading at specific index
    /// @param[in] index - index of the laser reading
    /// @return position cartesian values
    std::pair<double, double> polarToCart(int index);
    
    /// @brief Given two points (only x,y are used), returns the slope slope of the lines connecting them
    /// @param[in] p1 - first point
    /// @param[in] p2 - second point
    /// @return the slope of the line connecting the two points
    double angleConnectingPoints(std::pair<double, double> p1, std::pair<double, double> p2);
};
#endif