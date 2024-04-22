#include "laserprocessing.h"
#include <algorithm>
#include <numeric>

using namespace std;

//Default constructor of Laserprocessing, needs an initial set of laser data to be initialised
LaserProcessing::LaserProcessing(sensor_msgs::LaserScan laserScan):
    laserScan_(laserScan)
    {
        
    }

// //Getter for distance and angle to the nearest obstacle
// std::pair<double, double> LaserProcessing::MinDistAngle(){
//     std::pair<double, double> distAngle;
//     distAngle.first = 0.0;
//     distAngle.second = 1.0;

//     return distAngle;
// }

// Ali's code

// Min Distance and Angle implementation
std::pair<double, double> LaserProcessing::MinDistAngle() {
    std::pair<double, double> distAngle;
    double minElement = laserScan_.ranges[0];
    int minIndex = 0;

    for (int i = 1; i < laserScan_.ranges.size(); i++) {
        if (laserScan_.ranges[i] < minElement) {
            minElement = laserScan_.ranges[i];
            minIndex = i;
        }
    }

    distAngle.first = minElement * 1000; // To convert to mm
    distAngle.second = (minIndex * laserScan_.angle_increment + laserScan_.angle_min) * 180 / M_PI; // To convert to degrees
    return distAngle;
}
// 'Pass' test implementation
std::pair<double, double> LaserProcessing::RangeAngle() {
    std::pair<double, double> rangeAngle;
    rangeAngle.first = *std::max_element(laserScan_.ranges.begin(), laserScan_.ranges.end());
    rangeAngle.second = std::distance(laserScan_.ranges.begin(), std::max_element(laserScan_.ranges.begin(), laserScan_.ranges.end())) * laserScan_.angle_increment + laserScan_.angle_min;
    return rangeAngle;
    ROS_INFO("Range: %f, Angle: %f\n", rangeAngle.first, rangeAngle.second);
}

std::pair<double, double> LaserProcessing::Coordinate() {
    std::pair<double, double> coordinate;

// This needs to change. Variables are wrong.
    double xRelative = laserScan_.ranges[0] * cos(laserScan_.angle_min);
    double yRelative = laserScan_.ranges[0] * sin(laserScan_.angle_min);
    coordinate.first = xRelative;
    coordinate.second = yRelative;
    return coordinate;

    ROS_INFO("X: %f, Y: %f\n", coordinate.first, coordinate.second);
}

// From PFMS Quiz 3
double normaliseAngle(double theta) {
      if (theta > (2 * M_PI))
        theta = theta - (2 * M_PI);
      else if (theta < 0)
        theta = theta + (2 * M_PI);

      if (theta > M_PI){
          theta = -( (2* M_PI) - theta);
      }

      return theta;
    }