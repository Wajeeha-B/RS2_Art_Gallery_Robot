#include "laserprocessing.h"
#include <algorithm>
#include <numeric>
#include <math.h>

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
    distAngle.second = (minIndex * laserScan_.angle_increment + laserScan_.angle_min) * (180.0 / M_PI); // To convert to degrees
    return distAngle;
}