#include "laserprocessing.h"
#include <algorithm>
#include <numeric>
#include <thread>


using namespace std;

//Default constructor of Laserprocessing, needs an initial set of laser data to be initialised
LaserProcessing::LaserProcessing(sensor_msgs::LaserScan laserScan):
    laserScan_(laserScan)
{
        
}


// Can add more functions!!

// //Getter for distance and angle to the nearest obstacle
// std::pair<double, double> LaserProcessing::MinDistAngle(){
//     std::pair<double, double> distAngle;
//     distAngle.first = 0.0;
//     distAngle.second = 1.0;

//     return distAngle;
// }


// Getter for distance and angle to the nearest obstacle
std::pair<double, double> LaserProcessing::MinDistAngle() {
    std::pair<double, double> distAngle;
    auto minElement = std::min_element(laserScan_.ranges.begin(), laserScan_.ranges.end());
    distAngle.first = *minElement;
    distAngle.second = std::distance(laserScan_.ranges.begin(), minElement) * laserScan_.angle_increment + laserScan_.angle_min;
    return distAngle;
}