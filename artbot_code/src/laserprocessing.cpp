#include "laserprocessing.h"
#include <algorithm>
#include <numeric>

using namespace std;

//Default constructor of Laserprocessing, needs an initial set of laser data to be initialised
LaserProcessing::LaserProcessing(sensor_msgs::LaserScan laserScan):
    laserScan_(laserScan){
        
    }

// //Getter for distance and angle to the nearest obstacle
// std::pair<double, double> LaserProcessing::MinDistAngle(){
//     std::pair<double, double> distAngle;
//     distAngle.first = 0.0;
//     distAngle.second = 1.0;

//     return distAngle;
// }

// Ali's code

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

    distAngle.first = minElement;
    distAngle.second = minIndex * laserScan_.angle_increment + laserScan_.angle_min;
    return distAngle;
}

std::pair<double, double> LaserProcessing::RangeAngle() {
    std::pair<double, double> rangeAngle;
    rangeAngle.first = *std::max_element(laserScan_.ranges.begin(), laserScan_.ranges.end());
    rangeAngle.second = std::distance(laserScan_.ranges.begin(), std::max_element(laserScan_.ranges.begin(), laserScan_.ranges.end())) * laserScan_.angle_increment + laserScan_.angle_min;
    return rangeAngle;
}