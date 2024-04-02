#include "laserprocessing.h"
#include <algorithm>
#include <numeric>

using namespace std;

//Default constructor of Laserprocessing, needs an initial set of laser data to be initialised
LaserProcessing::LaserProcessing(sensor_msgs::LaserScan laserScan):
    laserScan_(laserScan){
        
    }

//Getter for distance and angle to the nearest obstacle
std::pair<double, double> LaserProcessing::MinDistAngle(){
    std::pair<double, double> distAngle;
    distAngle.first = 0.0;
    distAngle.second = 0.0;

    return distAngle
}