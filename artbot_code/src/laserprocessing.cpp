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

std::pair<double, double> LaserProcessing::MinDistAngle() {
    // This pair will hold the distance and angle to the nearest obstacle
    std::pair<double, double> distAngle;

    // This will hold the minimum element in the ranges
    double minElement = laserScan_.ranges[0];

    // This will hold the index of the minimum element
    int minIndex = 0;

// Iterate through the ranges to find the minimum element
    for (int i = 1; i < laserScan_.ranges.size(); i++) {
        if (laserScan_.ranges[i] < minElement) {
            minElement = laserScan_.ranges[i];
            minIndex = i;
        }
    }

// Assign the minimum distance and angle to the pair
    distAngle.first = minElement * 1000; // To convert to mm
    distAngle.second = (minIndex * laserScan_.angle_increment + laserScan_.angle_min) * (180.0 / M_PI); // To convert to degrees
    return distAngle;
}


unsigned int LaserProcessing::countObjectReadings()
{
    // This count will hold the number of object readings
  unsigned int count=0;
    // iterate through the ranges
      for (const auto& range : laserScan_.ranges)
    {
        // if the range is not infinity, not nan, and not the max range
        if (!std::isinf(range) && !std::isnan(range) && range != laserScan_.range_max)
        {
            // increment the count
            count++;
        }
    }
    // return the count
  return count;
}

unsigned int LaserProcessing::countSegments()
{
    // This count will hold the number of segments
    unsigned int count=0;
    // Check if we are in a segment - initialised to false.
    bool inSegment = false;
    // Iterating over all ranges
    for (const auto& range : laserScan_.ranges)
    {
        std::pair<double, double> p1 = polarToCart(range - 1); //Previous point
        std::pair<double, double> p2 = polarToCart(range); // Current point

        // Calculate the Euclidean distance between the two points
        double distance = sqrt(pow(p2.first - p1.first, 2) + pow(p2.second - p1.second, 2));
        
        // If the distance is less than 0.3, we are in the same segment
        if (distance < 0.3)
        {
            // Change initialised inSegment to true since we're in the same segment (i.e., the distance is less than 0.3).
            if (!inSegment)
            {
                inSegment = true;
                count++;
            }
        }
        // If the distance is greater than 0.3, we are not in the same segment
        else
        {
            inSegment = false;
        }
    // return the number of segments.
    return count;
    }
}

std::pair<double, double> LaserProcessing::polarToCart(int index)
{
    // This pair will hold the cartesian position of the laser reading at the specific index
    std::pair<double, double> point;
    // Calculate the x and y coordinates of the laser reading at the specific index
    point.first = laserScan_.ranges[index] * cos(index * laserScan_.angle_increment + laserScan_.angle_min);
    point.second = laserScan_.ranges[index] * sin(index * laserScan_.angle_increment + laserScan_.angle_min);
    return point;
}

double LaserProcessing::angleConnectingPoints(std::pair<double, double> p1, std::pair<double, double> p2)
{
    // Return the angle between the two points
    return atan2(p2.second - p1.second, p2.first - p1.first);
}