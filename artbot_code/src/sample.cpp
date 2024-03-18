#include "sample.h"
#include <iostream>
#include <cmath>
#include <thread>
#include <chrono>
#include <time.h>

using std::cout;
using std::endl;

//Default constructor of the sample class
Sample::Sample(ros::NodeHandle nh) :
    //Setting the default value for some variables
    nh_(nh), running_(false)
{
    
}

// We delete anything that needs removing here specifically
Sample::~Sample(){

}

void Sample::seperateThread() {
    
}