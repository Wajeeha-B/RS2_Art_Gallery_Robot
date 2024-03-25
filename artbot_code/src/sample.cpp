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
    nh_(nh), running_(false), laserProcessingPtr_(nullptr),
    tooClose_(false)
{
    //Subscribing to the laser sensor
    sub1_ = nh_.subscribe("/scan", 100, &Sample::laserCallback,this);
    //Publishing the driving commands
    pubDrive_ = nh.advertise<geometry_msgs::Twist>("/cmd_vel",3,false);
    //Service to enable the robot to start and stop from command line input
    service1_ = nh_.advertiseService("/mission", &Sample::request,this);
    
    //Sets the default robotPose_ to 0
    robotPose_.position.x = 0.0;
    robotPose_.position.y = 0.0;
    robotPose_.position.z = 0.0;
    robotPose_.orientation.x = 0.0;
    robotPose_.orientation.y = 0.0;
    robotPose_.orientation.z = 0.0;
    robotPose_.orientation.w = 0.0;

    //Sets the default goal to (0,0,0)
    goal_.x = 0.0;
    goal_.y = 0.0;
    goal_.z = 0.0;
}

// We delete anything that needs removing here specifically
Sample::~Sample(){
    if(laserProcessingPtr_ != nullptr){
        delete laserProcessingPtr_;
    }
}

//A callback for the laser scanner
void Sample::laserCallback(const sensor_msgs::LaserScanConstPtr& msg)
{
    std::unique_lock<std::mutex> lck(laserDataMtx_); // Locks the data for the laserData to be saved
    laserData_ = *msg; // We store a copy of the LaserScan in laserData_
}

// //A callback for odometry
// void Sample::odomCallback(const nav_msgs::OdometryConstPtr &msg)
// {
//     geometry_msgs::Pose pose = msg->pose.pose;
//     std::unique_lock<std::mutex> lck(robotPoseMtx_); // Locks the data for the robotPose to be saved
//     robotPose_ = pose; // We copy the pose here
// }

void Sample::seperateThread() {
    //Waits for the data to be populated from ROS
    while(laserData_.range_min+laserData_.range_max == 0.0 ||
          robotPose_.orientation.w+robotPose_.orientation.x+
          robotPose_.orientation.y+robotPose_.orientation.z == 0.0);

    //Limits the execution of this code to 5Hz
    ros::Rate rate_limiter(5.0);
    while (ros::ok()) {
        //Locks all of the data with mutexes
        std::unique_lock<std::mutex> lck1 (laserDataMtx_);
        std::unique_lock<std::mutex> lck2 (robotPoseMtx_);
        
        //Creates the class object and gives the data from the sensors
        LaserProcessing laserProcessing(laserData_);

        //Unlocks all mutexes
        lck2.unlock();
        lck1.unlock();

        //Gets the angle of the laser scanner using the x value of the image
        double angle = 0;

        // ROS_INFO("AngleMin= %f\n AngleMax= %f\n AngleIncrement= %f", laserData_.angle_min, laserData_.angle_max, laserData_.angle_increment);
        
        double angle_of_min_distance;
        std::pair<double, double> rangeBearing;
        rangeBearing.first = 0.0;
        rangeBearing.second = 0.0;
        // rangeBearing = laserProcessing.MinDistAngle(STOP_DISTANCE_);

        //Gets the distance from the angle
        double dist = 0;
        // dist = laserProcessing.FindDistance(angle);

        //If the distance is less than the stop distance or more than the max value of an int (an invalid reading) the robot should stop
        if(dist < STOP_DISTANCE_ || dist > 2147483647 || rangeBearing.first < STOP_DISTANCE_) tooClose_ = true;
        //Otherwise the robot is not too close
        else tooClose_ = false;

        //Creates the variable for driving the TurtleBot
        geometry_msgs::Twist drive;
        if(running_ && !tooClose_){
            drive.linear.x = 0.1; //sends it forward
            drive.linear.y = 0.0;
            drive.linear.z = 0.0;
            drive.angular.x = 0.0;
            drive.angular.y = 0.0;
            if (angle > 0.001 || angle < -0.001) drive.angular.z = angle; //sends the angle of turn required
            else drive.angular.z = 0.0;
            ROS_INFO("TurtleBot is going forwards");
        }
        //Stops the TurtleBot
        else{
            if (rangeBearing.first < STOP_DISTANCE_) {
                drive.linear.x = 0.0;  // Stop any forward motion
                drive.linear.y = 0.0;
                drive.linear.z = 0.0;

                // Determine the rotation direction based on the angle of the minimum distance
                if (rangeBearing.second >= 0 && rangeBearing.second <= 90) {
                    // Turn 90 degrees to the right
                    drive.angular.z = -M_PI / 2;
                }
                else if (rangeBearing.second >= 270 && rangeBearing.second <= 359) {
                    // Turn 90 degrees to the left
                    drive.angular.z = M_PI / 2;
                }
            }
            else {
                drive.linear.x = 0.0;
                drive.linear.y = 0.0;
                drive.linear.z = 0.0;
                drive.angular.x = 0.0;
                drive.angular.y = 0.0;
                drive.angular.z = 0.0;
            }
            ROS_INFO("TurtleBot is stopped");
        }

        //Publishes the drive variable to control the TurtleBot
        pubDrive_.publish(drive);

        //We have a rate timer, this sleep here is needed to ensure it stops and sleeps 
        //it will do it for the exact amount of time needed to run at 5Hz
        rate_limiter.sleep();
    }
}

//Service that handles starting and stopping the missions based on command line input
//Communicate with this service using 
//rosservice call /mission "data: true"
bool Sample::request(std_srvs::SetBool::Request  &req,
                     std_srvs::SetBool::Response &res)
{  
    //If the request is true, start the mission
    if(req.data)
    {
        ROS_INFO_STREAM("Requested: Start mission");
        running_ = true; //start the robot if there is a goal
        res.success = true;
        res.message = "The Turtlebot has started it's mission";

    }
    //if the request is false, stop the mission
    else
    {
        ROS_INFO_STREAM("Requested: Stop mission");
        res.success = true;
        res.message = "Turtlebot stopping";
        running_ = false;
    }
    //return true when the service completes its request
    return true;
}

//Gets the distance from the goal to the robot
double Sample::DistanceToGoal(geometry_msgs::Point goal, geometry_msgs::Pose robot)
{
    //finds the difference in x and y and get the hypotenuse between the two points
    double dist = sqrt(pow(goal.x-robot.position.x,2)+pow(goal.y-robot.position.y,2));
    return dist;
}

//Gets the distance from two goals
double Sample::DistanceBetweenGoals(geometry_msgs::Point goal1, geometry_msgs::Point goal2)
{
    //finds the difference in x and y and get the hypotenuse between the two points
    double dist = sqrt(pow(goal1.x-goal2.x,2)+pow(goal1.y-goal2.y,2));
    return dist;
}

//Gets the steering value
double Sample::GetSteering(geometry_msgs::Point goal, geometry_msgs::Pose robot)
{
    /*-----Chord Length-----*/
    double dx = goal.x-robot.position.x;    //Difference in x between the robot's current position and the goal
    double dy = goal.y-robot.position.y;    //Difference in y between the robot's current position and the goal
    double CL = sqrt(pow(dx,2)+pow(dy,2));  //Chord length is hypotenuse of perpendicular lengths of dx and dy

    /*-----Alpha Angle-----*/
    double AA = atan2(dy, dx)-tf::getYaw(robot.orientation);  //Alpha angle made with angle formed from chord from the yaw angle of the robot
    return AA;
}