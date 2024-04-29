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
    nh_(nh), running_(false), real_(true), laserProcessingPtr_(nullptr),
    tooClose_(false), stateChange_(true)
{
    //Subscribing to the laser sensor
    sub1_ = nh_.subscribe("/scan", 100, &Sample::laserCallback,this);
    //Subscribing to odometry of the robot
    sub2_ = nh_.subscribe("/amcl_pose", 100, &Sample::amclCallback,this);
    //Publishing the driving commands
    pubDrive_ = nh.advertise<geometry_msgs::Twist>("/cmd_vel",3,false);
    //Service to enable the robot to start and stop from command line input
    service1_ = nh_.advertiseService("/mission", &Sample::request,this);

    service2_ = nh_.advertiseService("/real", &Sample::real,this);
    
    //Sets the default robotPose_ to 0
    robotPose_.position.x = 0.0;
    robotPose_.position.y = 0.0;
    robotPose_.position.z = 0.0;
    robotPose_.orientation.x = 0.0;
    robotPose_.orientation.y = 0.0;
    robotPose_.orientation.z = 0.0;
    robotPose_.orientation.w = 0.0;

    //Sets the default goal to (0,0,0)
    goal_.x = DBL_MAX_;
    goal_.y = DBL_MAX_;
    goal_.z = DBL_MAX_;
    // ROS_INFO_STREAM("Goal is reset!");

    goalIdx_ = 0;
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

void Sample::amclCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg)
{
    geometry_msgs::Pose pose = msg->pose.pose;
    std::unique_lock<std::mutex> lck(robotPoseMtx_); // Locks the data for the robotPose to be saved
    robotPose_ = pose; // We copy the pose here
}

void Sample::seperateThread() {
    // Waits for the data to be populated from ROS
    while(laserData_.range_min+laserData_.range_max == 0.0) ROS_INFO_STREAM("Loading...");
    //Limits the execution of this code to 5Hz
    int counter = 0;
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

        // ROS_INFO("AngleMin= %f\n AngleMax= %f\n AngleIncrement= %f", laserData_.angle_min, laserData_.angle_max, laserData_.angle_increment);
        
        std::pair<double, double> rangeBearing;
        // rangeBearing.first = 0.0;
        // rangeBearing.second = 0.0;
        rangeBearing = laserProcessing.MinDistAngle();
        // ROS_INFO("Min Range: %f", rangeBearing.first);

        //Gets the distance from the angle
        double dist = rangeBearing.first;
        // dist = laserProcessing.FindDistance(angle);

        //If the distance is less than the stop distance or more than the max value of an int (an invalid reading) the robot should stop
        if(dist < STOP_DISTANCE_ || dist > 2147483647){
            tooClose_ = true;
            ROS_INFO_STREAM("TurtleBot is too close to an obstacle!");
            ROS_INFO("Obstacle Range: %f\nObstacle angle: %f", rangeBearing.first, (rangeBearing.second*180/M_PI));
        }
        //Otherwise the robot is not too close
        else tooClose_ = false;
        
        // ROS_INFO("Obstacle Range: %f\nObstacle angle: %f", rangeBearing.first, (rangeBearing.second*180/M_PI));
        tooClose_ = false;
        
        // goals_ = pathPlanning.GetGoals();
        
        if(goal_.x == DBL_MAX_ && goal_.y == DBL_MAX_ && goal_.z == DBL_MAX_){
            std::vector<geometry_msgs::Point> fakeGoals;
            // int ARRAY_SIZE = 6;
            // double goal_arrayX[ARRAY_SIZE] = {2.0, 4.0,  7.0, 10.0,  8.0};
            // double goal_arrayY[ARRAY_SIZE] = {-2.0, 0.0, 1.0, -1.0, -2.5};
            int ARRAY_SIZE = 5;
            // double goal_arrayX[ARRAY_SIZE] = {1.0, 2.0, 1.0, 0.0};
            // double goal_arrayY[ARRAY_SIZE] = {1.0, 0.0, -1.0, 0.0};

            double goal_arrayX[ARRAY_SIZE] = {1.0, 1.0, 0.0, 0.0};
            double goal_arrayY[ARRAY_SIZE] = {0.0, -1.0, -1.0, 0.0};
            
            for(int i = 0; i+1 < ARRAY_SIZE; i++){
                geometry_msgs::Point fakeGoal;
                fakeGoal.x = goal_arrayX[i];
                fakeGoal.y = goal_arrayY[i];
                fakeGoals.push_back(fakeGoal);
            }
            goals_ = fakeGoals;
        }

        else if(DistanceToGoal(goal_, robotPose_) < GOAL_DISTANCE_) {
            if(goalIdx_+1 == goals_.size()){
                running_ = false;
                stateChange_ = true;
            }
            else{
                ROS_INFO_STREAM("***GOAL REACHED***\n");
                goalIdx_++;
            }
        }
        // else if(goalIdx_ == goals_.size()-1 && DistanceToGoal(goal_, robotPose_) < GOAL_DISTANCE_) 
        
        // std::vector<geometry_msgs::Point> fakeGoals;
        // geometry_msgs::Point fakeGoal;
        // fakeGoal.x = 2.0;
        // fakeGoal.y = 2.0;
        // fakeGoals.push_back(fakeGoal);
        // goals_ = fakeGoals;

        goal_ = goals_.at(goalIdx_);

        counter++;
        if(trajMode_ == 1 && counter == 5){
            ROS_INFO("goal_: (%f, %f)", goal_.x, goal_.y);
            ROS_INFO("Distance: %f", DistanceToGoal(goal_, robotPose_));
            counter = 0;
        }
        // for(int i = 0; i < goals_.size(); i++){
        //     ROS_INFO("goals_: (%f, %f)", goals_.at(i).x, goals_.at(i).y);
        // }

        //Creates the variable for driving the TurtleBot
        geometry_msgs::Twist drive;
        if(running_ && !tooClose_){
            drive.linear.x = 0.5; //sends it forward
            drive.linear.y = 0.0;
            drive.linear.z = 0.0;
            drive.angular.x = 0.0;
            drive.angular.y = 0.0;
            drive.angular.z = 0.0;

            if(trajMode_ == 1){
                double goal_angle = GetGoalAngle(goal_,robotPose_);
                // ROS_INFO("steering = %f", fabs(goal_angle));
                if(fabs(goal_angle) > 0.1){
                    goal_angle = GetGoalAngle(goal_,robotPose_);
                    drive.linear.x = 0.0;
                    drive.linear.y = 0.0;
                    drive.linear.z = 0.0;
                    drive.angular.x = 0.0;
                    drive.angular.y = 0.0;
                    drive.angular.z = goal_angle*STEERING_SENS_;
                    // ROS_INFO("steering = %f", drive.angular.z);
                }
                else{
                    drive.linear.x = 0.2;
                    drive.linear.y = 0.0;
                    drive.linear.z = 0.0;
                    drive.angular.x = 0.0;
                    drive.angular.y = 0.0;
                    drive.angular.z = 0.0;
                    // ROS_INFO("driving = %f", drive.linear.x);
                }
            }
            if(trajMode_ == 2){
                squiggles::Constraints constraints = squiggles::Constraints(MAX_VEL, MAX_ACCEL, MAX_JERK);
                squiggles::SplineGenerator generator = squiggles::SplineGenerator(
                    constraints,
                    std::make_shared<squiggles::TankModel>(ROBOT_WIDTH_, constraints));

                std::vector<squiggles::ProfilePoint> path = generator.generate({squiggles::Pose(0, 0, 0), squiggles::Pose(-2, -2, 0)});
                
                double desiredVelocity = path.front().vector.vel;
                // std::string single_path = path.at(0).to_string();
                // ROS_INFO("Path: %s", single_path.c_str());
                // std::stringstream ss;
                // ss << "Member 1: " << path.member1 << ", ";
                // ss << "Member 2: " << path.member2 << ", ";
                // ss << "Member 3: " << path.member3;

                // std::vector<squiggles::ProfilePoint> path = generator.generate({
                // squiggles::Pose(0.0, 0.0, 1.0),
                // squiggles::Pose(-4.0, -4.0, 1.0)});
            }

            if(stateChange_){
                ROS_INFO_STREAM("TurtleBot is moving");
                stateChange_ = false;
            }
        }
        //Stops the TurtleBot
        else{
            drive.linear.x = 0.0;
            drive.linear.y = 0.0;
            drive.linear.z = 0.0;
            drive.angular.x = 0.0;
            drive.angular.y = 0.0;
            drive.angular.z = 0.0;

            if(stateChange_){
                ROS_INFO_STREAM("TurtleBot is stopped");
                stateChange_ = false;
            }
        }
        // geometry_msgs::Twist drive;
        // drive.linear.x = 0.5; //sends it forward
        // drive.linear.y = 0.0;
        // drive.linear.z = 0.0;
        // drive.angular.x = 0.0;
        // drive.angular.y = 0.0;
        // drive.angular.z = 0.0;
        // ROS_INFO_STREAM("TurtleBot is moving");
        
        // Publishes the drive variable to control the TurtleBot
        if(trajMode_ != 0) pubDrive_.publish(drive);

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
        stateChange_ = true;
        res.success = true;
        res.message = "The Turtlebot has started it's mission";

    }
    //if the request is false, stop the mission
    else
    {
        ROS_INFO_STREAM("Requested: Stop mission");
        res.success = true;
        stateChange_ = true;
        res.message = "Turtlebot stopping";
        running_ = false;
    }
    //return true when the service completes its request
    return true;
}

//Service that handles starting and stopping the missions based on command line input
//Communicate with this service using 
//rosservice call /mission "data: true"
bool Sample::real(std_srvs::SetBool::Request  &req,
                     std_srvs::SetBool::Response &res)
{  
    //If the request is true, start the mission
    if(req.data)
    {
        ROS_INFO_STREAM("Requested: Real robot");
        real_ = true; //start the robot if there is a goal
        // stateChange_ = true;
        res.success = true;
        res.message = "The Turtlebot has started it's mission";

    }
    //if the request is false, stop the mission
    else
    {
        ROS_INFO_STREAM("Requested: Sim robot");
        real_ = false;
        res.success = true;
        res.message = "Turtlebot stopping";
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
double Sample::GetGoalAngle(geometry_msgs::Point goal, geometry_msgs::Pose robot)
{
    /*-----Chord Length-----*/
    double dx = goal.x-robot.position.x;    //Difference in x between the robot's current position and the goal
    double dy = goal.y-robot.position.y;    //Difference in y between the robot's current position and the goal
    double CL = sqrt(pow(dx,2)+pow(dy,2));  //Chord length is hypotenuse of perpendicular lengths of dx and dy

    /*-----Alpha Angle-----*/
    double AA = atan2(dy, dx)-tf::getYaw(robot.orientation);  //Alpha angle made with angle formed from chord from the yaw angle of the robot
    return AA;
}

double Sample::fabs(double x)
{
    if(x < 0) return -x;
    else return x;
}