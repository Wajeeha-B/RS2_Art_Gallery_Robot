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
    nh_(nh), running_(false), real_(true), laserProcessingPtr_(nullptr), pathPlanningPtr_(nullptr),
    tooClose_(false), stateChange_(true), marker_counter_(0), world_x_(0.0), world_y_(0.0)
{
    //Subscribing to the laser sensor
    sub1_ = nh_.subscribe("/scan", 100, &Sample::laserCallback,this);
    //Subscribing to odometry of the robot
    sub2_ = nh_.subscribe("/amcl_pose", 100, &Sample::amclCallback,this);

    sub3_ = nh_.subscribe("/thepath", 10, &Sample::pathCallback,this);

    sub4_ = nh_.subscribe("/map", 100, &Sample::mapCallback, this);
    //Publishing the driving commands
    pubDrive_ = nh.advertise<geometry_msgs::Twist>("/cmd_vel",3,false);

    pubVis_ = nh_.advertise<visualization_msgs::MarkerArray>("visualization_marker",3,false);

    goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);

    //Service to enable the robot to start and stop from command line input
    service1_ = nh_.advertiseService("/mission", &Sample::request,this);

    service2_ = nh_.advertiseService("/real", &Sample::real,this);

    make_plan_ = nh.serviceClient<nav_msgs::GetPlan>("/move_base/NavfnROS/make_plan");
    
    //Sets the default robotPose_ to 0
    robotPose_.position.x = 0.0;
    robotPose_.position.y = 0.0;
    robotPose_.position.z = 0.0;
    robotPose_.orientation.x = 0.0;
    robotPose_.orientation.y = 0.0;
    robotPose_.orientation.z = 0.0;
    robotPose_.orientation.w = 0.0;

    //Sets the default pathData_ to 0
    pathData_.x = 0.0;
    pathData_.y = 0.0;
    pathData_.z = 0.0;

    //Sets the default goal to (0,0,0)
    goal_.x = DBL_MAX_;
    goal_.y = DBL_MAX_;
    goal_.z = DBL_MAX_;
    // ROS_INFO_STREAM("Goal is reset!");

    goalIdx_ = 0;
    
    velIdx_ = 0;
    time_ = 0;
    smoothVelIdx_ = 0;
    poseError_ = 0.0;
    minIdx_ = 0;
}

// We delete anything that needs removing here specifically
Sample::~Sample(){
    if(laserProcessingPtr_ != nullptr){
        delete laserProcessingPtr_;
    }
    if(pathPlanningPtr_ != nullptr){
        delete pathPlanningPtr_;
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

//A callback for the laser scanner
void Sample::pathCallback(const geometry_msgs::PointConstPtr& msg)
{
    std::unique_lock<std::mutex> lck(pathDataMtx_); // Locks the data for the laserData to be saved
    pathData_ = *msg; // We store a copy of the LaserScan in laserData_
}

//A callback for map
void Sample::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    std::unique_lock<std::mutex> lck(mapMtx_);
     // Parse the map data
    map_width_ = msg->info.width;
    //std::cout << "Map Width: " << map_width_ << std::endl;
    map_height_ = msg->info.height;
    //std::cout << "Map Height: " << map_height_ << std::endl;
    map_resolution_ = msg->info.resolution;
    //std::cout << "Map Resolution: " << map_resolution_ << std::endl;
    map_origin_x_ = msg->info.origin.position.x;
    //std::cout << "Map Origin X: " << map_origin_x_ << std::endl;
    map_origin_y_ = msg->info.origin.position.y;
    //std::cout << "Map Origin Y: " << map_origin_y_ << std::endl;
    map_data_ = msg->data;
    //std::cout << "Map data provided." << std::endl;
    // Generate a random goal point
}

void Sample::seperateThread() {
    // Waits for the data to be populated from ROS
    while(laserData_.range_min+laserData_.range_max == 0.0) ROS_INFO_STREAM("Loading...");
    //Limits the execution of this code to 5Hz
    int counter = 0;
    ros::Rate rate_limiter(10.0);
    while (ros::ok()) {
        //Locks all of the data with mutexes
        std::unique_lock<std::mutex> lck1 (laserDataMtx_);
        std::unique_lock<std::mutex> lck2 (robotPoseMtx_);
        // std::unique_lock<std::mutex> lck3 (pathDataMtx_);
        std::unique_lock<std::mutex> lck3 (mapMtx_);
        
        //Creates the class object and gives the data from the sensors
        LaserProcessing laserProcessing(laserData_);
        PathPlanning pathPlanning(map_width_, map_height_, map_resolution_, map_origin_x_, map_origin_y_, map_data_, world_x_, world_y_);

        //Unlocks all mutexes
        // lck4.unlock();
        lck3.unlock();
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
            // ROS_INFO_STREAM("TurtleBot is too close to an obstacle!");
            // ROS_INFO("Obstacle Range: %f\nObstacle angle: %f", rangeBearing.first, (rangeBearing.second*180/M_PI));
        }
        //Otherwise the robot is not too close
        else if(!tooClose_) tooClose_ = false;
        
        // ROS_INFO("Obstacle Range: %f\nObstacle angle: %f", rangeBearing.first, (rangeBearing.second*180/M_PI));
        tooClose_ = false;
        
        geometry_msgs::Point fakeMarker;
        fakeMarker.x = 1.0;
        fakeMarker.y = 1.0;
        fakeMarker.z = 0.0;

        visualization_msgs::MarkerArray markerArray; //creates the marker array for publishing

        // markerArray = CollectGoals(markerArray);
        // if(!goals_.empty()) goal_ = goals_.at(goalIdx_);

        // for(int i = 0; )
        //We create a marker for the goal
        // visualization_msgs::Marker marker = createMarker(fakeMarker, 0, 1, 0);
        // markerArray.markers.push_back(marker); //goal marker is pushed into marker array

        // goals_ = pathPlanning.GetGoals();
        // markerArray = CollectGoals(markerArray);
        
        if(goals_.empty()) goals_ = generateRandomGoals(pathPlanning);
        else{
            for(int i = 0; i < goals_.size(); i++){
                geometry_msgs::Point markerPoint;
                markerPoint.x = goals_.at(i).x;
                markerPoint.y = goals_.at(i).y;
                visualization_msgs::Marker marker = createMarker(markerPoint, 1.0, 0.0, 0.0);
                markerArray.markers.push_back(marker);
            }
        }
        goal_ = goals_.at(goalIdx_);


        // if(goal_.x == DBL_MAX_ && goal_.y == DBL_MAX_ && goal_.z == DBL_MAX_){
        //     std::vector<geometry_msgs::Point> fakeGoals;
        //     // int ARRAY_SIZE = 6;
        //     // double goal_arrayX[ARRAY_SIZE] = {2.0, 4.0,  7.0, 10.0,  8.0};
        //     // double goal_arrayY[ARRAY_SIZE] = {-2.0, 0.0, 1.0, -1.0, -2.5};
        //     int ARRAY_SIZE = 5;
        //     // double goal_arrayX[ARRAY_SIZE] = {1.0, 2.0, 1.0, 0.0};
        //     // double goal_arrayY[ARRAY_SIZE] = {1.0, 0.0, -1.0, 0.0};

        //     // double goal_arrayX[ARRAY_SIZE] = {2.0, 4.0, 6.0, 8.0};
        //     // double goal_arrayY[ARRAY_SIZE] = {0.0, -2.0, -2.5, -2.5};

        //     double goal_arrayX[ARRAY_SIZE] = {0.0, 0.5, 0.0, 2.0};
        //     double goal_arrayY[ARRAY_SIZE] = {1.5, 0.0, -1.5, -1.5};

        //     for(int i = 0; i+1 < ARRAY_SIZE; i++){
        //         geometry_msgs::Point fakeGoal;
        //         fakeGoal.x = goal_arrayX[i];
        //         fakeGoal.y = goal_arrayY[i];
        //         fakeGoals.push_back(fakeGoal);
        //     }
        //     goals_ = fakeGoals;
        // }

        if(DistanceToGoal(goal_, robotPose_) < GOAL_DISTANCE_) {
            if(goalIdx_+1 == goals_.size()){ //if this is the last goal
                running_ = false;
                stateChange_ = true;
            }
            else if(goalIdx_+1 < goals_.size()){ //if this isnt the last goal
                ROS_INFO_STREAM("***GOAL REACHED***\n");
                goalIdx_++;
            }
            if(trajMode_ == 2 && running_) GenerateSpline(); //generate a spline
        }

        if(trajMode_ == 2 && path_.empty()){
            GenerateSpline();
        }
        
        // Calculate pose error
        if(trajMode_ == 2){
            geometry_msgs::Point velPose;
            velPose.x = path_.at(velIdx_).vector.pose.x;
            velPose.y = path_.at(velIdx_).vector.pose.y;
            poseError_ = DistanceToGoal(velPose, robotPose_);
            for(int i = 0; i < path_.size(); i++){
                geometry_msgs::Point markerPoint;
                markerPoint.x = path_.at(i).vector.pose.x;
                markerPoint.y = path_.at(i).vector.pose.y;
                visualization_msgs::Marker marker = createMarker(markerPoint, 1.0, 0.0, 0.0);
                markerArray.markers.push_back(marker);
            }
        }

        // if(poseError_ > 0.2) smoothVelIdx_ -= 2;

        counter++;
        if(counter == 10 && goal_.x != DBL_MAX_ && goal_.y != DBL_MAX_ && goal_.z != DBL_MAX_){
            ROS_INFO("goal_: (%f, %f)", goal_.x, goal_.y);
            ROS_INFO("Distance: %f", DistanceToGoal(goal_, robotPose_));
            // ROS_INFO("GoalIdx: %d", goalIdx_);
            // ROS_INFO("Pose error: %f", poseError_);
            // ROS_INFO("goals_ size: %ld", goals_.size());
            // if(!goals_.empty()){
            //     for (int i = 0; i < goals_.size()-1; i++){
            //         ROS_INFO("goals_ at %d: (%f, %f)", i, goals_.at(i).x, goals_.at(i).y);
            //     }
            // }
            counter = 0;
        }
        else if (counter > 10) counter = 0;
        // for(int i = 0; i < goals_.size(); i++){
        //     ROS_INFO("goals_: (%f, %f)", goals_.at(i).x, goals_.at(i).y);
        // }
        

        //Creates the variable for driving the TurtleBot
        geometry_msgs::Twist drive;
        if(running_ && !tooClose_){
            if(stateChange_){
                ROS_INFO_STREAM("TurtleBot is moving");
                stateChange_ = false;
            }

            drive.linear.x = 0.5; //sends it forward
            drive.linear.y = 0.0;
            drive.linear.z = 0.0;
            drive.angular.x = 0.0;
            drive.angular.y = 0.0;
            drive.angular.z = 0.0;

            if(trajMode_ == 1){
                geometry_msgs::Point lookaheadPoint = FindLookaheadPoint(goals_);
                double goal_angle = GetGoalAngle(lookaheadPoint,robotPose_);
                visualization_msgs::Marker marker = createMarker(lookaheadPoint, 0.0, 1.0, 0.0);
                markerArray.markers.push_back(marker);
                
                // ROS_INFO("steering = %f", goal_angle);
                // ROS_INFO("smoothVelIdx = %d", smoothVelIdx_);

                // double goal_angle = GetGoalAngle(goal_,robotPose_);
                // ROS_INFO("steering = %f", fabs(goal_angle));
                if(fabs(goal_angle) > 0.1){
                    smoothVelIdx_ = 0;
                    // goal_angle = GetGoalAngle(goal_,robotPose_);
                    drive.linear.x = 0.0;
                    drive.linear.y = 0.0;
                    drive.linear.z = 0.0;
                    drive.angular.x = 0.0;
                    drive.angular.y = 0.0;
                    drive.angular.z = goal_angle*STEERING_SENS_;
                    // ROS_INFO("steering = %f", drive.angular.z);
                }
                else{
                    smoothVelIdx_++;
                    drive.linear.x = SmoothVel(smoothVelIdx_);
                    drive.linear.y = 0.0;
                    drive.linear.z = 0.0;
                    drive.angular.x = 0.0;
                    drive.angular.y = 0.0;
                    drive.angular.z = 0.0;
                    // ROS_INFO("driving = %f", drive.linear.x);
                }
            }
            if(trajMode_ == 2){
                std::vector<geometry_msgs::Point> goals;
                for(int i = 0; i < path_.size(); i++){
                    goals.at(i).x = path_.at(i).vector.pose.x;
                    goals.at(i).y = path_.at(i).vector.pose.y;
                }
                geometry_msgs::Point lookaheadPoint = FindLookaheadPoint(goals);
                double goal_angle = GetGoalAngle(lookaheadPoint,robotPose_);
                visualization_msgs::Marker marker = createMarker(lookaheadPoint, 0.0, 1.0, 0.0);
                markerArray.markers.push_back(marker);

                // smoothVelIdx_++;
                // drive.linear.x = SmoothVel(smoothVelIdx_);
                drive.linear.x = 0.1;
                drive.linear.y = 0.0;
                drive.linear.z = 0.0;
                drive.angular.x = 0.0;
                drive.angular.y = 0.0;
                drive.angular.z = goal_angle;
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

        pubVis_.publish(markerArray);

        //We have a rate timer, this sleep here is needed to ensure it stops and sleeps 
        //it will do it for the exact amount of time needed to run at 5Hz
        rate_limiter.sleep();
    }
}

visualization_msgs::Marker Sample::createMarker(geometry_msgs::Point point, double r, double g, double b){
  
  visualization_msgs::Marker marker;

  //Set the reference frame ID and time stamp.
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time::now();

  //Lifetime of the object, matches the Hz of the rate limiter
  marker.lifetime = ros::Duration(0.1);
  
  //Unique ID of the marker
  marker.id = marker_counter_++; 

  marker.type = visualization_msgs::Marker::CYLINDER;
  // Set the scale of the marker
  marker.scale.x = 0.1;
  marker.scale.y = 0.1;
  marker.scale.z = 0.1;
  //Colour is r,g,b where each channel of colour is 0-1. This marker is green
  marker.color.r = r;
  marker.color.g = g;
  marker.color.b = b;

  // Set the marker action. This will add it to the screen
  marker.action = visualization_msgs::Marker::ADD;

  // Sets the position of the marker
  marker.pose.position.x = point.x;
  marker.pose.position.y = point.y;
  marker.pose.position.z = point.z;

  //Sets the orientation, we are not going to orientate it
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  //Alpha is transparency (50% transparent)
  marker.color.a = 0.5f;

  return marker;
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

visualization_msgs::MarkerArray Sample::CollectGoals(visualization_msgs::MarkerArray markerArray)
{
    if(pathData_.x > 0.0 || pathData_.x < 0.0){
        // if(goals_.empty()) goals_.push_back(pathData_);
        // else if(pathData_.x != goals_.back().x &&
        //         pathData_.y != goals_.back().y &&
        //         pathData_.z != goals_.back().z) goals_.push_back(pathData_);
        if(goals_.empty()||(pathData_ != goals_.back())) goals_.push_back(pathData_);
    }

    if(!goals_.empty()){
        goal_ = goals_.at(goalIdx_);
        for(int i = 0; i < goals_.size(); i++){
            geometry_msgs::Point markerPoint;
            markerPoint.x = goals_.at(i).x;
            markerPoint.y = goals_.at(i).y;
            visualization_msgs::Marker marker = createMarker(markerPoint, 1.0, 0.0, 0.0);
            markerArray.markers.push_back(marker);
        }
    }
    return markerArray;
}

void Sample::GenerateSpline(){
    goal_ = goals_.at(goalIdx_);
    squiggles::Constraints constraints = squiggles::Constraints(MAX_VEL, MAX_ACCEL, MAX_JERK);
    squiggles::SplineGenerator generator = squiggles::SplineGenerator(
        constraints,
        std::make_shared<squiggles::TankModel>(ROBOT_WIDTH_, constraints));
    path_ = generator.generate({squiggles::Pose(robotPose_.position.x, robotPose_.position.y, tf::getYaw(robotPose_.orientation)),
                                squiggles::Pose(goal_.x, goal_.y, GetGoalOrientation(goals_, robotPose_))});
    time_ = 0.0;
    velIdx_ = 0;
    minIdx_ = 0;
    ROS_INFO_STREAM("Path generated\n");
}

double Sample::GetGoalOrientation(std::vector<geometry_msgs::Point> goals, geometry_msgs::Pose robot)
{
    if (goalIdx_ < goals.size()-2){
        geometry_msgs::Point goal = goals.at(goalIdx_+1);
        double angle = GetGoalAngle(goal,robot);
        // ROS_INFO("goal1: %f, %f)", goals.at(goalIdx_).x, goals.at(goalIdx_).y);
        // ROS_INFO("goal2: %f, %f)", goals.at(goalIdx_+1).x, goals.at(goalIdx_+1).y);
        // ROS_INFO("angle: %f", angle);
        return angle;
    }
    else return 0.0;
}

geometry_msgs::Point Sample::FindLookaheadPoint(std::vector<geometry_msgs::Point> goals)
{
    for (size_t i = minIdx_; i < goals.size() - 1; i++) {
        geometry_msgs::Point goal;
        goal.x = goals.at(i).x;
        goal.y = goals.at(i).y;
        goalIdx_ = i;
        
        if (DistanceToGoal(goal, robotPose_) > lookahead_dist_) {
            minIdx_ = i;
            return goal;
        }
    }

    // If no valid lookahead point is found, return the last waypoint
    geometry_msgs::Point back;
    back.x = goals.back().x;
    back.y = goals.back().y;
    goalIdx_ = goals.size() - 1;
    return back;
}

double Sample::computeCurvature(geometry_msgs::Point goal, geometry_msgs::Pose robot)
{
    double alpha = GetGoalAngle(goal, robot);
    double distance = DistanceToGoal(goal, robot);
    return (2 * std::sin(alpha)) / distance;
}

double Sample::SmoothVel(unsigned int idx)
{
    if(idx*0.01 < 0.26) return idx*0.01;
    else return 0.26;
}

std::vector<geometry_msgs::Point> Sample::generateRandomGoals(PathPlanning pathPlanning)
{
    // unordered_goals_.clear();
    //PathPlanning.getGoals();
    
    geometry_msgs::Point start;
    geometry_msgs::Point end;
    std::vector<geometry_msgs::Point> waypts_simplified; // the vector created by plan between two Goals
    std::vector<geometry_msgs::Point> combined_waypoints;
    for (int i=0; i<5; i++)
    {
        // push a random goal into the unordered_goals_ vector
        pathPlanning.generateRandomGoal(unordered_goals_, robotPose_);
        //start = unordered_goals_[i].pose.position;
        //std::cout << "unordered_goals_[ " << i << " ]: {" << start.x << " , " << start.y << "}." << std::endl; 
        if(i==0)
        {
            start.x = robotPose_.position.x; start.y = robotPose_.position.x; // as starting point is always (0,0)
            end = unordered_goals_[i].pose.position;
        }
        else
        {
            start = unordered_goals_[i-1].pose.position;
            end = unordered_goals_[i].pose.position;
        }
        
        // for error checking
        std::cout << "Start set: {" << start.x << " , " << start.y << "}." << std::endl; 
        std::cout << "End set : {" << end.x << " , " << end.y << "}." << std::endl; 
        
        // find simplified path
        waypts_simplified = planBetweenTwoGoals(pathPlanning, start, end);
        // publish to rostopic 'thepath'
        // publishPath(waypts_simplified);
        std::cout << "Waypoints from Goal " << i << " to Goal " << (i+1) << " published to ROS Topic /thepath." << std::endl;
        combined_waypoints.insert(combined_waypoints.end(), waypts_simplified.begin(), waypts_simplified.end());
    }
    return combined_waypoints;
            
  
        // // making vector of point msgs from start and end
 



    //  ADD TRAVELLING SALESMAN 

        // geometry_msgs::PoseStamped goal_msg;
        // goal_msg.pose.position.x = goal_point.x; // Set the goal position x-coordinate
        // goal_msg.pose.position.y = goal_point.y; // Set the goal position y-coordinate
        // goal_msg.pose.orientation.w = 1.0;
        // goal_msg.header.stamp = ros::Time::now();
        // goal_msg.header.frame_id = "map";
        // //std::cout << "Goal about to be published." << std::endl;
        // goal_pub_.publish(goal_msg);
        // std::cout << "Goal published." << std::endl;

}

std::vector<geometry_msgs::Point> Sample::planBetweenTwoGoals(PathPlanning pathPlanning, geometry_msgs::Point st, geometry_msgs::Point en)
{
    std::vector<geometry_msgs::Point> points;
  // Create a request message for the service
    nav_msgs::GetPlan srv;
    srv.request.start.header.frame_id = "map";
    srv.request.start.pose.position.x = st.x;
    srv.request.start.pose.position.y = st.y;
    srv.request.start.pose.orientation.w = 1.0;

    srv.request.goal.header.frame_id = "map";
    srv.request.goal.pose.position.x = en.x;
    srv.request.goal.pose.position.y = en.y;
    srv.request.goal.pose.orientation.w = 1.0;

    // Call the service - the statement within the if() calls the service with srv
    if (make_plan_.call(srv)) {
        if (!srv.response.plan.poses.empty())
        {
            ROS_INFO("Plan received with %ld poses", srv.response.plan.poses.size());
            for (size_t i = 0; i < srv.response.plan.poses.size(); i += 10) // get every 10 points
            {   
                points.push_back(srv.response.plan.poses[i].pose.position);
                double x = srv.response.plan.poses[i].pose.position.x;
                double y = srv.response.plan.poses[i].pose.position.y;
                //std::cout << "Position: (" << x << " , " << y << ")." << std::endl; 
            }
            // if last point is not end point / goal
            if(!(points.back()==en))
            {
                //std::cout << "Since point vector doesn't have end goal as last point, pushing it back." << std::endl; 
                points.push_back(en);
                //std::cout << "Position: (" << en.x << " , " << en.y << ")." << std::endl; 
            }

        } 
        else 
        {
            ROS_WARN("Received an empty plan so removing last random goal. Generating new goal...");
            unordered_goals_.pop_back();
            pathPlanning.generateRandomGoal(unordered_goals_, robotPose_);
        }
    } 
    else
    {
        ROS_ERROR("Failed to call service make_plan");
    }

    std::cout << "Number of elements in the simplified points vector is: " << points.size() << std::endl;
    return points;
}