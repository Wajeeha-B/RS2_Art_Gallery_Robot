#include "sample.h"
#include <iostream>
#include <cmath>
#include <thread>
#include <chrono>
#include <time.h>
#include <random>
  
//Default constructor of the sample class
Sample::Sample(ros::NodeHandle nh) :
    //Setting the default value for some variables
    nh_(nh), laserProcessingPtr_(nullptr), world_x(0.0), world_y(0.0)
{
    //Subscribing to the laser sensor
    sub_scan = nh_.subscribe("/scan", 100, &Sample::laserCallback,this);
    //Subscribing to odometry of the robot
    sub_odom = nh_.subscribe("/odom", 100, &Sample::odomCallback,this);
    map_sub_ = nh_.subscribe("/map", 100, &Sample::mapCallback, this);

    goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);
    custompath_pub_ = nh_.advertise<geometry_msgs::Point>("/thepath", 10);

    //goals_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/randomgoals", 5);

    make_plan_ = nh.serviceClient<nav_msgs::GetPlan>("/move_base/NavfnROS/make_plan");
  
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

//A callback for odometry
void Sample::odomCallback(const nav_msgs::OdometryConstPtr &msg)
{
    geometry_msgs::Pose pose = msg->pose.pose;
    std::unique_lock<std::mutex> lck(robotPoseMtx_); // Locks the data for the robotPose to be saved
    robotPose_ = pose; // We copy the pose here
}

//A callback for map
void Sample::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
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

void Sample::generateRandomGoals()
{
    unordered_goals_.clear();
    
    geometry_msgs::Point start;
    geometry_msgs::Point end;
    std::vector<geometry_msgs::Point> waypts_simplified; // the vector created by plan between two Goals
    for (int i=0; i<5; i++)
    {
        // push a random goal into the unordered_goals_ vector
        generateRandomGoal();
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
        waypts_simplified = planBetweenTwoGoals(start, end);
        // publish to rostopic 'thepath'
        publishPath(waypts_simplified);
        std::cout << "Waypoints from Goal " << i << " to Goal " << (i+1) << " published to ROS Topic /thepath." << std::endl; 
    }

            
  
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


void Sample::publishPath(std::vector<geometry_msgs::Point> vec_of_simplified_waypts)
{
   for (const auto &point : vec_of_simplified_waypts)
    {
        custompath_pub_.publish(point);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

    }
    // wait half a sec
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
}



std::vector<geometry_msgs::Point> Sample::planBetweenTwoGoals(geometry_msgs::Point st, geometry_msgs::Point en)
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
            for (size_t i = 0; i < srv.response.plan.poses.size(); i += 10)
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
            generateRandomGoal();

        }
    } 
    else
    {
        ROS_ERROR("Failed to call service make_plan");
    }

    std::cout << "Number of elements in the simplified points vector is: " << points.size() << std::endl;
    return points;
}


void Sample::generateRandomGoal()
{
    std::this_thread::sleep_for(std::chrono::seconds(1));
    uint32_t idx;

        while (true) {
            std::random_device rd;
            std::mt19937 gen(rd());
            std::uniform_real_distribution<> grid_x(10, map_width_-10); // 5
            std::uniform_real_distribution<> grid_y(10, map_height_-10);
            double map_x = grid_x(gen);
            double map_y = grid_y(gen);

            if (Sample::isGoalValid(map_x, map_y))
            {   
                //std::cout << "map_x: " << map_x << ", map_y: " << map_y << "} is a valid goal." << std::endl;
                //std::cout << " Random Goal { " << world_x << " , " << world_y << " } is valid." << std::endl;
                break;
            }
        }

        // Put the goal point in PoseStamped format
        currentgoal_.pose.position.x = world_x; // Set the goal position x-coordinate
        currentgoal_.pose.position.y = world_y; // Set the goal position y-coordinate
        currentgoal_.pose.orientation.w = 1.0;
        currentgoal_.header.stamp = ros::Time::now();
        currentgoal_.header.frame_id = "map";

        // Push goal message into vector
        unordered_goals_.push_back(currentgoal_);
        std::cout << "Goal {" << world_x << " , " << world_y << " } has been pushed into unordered vector." << std::endl;
}

bool Sample::isGoalValid(double x, double y)
{
    world_x = map_origin_x_ + (x) * map_resolution_; // (x+ 0.5)
    world_y = map_origin_y_ + (y) * map_resolution_;
    // Check if the goal point is a threshold distance away from obstacles
        //int map_x = (x - map_origin_x_) / map_resolution_;
        //int map_y = (y - map_origin_y_) / map_resolution_;
        uint32_t idx = x + y * map_width_;
        // std::cout << "Index is: " << idx << std::endl;
        //int index = map_y * map_width_ + map_x;
        if (idx >= 0 && idx < map_data_.size()) 
        {
            bool withinBounds = false;
            bool freeSpace = false;
            bool neighboursUnoccupied = true; // assume unoccupied 
            bool withinThreshold = false; // TO ADD
            if(world_x >= 0 && world_x <= 10 && world_y >= -9 && world_y <= 0)
            {
                withinBounds = true;
            }
            if (map_data_[idx] == 0)
            {
                freeSpace = true;
            }
            for (int i = idx-100; i <= idx+100; ++i) // for data 100 to right and 100 to left
            {
                if(!(map_data_[idx] == 0))
                {
                    neighboursUnoccupied = false;
                    break;
                }
            }
            if(DistanceToGoal(world_x, world_y, robotPose_) > 0.75)
            {
                withinThreshold = true;
            }


            if(withinBounds && freeSpace && neighboursUnoccupied && withinThreshold)
            {
                //std::cout <<  "If 0, free space: " << map_data_[idx] << std::endl; 
                //std::cout << "Goal VALID as free space, within bounds and distance threshold and neighbours cells are unoccupied." << std::endl;
                return true;
            }
            else
            {
                //std::cout << "Goal inside obstacle, out of bounds or distance threshold, unknown, or neigbouring cells occupied." << std::endl;
                return false; // Goal point is inside an obstacle
                generateRandomGoal(); // generate another random goal
            }
        }
}

//Gets the distance from the goal to the robot
double Sample::DistanceToGoal(double goal_x, double goal_y, geometry_msgs::Pose robot)
{
    //finds the difference in x and y and get the hypotenuse between the two points
    double dist = sqrt(pow(goal_x-robot.position.x,2)+pow(goal_y-robot.position.y,2));
    return dist;
}

//Gets the distance from two goals
double Sample::DistanceBetweenGoals(geometry_msgs::Point goal1, geometry_msgs::Point goal2)
{
    //finds the difference in x and y and get the hypotenuse between the two points
    double dist = sqrt(pow(goal1.x-goal2.x,2)+pow(goal1.y-goal2.y,2));
    return dist;
}

void Sample::seperateThread() {
    //Waits for the data to be populated from ROS
    while(laserData_.range_min+laserData_.range_max == 0.0);//||
        //   robotPose_.orientation.w+robotPose_.orientation.x+
        //   robotPose_.orientation.y+robotPose_.orientation.z == 0.0);

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

        // can put code here 

        //We have a rate timer, this sleep here is needed to ensure it stops and sleeps 
        //it will do it for the exact amount of time needed to run at 5Hz
        rate_limiter.sleep();
    }
}




double Sample::fabs(double x)
{
    if(x < 0) return -x;
    else return x;
}