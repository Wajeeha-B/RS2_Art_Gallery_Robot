#include "pathplanning.h"
#include <algorithm>
#include <numeric>
#include <iostream>
#include <cmath>
#include <thread>
#include <chrono>
#include <time.h>
#include <random>

using namespace std;

//Default constructor of Laserprocessing, needs an initial set of laser data to be initialised
PathPlanning::PathPlanning(int map_width, int map_height, double map_resolution,
                           double map_origin_x, double map_origin_y, std::vector<int8_t> map_data, double world_x, double world_y):
                           map_width_(map_width), map_height_(map_height), map_resolution_(map_resolution),
                           map_origin_x_(map_origin_x), map_origin_y_(map_origin_y), map_data_(map_data), world_x_(world_x),world_y_(world_y)
    {

    }

PathPlanning::~PathPlanning(){

}

void PathPlanning::generateRandomGoal(std::vector<geometry_msgs::PoseStamped>& unordered_goals, geometry_msgs::Pose robotPose)
{
    std::this_thread::sleep_for(std::chrono::seconds(1));
    // uint32_t idx;

    while (true) {
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<> grid_x(10, map_width_-10); // 5
        std::uniform_real_distribution<> grid_y(10, map_height_-10);
        double map_x = grid_x(gen);
        double map_y = grid_y(gen);

        if (isGoalValid(map_x, map_y, unordered_goals, robotPose))
        {   
            //std::cout << "map_x: " << map_x << ", map_y: " << map_y << "} is a valid goal." << std::endl;
            //std::cout << " Random Goal { " << world_x << " , " << world_y << " } is valid." << std::endl;
            break;
        }
    }

    // Put the goal point in PoseStamped format
    geometry_msgs::PoseStamped currentgoal;

    currentgoal.pose.position.x = world_x_; // Set the goal position x-coordinate
    currentgoal.pose.position.y = world_y_; // Set the goal position y-coordinate
    currentgoal.pose.orientation.w = 1.0;
    currentgoal.header.stamp = ros::Time::now();
    currentgoal.header.frame_id = "map";

    // Push goal message into vector
    unordered_goals.push_back(currentgoal);
    // unordered_goals = unordered_goals_;
    std::cout << "Goal {" << world_x_ << " , " << world_y_ << " } has been pushed into unordered vector." << std::endl;
}

bool PathPlanning::isGoalValid(double x, double y, std::vector<geometry_msgs::PoseStamped>& unordered_goals, geometry_msgs::Pose robotPose)
{
    world_x_ = map_origin_x_ + (x) * map_resolution_; // (x+ 0.5)
    world_y_ = map_origin_y_ + (y) * map_resolution_;
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
        if(world_x_ >= 0 && world_x_ <= 10 && world_y_ >= -9 && world_y_ <= 0)
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
        if(DistanceToGoal(world_x_, world_y_, robotPose) > 0.75)
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
            generateRandomGoal(unordered_goals, robotPose); // generate another random goal
        }
    }
}

// void PathPlanning::generateRandomGoals()
// {
//     unordered_goals_.clear();
    
//     geometry_msgs::Point start;
//     geometry_msgs::Point end;
//     std::vector<geometry_msgs::Point> waypts_simplified; // the vector created by plan between two Goals
//     for (int i=0; i<5; i++)
//     {
//         // push a random goal into the unordered_goals_ vector
//         generateRandomGoal();
//         //start = unordered_goals_[i].pose.position;
//         //std::cout << "unordered_goals_[ " << i << " ]: {" << start.x << " , " << start.y << "}." << std::endl; 
//         if(i==0)
//         {
//             start.x = robot.position.x; start.y = robot.position.x; // as starting point is always (0,0)
//             end = unordered_goals_[i].pose.position;
//         }
//         else
//         {
//             start = unordered_goals_[i-1].pose.position;
//             end = unordered_goals_[i].pose.position;
//         }
        
//         // for error checking
//         std::cout << "Start set: {" << start.x << " , " << start.y << "}." << std::endl; 
//         std::cout << "End set : {" << end.x << " , " << end.y << "}." << std::endl; 
        
//         // find simplified path
//         waypts_simplified = planBetweenTwoGoals(start, end);
//         // publish to rostopic 'thepath'
//         // publishPath(waypts_simplified);
//         std::cout << "Waypoints from Goal " << i << " to Goal " << (i+1) << " published to ROS Topic /thepath." << std::endl; 
//     }

            
  
//         // // making vector of point msgs from start and end
 



//     //  ADD TRAVELLING SALESMAN 

//         // geometry_msgs::PoseStamped goal_msg;
//         // goal_msg.pose.position.x = goal_point.x; // Set the goal position x-coordinate
//         // goal_msg.pose.position.y = goal_point.y; // Set the goal position y-coordinate
//         // goal_msg.pose.orientation.w = 1.0;
//         // goal_msg.header.stamp = ros::Time::now();
//         // goal_msg.header.frame_id = "map";
//         // //std::cout << "Goal about to be published." << std::endl;
//         // goal_pub_.publish(goal_msg);
//         // std::cout << "Goal published." << std::endl;

// }

//Gets the distance from the goal to the robot
double PathPlanning::DistanceToGoal(double goal_x, double goal_y, geometry_msgs::Pose robot)
{
    //finds the difference in x and y and get the hypotenuse between the two points
    double dist = sqrt(pow(goal_x-robot.position.x,2)+pow(goal_y-robot.position.y,2));
    return dist;
}