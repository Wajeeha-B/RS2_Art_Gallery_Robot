#include <ros/ros.h>
#include <ros/package.h>
#include <ros/master.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <random>
#include <fstream>
#include <iostream>
#include <cstdlib>
#include <sstream>
#include <cmath>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// Global Variables
std::string logFilePath = "odometry_data.csv";
std::float_t samplePeriod_ = 0.1; // time period between each sample saved to the csv file

std::string filtered_node_name_; //= "/robot_pose_ekf/odom_combined";

ros::Subscriber filtered_odom_subscriber;
std::ofstream csv_file;
std::stringstream filteredData_, rawOdomData_, NoisyOdomData_;

// Function Prototypes
bool checkFilteredTopicAvailability();
void FilteredOdomCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &filtered_odom_msg);
void noisyOdomCallback(const nav_msgs::Odometry::ConstPtr &odom_msg);
void odomCallback(const nav_msgs::Odometry::ConstPtr &odom_msg);
void writeDataToCSV(const ros::TimerEvent &);
std::string getFilterTopic();

int main(int argc, char **argv)
{
    // ROS Initialization
    ros::init(argc, argv, "noise_and_filter_recorder");
    ros::NodeHandle nh;

    // Open CSV File
    csv_file.open(logFilePath);
    if (!csv_file.is_open())
    {
        ROS_ERROR("Failed to open the CSV file for writing.");
        return 1;
    }
    csv_file << "Original_X,Original_Y,Original_Z,Noisy_X,Noisy_Y,Noisy_Z,Filtered_X,Filtered_Y,Filtered_Z\n";


    // ask for filter topic node name
    filtered_node_name_ = getFilterTopic();


    // Check Topic Availability and Subscribe
    if (checkFilteredTopicAvailability())
    {
        filtered_odom_subscriber = nh.subscribe(filtered_node_name_, 10, FilteredOdomCallback);
    }

    // Subscribe to Odometry and Advertise Noisy Odometry
    ros::Subscriber odom_subscriber = nh.subscribe("/odom", 10, odomCallback);
    ros::Subscriber noisy_odom_subscriber = nh.subscribe("/noisy_odom", 10, noisyOdomCallback);

    // Timer for Writing Data to CSV
    ros::Timer timer = nh.createTimer(ros::Duration(samplePeriod_), writeDataToCSV);

    // ROS Spin
    ros::spin();
    return 0;
}

std::string getFilterTopic()
{
    std::string nodeName;
    std::cout << "Please enter the filter topic node (eg: /odom): ";
    std::cin >> nodeName;
    return nodeName;
}

bool checkFilteredTopicAvailability()
{
    std::vector<ros::master::TopicInfo> topic_infos;
    bool found_ = false;
    if (ros::master::getTopics(topic_infos))
    {
        for (const auto &info : topic_infos)
        {
            if (info.name == filtered_node_name_)
            {
                found_ = true;
                break;
            }
        }
        if (!found_)
        {
            ROS_INFO("Topic %s not found, writing to file anyway...", filtered_node_name_.c_str());
            filteredData_ << "-"
                          << ","
                          << "-"
                          << ","
                          << "-";
        }
        else
        {
            ROS_INFO("Writing to .csv file...");
        }
    }
    return found_;
}

void FilteredOdomCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &filtered_odom_msg)
{
    filteredData_.str("");
    filteredData_.clear();
    filteredData_ << filtered_odom_msg->pose.pose.position.x << ","
                  << filtered_odom_msg->pose.pose.position.y << ","
                  << filtered_odom_msg->pose.pose.position.z;
}

void noisyOdomCallback(const nav_msgs::Odometry::ConstPtr &noisy_odom_msg)
{
    NoisyOdomData_.str("");
    NoisyOdomData_.clear();
    NoisyOdomData_ << noisy_odom_msg->pose.pose.position.x << ","
                   << noisy_odom_msg->pose.pose.position.y << ","
                   << noisy_odom_msg->pose.pose.position.z;
}

void odomCallback(const nav_msgs::Odometry::ConstPtr &odom_msg)
{
    rawOdomData_.str("");
    rawOdomData_.clear();
    rawOdomData_ << odom_msg->pose.pose.position.x << ","
                 << odom_msg->pose.pose.position.y << ","
                 << odom_msg->pose.pose.position.z;
}

void writeDataToCSV(const ros::TimerEvent &)
{
    if (!rawOdomData_.str().empty() && !NoisyOdomData_.str().empty() && !filteredData_.str().empty())
    {
        csv_file << rawOdomData_.str() << "," << NoisyOdomData_.str() << "," << filteredData_.str() << "\n";
        csv_file.flush();
    }
    else
    {
        ROS_INFO("Waiting for all data to become available");
    }
}
