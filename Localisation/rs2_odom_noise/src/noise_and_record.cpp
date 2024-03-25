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
std::string filtered_node_name_; //= "/robot_pose_ekf/odom_combined";
std::float_t samplePeriod_ = 0.1; // time period between each sample saved to the csv file
ros::Publisher noisy_odom_publisher;
ros::Subscriber filtered_odom_subscriber;
std::ofstream csv_file;
std::stringstream filteredData_, rawOdomData_, NoisyOdomData_;

// Function Prototypes
bool checkFilteredTopicAvailability();
double getRandomNoise();
void FilteredOdomCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &filtered_odom_msg);
void odomCallback(const nav_msgs::Odometry::ConstPtr &odom_msg);
void writeDataToCSV(const ros::TimerEvent &);
std::string getFilterTopic();

int main(int argc, char **argv)
{
    // ROS Initialization
    ros::init(argc, argv, "noisy_odometry_publisher");
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
    noisy_odom_publisher = nh.advertise<nav_msgs::Odometry>("/noisy_odom", 10);

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
            ROS_INFO("Topic /odom_filtered not found, writing to file anyway...");
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

double getRandomNoise()
{
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> dis(-0.05, 0.05);
    return dis(gen);
}

void FilteredOdomCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &filtered_odom_msg)
{
    filteredData_.str("");
    filteredData_.clear();
    filteredData_ << filtered_odom_msg->pose.pose.position.x << ","
                  << filtered_odom_msg->pose.pose.position.y << ","
                  << filtered_odom_msg->pose.pose.position.z;
}

void odomCallback(const nav_msgs::Odometry::ConstPtr &odom_msg)
{
    nav_msgs::Odometry noisy_odom;

    noisy_odom = *odom_msg; // copy across first

    noisy_odom.pose.pose.position.x = odom_msg->pose.pose.position.x * (1 + getRandomNoise());
    noisy_odom.pose.pose.position.y = odom_msg->pose.pose.position.y * (1 + getRandomNoise());
    noisy_odom.pose.pose.position.z = odom_msg->pose.pose.position.z * (1 + getRandomNoise());

    noisy_odom.pose.pose.orientation.x = odom_msg->pose.pose.orientation.x * (1 + getRandomNoise());
    noisy_odom.pose.pose.orientation.y = odom_msg->pose.pose.orientation.y * (1 + getRandomNoise());
    noisy_odom.pose.pose.orientation.z = odom_msg->pose.pose.orientation.z * (1 + getRandomNoise());


    // converting quaternion to roll pitch yaw for covariance matrix
    tf2::Quaternion q(
        odom_msg->pose.pose.orientation.x,
        odom_msg->pose.pose.orientation.y,
        odom_msg->pose.pose.orientation.z,
        odom_msg->pose.pose.orientation.w);

    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    // Assuming you have angular uncertainty in roll, pitch, yaw
    float variance_roll = std::pow((5.0 / 100.0) * roll, 2);
    float variance_pitch = std::pow((5.0 / 100.0) * pitch, 2);
    float variance_yaw = std::pow((5.0 / 100.0) * yaw, 2);

    // Calculate variances based on odom_msg mean measurements
    float variance_x = std::pow((5.0 / 100.0) * odom_msg->pose.pose.position.x, 2);
    float variance_y = std::pow((5.0 / 100.0) * odom_msg->pose.pose.position.y, 2);
    float variance_z = std::pow((5.0 / 100.0) * odom_msg->pose.pose.position.z, 2);

    // Set the covariance matrix
    noisy_odom.pose.covariance[0] = variance_x;      // Variance in x
    noisy_odom.pose.covariance[7] = variance_y;      // Variance in y
    noisy_odom.pose.covariance[14] = variance_z;     // Variance in z
    noisy_odom.pose.covariance[21] = variance_roll;  // Variance in roll
    noisy_odom.pose.covariance[28] = variance_pitch; // Variance in pitch
    noisy_odom.pose.covariance[35] = variance_yaw;   // Variance in yaw

    noisy_odom_publisher.publish(noisy_odom);

    rawOdomData_.str("");
    rawOdomData_.clear();
    NoisyOdomData_.str("");
    NoisyOdomData_.clear();

    rawOdomData_ << odom_msg->pose.pose.position.x << ","
                 << odom_msg->pose.pose.position.y << ","
                 << odom_msg->pose.pose.position.z;

    NoisyOdomData_ << noisy_odom.pose.pose.position.x << ","
                   << noisy_odom.pose.pose.position.y << ","
                   << noisy_odom.pose.pose.position.z;
}

void writeDataToCSV(const ros::TimerEvent &)
{
    if (!rawOdomData_.str().empty() && !NoisyOdomData_.str().empty() && !filteredData_.str().empty())
    {
        csv_file << rawOdomData_.str() << "," << NoisyOdomData_.str() << "," << filteredData_.str() << "\n";
        csv_file.flush();
    }
}
