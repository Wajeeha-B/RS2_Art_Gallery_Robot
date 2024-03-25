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
std::string filtered_node_name_ = "/robot_pose_ekf/odom_combined";
ros::Publisher noisy_odom_publisher;
std::stringstream filteredData_, rawOdomData_, NoisyOdomData_;
double noise_var = 0.05;

// Function Prototypes
double getRandomNoise();
void odomCallback(const nav_msgs::Odometry::ConstPtr &odom_msg);

int main(int argc, char **argv)
{
    // ROS Initialization
    ros::init(argc, argv, "noisy_odometry_publisher");
    ros::NodeHandle nh;
    
    // Subscribe to Odometry and Advertise Noisy Odometry
    ros::Subscriber odom_subscriber = nh.subscribe("/odom", 10, odomCallback);
    noisy_odom_publisher = nh.advertise<nav_msgs::Odometry>("/noisy_odom", 10);

    // ROS Spin
    ros::spin();
    return 0;
}

double getRandomNoise()
{
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> dis(-noise_var, noise_var);
    return dis(gen);
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
    float variance_roll = std::pow((noise_var) * roll, 2);
    float variance_pitch = std::pow((noise_var) * pitch, 2);
    float variance_yaw = std::pow((noise_var) * yaw, 2);

    // Calculate variances based on odom_msg mean measurements
    float variance_x = std::pow((noise_var) * odom_msg->pose.pose.position.x, 2);
    float variance_y = std::pow((noise_var) * odom_msg->pose.pose.position.y, 2);
    float variance_z = std::pow((noise_var) * odom_msg->pose.pose.position.z, 2);

    // Set the covariance matrix
    noisy_odom.pose.covariance[0] = variance_x;      // Variance in x
    noisy_odom.pose.covariance[7] = variance_y;      // Variance in y
    noisy_odom.pose.covariance[14] = variance_z;     // Variance in z
    noisy_odom.pose.covariance[21] = variance_roll;  // Variance in roll
    noisy_odom.pose.covariance[28] = variance_pitch; // Variance in pitch
    noisy_odom.pose.covariance[35] = variance_yaw;   // Variance in yaw

    noisy_odom_publisher.publish(noisy_odom);
}
