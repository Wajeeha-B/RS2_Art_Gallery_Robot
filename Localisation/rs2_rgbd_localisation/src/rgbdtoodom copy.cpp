#include <ros/ros.h>
#include <apriltag_ros/AprilTagDetectionArray.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Quaternion.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

ros::Publisher odom_pub;
sensor_msgs::Imu global_imu;

geometry_msgs::Pose convertToBaseLinkFrame(const geometry_msgs::Pose& input_pose, const std::string& frame_id)
{
    // Initialize a TF listener
    tf::TransformListener listener;

    // Create a TransformStamped message to store the transformation
    tf::StampedTransform transform;

    try
    {
        // Look up the transformation from the specified frame to the base_link frame
        listener.waitForTransform("/base_link",frame_id,ros::Time(0),ros::Duration(4.0));
        listener.lookupTransform("/base_link", frame_id, ros::Time(0), transform);

        // Convert the input_pose to a TF pose
        tf::Pose input_tf_pose;
        tf::poseMsgToTF(input_pose, input_tf_pose);

        // Apply the transformation to convert to the base_link frame
        tf::Pose base_link_tf_pose = transform * input_tf_pose;

        // Convert the result back to a geometry_msgs::Pose
        geometry_msgs::Pose base_link_pose;
        tf::poseTFToMsg(base_link_tf_pose, base_link_pose);

        return base_link_pose;
    }
    catch(tf::TransformException& ex)
    {
        ROS_ERROR("Transform error: %s", ex.what());
        // Return an invalid pose or handle the error as needed
        return input_pose;
    }
}

void imuCallback(const sensor_msgs::Imu::ConstPtr& imu)
{
    global_imu = *imu;
}

geometry_msgs::Pose getYawCoordinates(double yaw, geometry_msgs::Pose pose) {
    // Specify the radius (distance from the origin)
    double radius = sqrt(pow(pose.position.x,2) + pow(pose.position.y,2));

    // Calculate x and y coordinates based on yaw angle
    geometry_msgs::Pose conv_pose;
    conv_pose.position.x = radius * cos(yaw);
    conv_pose.position.y = radius * sin(yaw);

    return conv_pose;
}

double quaternionToYaw(const geometry_msgs::Quaternion& quat) {
    tf2::Quaternion tf_quat(quat.x, quat.y, quat.z, quat.w);

    // Convert the quaternion to a 3x3 rotation matrix
    tf2::Matrix3x3 mat(tf_quat);

    // Extract the yaw (rotation around the z-axis) from the matrix
    double roll, pitch, yaw;
    mat.getRPY(roll, pitch, yaw);

    return yaw;
}

void detectedtagCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr& detection_msg)
{
    // Process tag detections and create an Odometry message
    nav_msgs::Odometry odom_msg;

    // Establish ground truth (Global coordinates of april tags)
    geometry_msgs::Point tag0, tag1, tag3, tag4, tag5, global, odom;
    tf::pointTFToMsg(tf::Vector3(-1.955090, 3.283930, 0.505621), tag0);
    tf::pointTFToMsg(tf::Vector3(3.618770, 3.480450, 0.480635), tag1);
    tf::pointTFToMsg(tf::Vector3(-1.788420, 6.437140, 0.491476), tag3);
    tf::pointTFToMsg(tf::Vector3(3.294030, 3.533260, 0.466723), tag4);
    tf::pointTFToMsg(tf::Vector3(-1.380480, 13.830400, 0.502502), tag5);

    for(auto it = detection_msg->detections.begin(); it != detection_msg->detections.end(); ++it)
    {
        // Calculate Global coordinates of camera from april tag
        switch ((*it).id.at(0))
        {
        case 0:
            global = tag0;
            break;
        case 1:
            global = tag1;
            break;
        case 3:
            global = tag3;
            break;
        case 4:
            global = tag4;
            break;
        case 5:
            global = tag5;
            break;
        }

        geometry_msgs::Pose conv_pose;

        conv_pose = convertToBaseLinkFrame((*it).pose.pose.pose,(*it).pose.header.frame_id);
        conv_pose = getYawCoordinates(quaternionToYaw(global_imu.orientation),conv_pose);

        odom.x += global.x - conv_pose.position.x;
        odom.y += global.y - conv_pose.position.y;
        odom.z += global.z - conv_pose.position.z;

    // Debugging Lines
    // ROS_INFO_STREAM("detected X: " + std::to_string(global.x - (*it).pose.pose.pose.position.x)+ " Y: " + std::to_string(global.y - (*it).pose.pose.pose.position.y) + " Z: " + std::to_string(global.z - (*it).pose.pose.pose.position.z));
    // ROS_INFO_STREAM("detected X: " + std::to_string(global.x - conv_pose.position.x)+ " Y: " + std::to_string(global.y - conv_pose.position.y) + " Z: " + std::to_string(global.z - conv_pose.position.z));
    // ROS_INFO_STREAM("detected X: " + std::to_string((*it).pose.pose.pose.position.x)+ " Y: " + std::to_string((*it).pose.pose.pose.position.y) + " Z: " + std::to_string((*it).pose.pose.pose.position.z));
    // ROS_INFO_STREAM("detected X: " + std::to_string(conv_pose.position.x)+ " Y: " + std::to_string(conv_pose.position.y) + " Z: " + std::to_string(conv_pose.position.z));
    // ROS_INFO_STREAM("global X: " + std::to_string(global.x)+ " Y: " + std::to_string(global.y) + " Z: " + std::to_string(global.z));
    }

    odom_msg.pose.pose.position.x = odom.x/detection_msg->detections.size();
    odom_msg.pose.pose.position.y = odom.y/detection_msg->detections.size();
    odom_msg.pose.pose.position.z = odom.z/detection_msg->detections.size();

    ROS_INFO_STREAM("avg X: " + std::to_string(odom_msg.pose.pose.position.x)+ " Y: " + std::to_string(odom_msg.pose.pose.position.y) + " Z: " + std::to_string(odom_msg.pose.pose.position.z));
    
    // Publish the Odometry message
    odom_pub.publish(odom_msg);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "tag_detection_node");
    ros::NodeHandle nh;

    ros::Subscriber tag_detection_sub = nh.subscribe("tag_detections", 10, detectedtagCallback);
    ros::Subscriber imu_sub = nh.subscribe("/imu", 10, imuCallback);
    ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("/rgbd_odom", 10);

    ros::spin();

    return 0;
}