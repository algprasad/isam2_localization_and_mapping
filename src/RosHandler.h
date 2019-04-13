//
// Created by socrob on 09-04-2019.
//

#ifndef ISAM2_ROSHANDLER_H
#define ISAM2_ROSHANDLER_H

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/PoseStamped.h>
#include "InitialParameters.h"

//visualization_msgs
#include <visualization_msgs/Marker.h>
#include <opencv-3.3.1-dev/opencv/cxeigen.hpp>

//tf include files
#include <tf/transform_broadcaster.h>


class RosHandler {
private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_imu_;
    ros::Subscriber sub_image_;
    ros::Subscriber sub_odometry_;
    ros::Publisher rviz_marker_corner_pub_;
    tf::TransformBroadcaster broadcast_pose_;

//values being received from the rostopics
public:

    sensor_msgs::Imu ros_imu_;
    sensor_msgs::Image ros_image_ ;
    geometry_msgs::PoseStamped ros_odometry_;
    cv::Mat current_image_;


    //to check if new values are being receieved
    bool new_imu_;
    bool new_image_;
    bool new_odometry_;


public:
    //Default constructor
    RosHandler(int argc, char** argv);

    void publishMarkerCorners(visualization_msgs::Marker points);
    void publishCurrentTFPose(tf::Transform current_pose_transform);

    //accessor functions;
    sensor_msgs::Imu getRosImu();
    sensor_msgs::Image getRosImage();
    geometry_msgs::PoseStamped getRosOdometry();

    //method to reset all boolean values to make sure new values are being used.
    void resetBools();


private:
    void imuCallBack(const sensor_msgs::Imu::ConstPtr& msg);
    void imageCallBack(const sensor_msgs::ImageConstPtr& msg);
    void odometryCallBack(const geometry_msgs::PoseStampedConstPtr &msg);


};


#endif //ISAM2_ROSHANDLER_H
