//
// Created by socrob on 09-04-2019.
//

#ifndef ISAM2_ROSHANDLER_H
#define ISAM2_ROSHANDLER_H

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/PoseStamped.h>

class RosHandler {
private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_imu_;
    ros::Subscriber sub_image_;
    ros::Subscriber sub_odometry_;

public:
    //Default constructor
    RosHandler();

    //publish topic
    int publish();

private:
    void imuCallBack(const sensor_msgs::Imu::ConstPtr& msg);
    void imageCallBack(const sensor_msgs::ImageConstPtr& msg);
    void odometryCallBack(const geometry_msgs::PoseStampedConstPtr &msg);

};


#endif //ISAM2_ROSHANDLER_H
