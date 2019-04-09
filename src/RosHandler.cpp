//
// Created by socrob on 09-04-2019.
//

#include "RosHandler.h"

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/PoseStamped.h>


RosHandler::RosHandler( ): nh_("~"){
    sub_imu_ = nh_.subscribe( "/mavros/imu/data",
                                1,
                                &RosHandler::imuCallBack,
                                this);
    sub_image_ = nh_.subscribe( "/camera/image_rect_color",
                                1,
                                &RosHandler::imageCallBack,
                                this);

    sub_image_ = nh_.subscribe( "/svo/pose_cam/0",
                                1,
                                &RosHandler::odometryCallBack,
                                this);

}


//TODO(ALG): Define all the callBack functions
void RosHandler::imuCallBack(const sensor_msgs::Imu::ConstPtr &msg) {

}

void RosHandler::imageCallBack(const sensor_msgs::ImageConstPtr &msg) {

}

void RosHandler::odometryCallBack(const geometry_msgs::PoseStampedConstPtr &msg) {


}