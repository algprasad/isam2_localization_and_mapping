//
// Created by socrob on 09-04-2019.
//

#ifndef ISAM2_RVIZVISUALIZATIONS_H
#define ISAM2_RVIZVISUALIZATIONS_H

#include <ros/ros.h>
//visualization_msgs
#include <visualization_msgs/Marker.h>
#include <opencv-3.3.1-dev/opencv/cxeigen.hpp>


class RVizVisualizations {
public:

    //for visualizing point of the landmark
    visualization_msgs::Marker points_;



public:
    RVizVisualizations(){

        points_.header.frame_id = "/world";
        points_.header.stamp = ros::Time::now();
        points_.ns = "points";
        points_.action = visualization_msgs::Marker::ADD;
        points_.pose.orientation.w = 1.0;
        points_.id = 0;
        points_.type = visualization_msgs::Marker::POINTS;


        // POINTS markers use x and y scale for width/height respectively
        points_.scale.x = 0.05;
        points_.scale.y = 0.05;
        // Points are green
        points_.color.g = 1.0f;
        points_.color.a = 1.0;
    }



};


#endif //ISAM2_RVIZVISUALIZATIONS_H
