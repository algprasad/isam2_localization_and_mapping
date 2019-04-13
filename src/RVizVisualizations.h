//
// Created by socrob on 09-04-2019.
//

#ifndef ISAM2_RVIZVISUALIZATIONS_H
#define ISAM2_RVIZVISUALIZATIONS_H

#include <ros/ros.h>
//visualization_msgs
#include <visualization_msgs/Marker.h>
//#include <opencv-3.3.1-dev/opencv/cxeigen.hpp>
#include "InitialParameters.h"
//tf include files
#include <tf/transform_broadcaster.h>


class RVizVisualizations {
public:

    //for publishing transforms in rviz

    tf::Transform pose_transform_;

    //for visualizing point of the landmark
    visualization_msgs::Marker corner_points_;




public:
    RVizVisualizations(){

        corner_points_.header.frame_id = "/world";
        corner_points_.header.stamp = ros::Time::now();
        corner_points_.ns = "points";
        corner_points_.action = visualization_msgs::Marker::ADD;
        corner_points_.pose.orientation.w = 1.0;
        corner_points_.id = 0;
        corner_points_.type = visualization_msgs::Marker::POINTS;


        // POINTS markers use x and y scale for width/height respectively
        corner_points_.scale.x = 0.05;
        corner_points_.scale.y = 0.05;
        // Points are green
        corner_points_.color.g = 1.0f;
        corner_points_.color.a = 1.0;
    }

    void assign_corner_position(Values current_estimate, unsigned int max_corners);
    void assign_current_pose(Pose3 current_pose);
    void setCornerColor(std_msgs::ColorRGBA& color, int i);



};




#endif //ISAM2_RVIZVISUALIZATIONS_H
