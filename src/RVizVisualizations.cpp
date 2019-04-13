//
// Created by socrob on 09-04-2019.
//

#include "RVizVisualizations.h"
using symbol_shorthand::L; // Landmark
using symbol_shorthand::X;

void RVizVisualizations::assign_corner_position(Values current_estimate, unsigned int max_corners) {

    for(unsigned int i = 0; i< max_corners; i++){
        geometry_msgs::Point temp_corner_position;
        temp_corner_position.x = current_estimate.at<Point3>(L(i)).x();
        temp_corner_position.y = current_estimate.at<Point3>(L(i)).y();
        temp_corner_position.z = current_estimate.at<Point3>(L(i)).z();


        std_msgs::ColorRGBA color;
        setCornerColor(color, i);


        corner_points_.points.push_back(temp_corner_position);
        corner_points_.colors.push_back(color);

    }

}

void RVizVisualizations::assign_current_pose(Pose3 current_pose) {

    pose_transform_.setOrigin(tf::Vector3(current_pose.translation()[0],
                                          current_pose.translation()[1],
                                          current_pose.translation()[2]));

    tf::Quaternion pose_quaternion(current_pose.rotation().toQuaternion().x(),
                                   current_pose.rotation().toQuaternion().y(),
                                   current_pose.rotation().toQuaternion().z(),
                                   current_pose.rotation().toQuaternion().w());

    pose_transform_.setRotation(pose_quaternion);

}


void RVizVisualizations::setCornerColor(std_msgs::ColorRGBA& color, int i) {


    //TODO(ALG): Use a swtich case for this
    //landmark#1 is green
    if(i >= 0 && i<4)
        color.g = 1.0;
    //landmark #2 is red
    if(i>=4 && i< 8)
        color.r = 1.0;

    //landmark#3 is blue
    if(i>=8 && i < 12)
        color.b = 1.0;

    //landmark #4 is light orange
    if(i>=12 && i < 16) {
        color.r = 0.5;
        color.g = 0.5;
    }

    //landmark#5 is purple
    if(i >=16 && i< 20) {
        color.g = 0.8;
        color.b = 0.8;
    }

    //landmark#6 is red + blue
    if(i>= 20 && i <24){
        color.r = 0.8;
        color.b = 0.8;
    }

    //landmark #7 is RED + green
    if (i>= 24 && i < 28){
        color.r= 0.9;
        color.g = 0.45;
    }

    color.a = 1;

}