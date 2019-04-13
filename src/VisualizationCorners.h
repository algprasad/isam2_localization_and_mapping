//
// Created by socrob on 09-01-2019.
//

#ifndef ISAM_ROS_VISUALIZATIONCORNERS_H
#define ISAM_ROS_VISUALIZATIONCORNERS_H

#include <opencv2/opencv.hpp>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>
#include <iostream>

#include <gtsam/geometry/SimpleCamera.h>

//Opencv files for drawing
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>


using namespace gtsam;

//This is a class to visualize the factors on the image space
class VisualizationCorners {
private:

    unsigned int pose_number_; //Pose nyumber in the graph
    unsigned int lindex_tl_; //landmark index of the top left corner in the graph

    //Measured pixel coordinates
    gtsam::Point2 measured_tl_, measured_tr_, measured_br_, measured_bl_;
    gtsam::Point2 cal_tl_, cal_tr_, cal_br_, cal_bl_;
    

    ///the pose of the robot and the position of the landmark will be used to caluclate the position of the pixel corrdinates
    /// and compard with the measured values
    gtsam::Pose3 pose_val_;// pose of the robot at that instance when factor was captured which will be opimised as the
    gtsam::Point3 tl_position_, tr_position_, br_position_, bl_position_;
    

    //Store the image for drawing the pixels
    cv::Mat img_;
    cv::Mat img_drawn_;

    //caliration parameters
    boost::shared_ptr<Cal3_S2> K_;
    





public:
    //Default Constructor
    VisualizationCorners(){}

    VisualizationCorners(unsigned int pose_num, unsigned int tl_index, Point2 meas_tl, Point2 meas_tr,
            Point2 meas_br, Point2 meas_bl, cv::Mat image, const boost::shared_ptr<Cal3_S2>& K );


    void calculatePredicted(Pose3 assign_pose, Point3 assign_point_tl, Point3 assign_point_tr, Point3 assign_point_br, Point3 assign_point_bl );

    cv::Mat drawOnImg();

    unsigned int getPoseIndex();

    unsigned int getTLindex();

    ~VisualizationCorners(){}


};


#endif //ISAM_ROS_VISUALIZATIONCORNERS_H
