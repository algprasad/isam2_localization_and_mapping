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

    unsigned int _pose_number;   //Pose nyumber in the graph
    unsigned int _lindex_tl;  //landmark index of the top left corner in the graph

    //Measured pixel coordinates
    gtsam::Point2 _measured_tl, _measured_tr, _measured_br, _measured_bl;
    gtsam::Point2 _cal_tl, _cal_tr, _cal_br, _cal_bl;

    ///the pose of the robot and the position of the landmark will be used to caluclate the position of the pixel corrdinates
    /// and compard with the measured values
    gtsam::Pose3 _pose_val;  // pose of the robot at that instance when factor was captured which will be opimised as the
    gtsam::Point3 _tl_position, _tr_position, _br_position, _bl_position;

    //Store the image for drawing the pixels
    cv::Mat _img;
    cv::Mat _img_drawn;

    //caliration parameters
    boost::shared_ptr<Cal3_S2> _K;





public:

    VisualizationCorners(){

    }

    VisualizationCorners(unsigned int pose_num, unsigned int tl_index, Point2 meas_tl, Point2 meas_tr,
            Point2 meas_br, Point2 meas_bl, cv::Mat image, const boost::shared_ptr<Cal3_S2>& K ){
        _pose_number = pose_num;
        _lindex_tl = tl_index;
        _img = image;

        //adding the  measured pixel values
        _measured_tl = meas_tl;
        _measured_tr = meas_tr;
        _measured_br = meas_br;
        _measured_bl = meas_bl;

        //calibration values
        _K = K;

    }

    //the values will be assigned by gettting the pose number and sending the pose values using the index from the graph.
    void calculatePredicted(Pose3 assign_pose, Point3 assign_point_tl, Point3 assign_point_tr, Point3 assign_point_br, Point3 assign_point_bl ){
        // this is where the predicted pixel values are calculated

        //assigning the calculated pose from the graph
        _pose_val = assign_pose;
        std::cout<<"The pose value is "<<_pose_val.translation()<<std::endl;



        //assigning the calculated position of points from the graph
        _tl_position = assign_point_tl;
        _tr_position = assign_point_tr;
        _br_position = assign_point_br;
        _bl_position = assign_point_bl;

        std::cout<<"The TL point value is: "<<_tl_position.x()<<" "<<_tl_position.y()<<" "<<_tl_position.z()<<std::endl;

        //Parameters to add in the project function
        boost::optional<Matrix&> H1 = boost::none;
        boost::optional<Matrix&> H2 = boost::none;


        //The camera object
        PinholeCamera<Cal3_S2> camera(_pose_val, *_K); //Need to get the calibration values

        //projecting the tl;
        _cal_tl = camera.project(_tl_position, boost::none, boost::none, boost::none);  // you could do this without the H1 and H2.. jsut by passing the boost::none
        _cal_tr = camera.project(_tr_position, boost::none, boost::none, boost::none);
        _cal_br = camera.project(_br_position, boost::none, boost::none, boost::none);
        _cal_bl = camera.project(_bl_position, boost::none, boost::none, boost::none);

    }


    cv::Mat drawOnImg(){

        cv::Mat rgb;
        cvtColor(_img, rgb, CV_GRAY2BGR);

        /***************************** Measured Values**********************************************************/
        cv::line(rgb, cv::Point(_measured_tl.x(), _measured_tl.y()), cv::Point(_measured_tr.x(), _measured_tr.y()), cv::Scalar(0, 240, 0), 2, 8);
        cv::line(rgb, cv::Point(_measured_tr.x(), _measured_tr.y()), cv::Point(_measured_br.x(), _measured_br.y()), cv::Scalar(0, 240, 0), 2, 8);
        cv::line(rgb, cv::Point(_measured_br.x(), _measured_br.y()), cv::Point(_measured_bl.x(), _measured_bl.y()), cv::Scalar(0, 240, 0), 2, 8);
        cv::line(rgb, cv::Point(_measured_bl.x(), _measured_bl.y()), cv::Point(_measured_tl.x(), _measured_tl.y()), cv::Scalar(0, 240, 0), 2, 8);


        /****************************Predicted values***************************************************************/
        cv::line(rgb, cv::Point(_cal_tl.x(), _cal_tl.y()), cv::Point(_cal_tr.x(), _cal_tr.y()), cv::Scalar(0, 0, 240), 2, 8);
        cv::line(rgb, cv::Point(_cal_tr.x(), _cal_tr.y()), cv::Point(_cal_br.x(), _cal_br.y()), cv::Scalar(0, 0, 240), 2, 8);
        cv::line(rgb, cv::Point(_cal_br.x(), _cal_br.y()), cv::Point(_cal_bl.x(), _cal_bl.y()), cv::Scalar(0, 0, 240), 2, 8);
        cv::line(rgb, cv::Point(_cal_bl.x(), _cal_bl.y()), cv::Point(_cal_tl.x(), _cal_tl.y()), cv::Scalar(0, 0, 240), 2, 8);

        _img_drawn = rgb;
        return rgb;


    }


    unsigned int getPoseIndex(){
        return _pose_number;

    }

    unsigned int getTLindex(){
        return _lindex_tl;
    }


    ~VisualizationCorners(){

    }


};


#endif //ISAM_ROS_VISUALIZATIONCORNERS_H
