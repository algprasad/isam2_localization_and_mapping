//
// Created by socrob on 09-01-2019.
//

#include "VisualizationCorners.h"

VisualizationCorners::VisualizationCorners(unsigned int pose_num, unsigned int tl_index, Point2 meas_tl, Point2 meas_tr,
                     Point2 meas_br, Point2 meas_bl, cv::Mat image, const boost::shared_ptr<Cal3_S2>& K ){
    pose_number_ = pose_num;
    lindex_tl_ = tl_index;
    img_ = image;

    //adding the  measured pixel values
    measured_tl_ = meas_tl;
    measured_tr_ = meas_tr;
    measured_br_ = meas_br;
    measured_bl_ = meas_bl;

    //calibration values
    K_ = K;

}

//the values will be assigned by gettting the pose number and sending the pose values using the index from the graph.
void VisualizationCorners::calculatePredicted(Pose3 assign_pose, Point3 assign_point_tl, Point3 assign_point_tr, Point3 assign_point_br, Point3 assign_point_bl ){
    // this is where the predicted pixel values are calculated

    //assigning the calculated pose from the graph
    pose_val_ = assign_pose;
    std::cout<<"The pose value is "<<pose_val_.translation()<<std::endl;



    //assigning the calculated position of points from the graph
    tl_position_ = assign_point_tl;
    tr_position_ = assign_point_tr;
    br_position_ = assign_point_br;
    bl_position_ = assign_point_bl;

    std::cout<<"The TL point value is: "<<tl_position_.x()<<" "<<tl_position_.y()<<" "<<tl_position_.z()<<std::endl;

    //Parameters to add in the project function
    boost::optional<Matrix&> H1 = boost::none;
    boost::optional<Matrix&> H2 = boost::none;


    //The camera object
    PinholeCamera<Cal3_S2> camera(pose_val_, *K_); //Need to get the calibration values

    //projecting the tl;
    cal_tl_ = camera.project(tl_position_, boost::none, boost::none, boost::none);  // you could do this without the H1 and H2.. jsut by passing the boost::none
    cal_tr_ = camera.project(tr_position_, boost::none, boost::none, boost::none);
    cal_br_ = camera.project(br_position_, boost::none, boost::none, boost::none);
    cal_bl_ = camera.project(bl_position_, boost::none, boost::none, boost::none);

}


cv::Mat VisualizationCorners::drawOnImg(){

    cv::Mat rgb;
    cvtColor(img_, rgb, CV_GRAY2BGR);

    /***************************** Measured Values**********************************************************/
    cv::line(rgb, cv::Point(measured_tl_.x(), measured_tl_.y()), cv::Point(measured_tr_.x(), measured_tr_.y()), cv::Scalar(0, 240, 0), 2, 8);
    cv::line(rgb, cv::Point(measured_tr_.x(), measured_tr_.y()), cv::Point(measured_br_.x(), measured_br_.y()), cv::Scalar(0, 240, 0), 2, 8);
    cv::line(rgb, cv::Point(measured_br_.x(), measured_br_.y()), cv::Point(measured_bl_.x(), measured_bl_.y()), cv::Scalar(0, 240, 0), 2, 8);
    cv::line(rgb, cv::Point(measured_bl_.x(), measured_bl_.y()), cv::Point(measured_tl_.x(), measured_tl_.y()), cv::Scalar(0, 240, 0), 2, 8);


    /****************************Predicted values***************************************************************/
    cv::line(rgb, cv::Point(cal_tl_.x(), cal_tl_.y()), cv::Point(cal_tr_.x(), cal_tr_.y()), cv::Scalar(0, 0, 240), 2, 8);
    cv::line(rgb, cv::Point(cal_tr_.x(), cal_tr_.y()), cv::Point(cal_br_.x(), cal_br_.y()), cv::Scalar(0, 0, 240), 2, 8);
    cv::line(rgb, cv::Point(cal_br_.x(), cal_br_.y()), cv::Point(cal_bl_.x(), cal_bl_.y()), cv::Scalar(0, 0, 240), 2, 8);
    cv::line(rgb, cv::Point(cal_bl_.x(), cal_bl_.y()), cv::Point(cal_tl_.x(), cal_tl_.y()), cv::Scalar(0, 0, 240), 2, 8);

    img_drawn_ = rgb;
    return rgb;


}


unsigned int VisualizationCorners::getPoseIndex(){
    return pose_number_;

}

unsigned int VisualizationCorners::getTLindex(){
    return lindex_tl_;
}