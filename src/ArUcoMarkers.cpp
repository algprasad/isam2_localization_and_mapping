//
// Created by socrob on 09-04-2019.
//

#include "ArUcoMarkers.h"

bool ArUcoMarkers::hasMarkers(cv::Mat current_image) {
    aruco::MarkerDetector marker_detector;
    marker_detector.detect(current_image, markers_, camera_parameters_, MARKER_SIZE);

    return !markers_.empty();

}

Eigen::VectorXd ArUcoMarkers::getCornerPoints(){

}