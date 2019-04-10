//
// Created by socrob on 09-04-2019.
//

#ifndef ISAM2_ARUCOMARKERS_H
#define ISAM2_ARUCOMARKERS_H

#include <aruco.h>
#include "Algutils.h"
#include "InitialParameters.h"
#include <iostream>
#include <vector>

class ArUcoMarkers {
public:

    //constants
    aruco::CameraParameters camera_parameters_;
    cv::Mat distortion_coefficient_;
    cv::Mat camera_matrix_;
    //float marker_size_ = MARKER_SIZE;
    aruco::MarkerDetector MDetector;
    std::vector<aruco::Marker> markers_;
    Eigen::VectorXd corner_points_;

    ArUcoMarkers(){
        camera_parameters_.readFromXMLFile(XML_FILE_PATH);
        distortion_coefficient_ = camera_parameters_.Distorsion;
        camera_matrix_ = camera_parameters_.CameraMatrix;

    }

public:

    aruco::CameraParameters getCameraParameters(){
        return camera_parameters_;
    }

    bool hasMarkers(cv::Mat current_image);

    //TODO(ALG): This should be run for all the markers detected
    Eigen::VectorXd getCornerPoints();








};


#endif //ISAM2_ARUCOMARKERS_H
