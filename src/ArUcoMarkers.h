//
// Created by socrob on 09-04-2019.
//

#ifndef ISAM2_ARUCOMARKERS_H
#define ISAM2_ARUCOMARKERS_H

#include <aruco.h>
#include "Algutils.h"

class ArUcoMarkers {
public:

    //constants
    aruco::CameraParameters camera_parameters_;
    cv::Mat distortion_coefficient_;
    cv::Mat camera_matrix_;
    //float marker_size_ = MARKER_SIZE;
    aruco::MarkerDetector MDetector;

    ArUcoMarkers(){
        camera_parameters_.readFromXMLFile(XML_FILE_PATH);
        distortion_coefficient_ = camera_parameters.Distorsion;
        camera_matrix_ = camera_parameters.CameraMatrix;

    }

public:

    aruco::CameraParameters getCameraParameters(){
        return camera_parameters_;
    }




};


#endif //ISAM2_ARUCOMARKERS_H
