//
// Created by alg on 18-09-2018.
//

#ifndef ATTITUDE_ESTIMATION_ARUCOALG_H
#define ATTITUDE_ESTIMATION_ARUCOALG_H

#include <aruco.h>
#include "Algutils.h"

class Arucoalg {
public:

    aruco::CameraParameters CamParam;
    cv::Mat discoeff;
    cv::Mat camMat;
    float Marker = MARKER_SIZE;

    aruco::MarkerDetector MDetector;
    Arucoalg(){
        CamParam.readFromXMLFile(XML_FILE_PATH);
        discoeff = CamParam.Distorsion;
        camMat = CamParam.CameraMatrix;

    }
    aruco::CameraParameters getCamParam(){
        return CamParam;
    }

};


#endif //ATTITUDE_ESTIMATION_ARUCOALG_H
