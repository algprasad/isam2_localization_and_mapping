//
// Created by socrob on 10-04-2019.
//
#include "Utils.h"
using namespace gtsam;

void setISAM2Parameters(ISAM2Params& parameters){

    //TODO(Need to tune the parameters.)----> Understand the meaning of each parameter

    //parameters.optimizationParams = ISAM2DoglegParams();
    //parameters.relinearizeThreshold = 0.1;
    //parameters.relinearizeSkip = 5;
    //parameters.
    parameters.enableRelinearization = true;

    ///Batch:  assigning params
    LevenbergMarquardtParams LMParams;
    //LMParams.setMaxIterations(100);
    //LMParams.setAbsoluteErrorTol(10000);
    LMParams.setVerbosity("ERROR");
    LMParams.setlambdaUpperBound(1e30);
    LMParams.setRelativeErrorTol(1e-25);
}


void setArucoMarkerCornerMatrices(Eigen::Matrix4d& M_H_TLTR, Eigen::Matrix4d& M_H_TRBR, Eigen::Matrix4d& M_H_BRBL, Eigen::Matrix4d& M_H_BLTL){

    M_H_TLTR<< 1, 0, 0, 0.185,
            0, 1, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1;

    M_H_TRBR << 1, 0, 0, 0,
            0, 1, 0, -0.185,
            0, 0, 1, 0,
            0, 0, 0, 1;

    M_H_BRBL << 1, 0, 0, -0.185,
            0, 1, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1;

    M_H_BLTL << 1, 0, 0, 0,
            0, 1, 0, 0.185,
            0, 0, 1, 0,
            0, 0, 0, 1;

}

void setWorldCameraLandmarkMatrices( Eigen::Matrix4d& oHc, Eigen::Matrix4d& lH_tl, Eigen::Matrix4d& lH_tr, Eigen::Matrix4d& lH_br, Eigen::Matrix4d& lH_bl){
    //The revelation of markers being upside down gave rise to this correction of camera frame
    oHc <<  0, 0, 1, 0,
            1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 0, 1;

    lH_tl <<1, 0, 0, -MARKER_SIZE / 2,
            0, 1, 0, MARKER_SIZE / 2,
            0, 0, 1, 0,
            0, 0, 0, 1;

    lH_tr <<1, 0, 0, MARKER_SIZE / 2,
            0, 1, 0, MARKER_SIZE / 2,
            0, 0, 1, 0,
            0, 0, 0, 1;

    lH_br <<1, 0, 0, MARKER_SIZE / 2,
            0, 1, 0, -MARKER_SIZE / 2,
            0, 0, 1, 0,
            0, 0, 0, 1;

    lH_bl <<1, 0, 0, -MARKER_SIZE / 2,
            0, 1, 0, -MARKER_SIZE / 2,
            0, 0, 1, 0,
            0, 0, 0, 1;


}


bool isMarkerDetectedFirstTime(int id, vector<int>& aruco_marker_indexes ){
    for(int i = 0; i < aruco_marker_indexes.size(); i++){
        if(aruco_marker_indexes[i] == id)
            return false;
    }

    aruco_marker_indexes.push_back(id);
    return true;
}

