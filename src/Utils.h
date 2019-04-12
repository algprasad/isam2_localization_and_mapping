//
// Created by socrob on 09-04-2019.
//

#ifndef ISAM2_UTILS_H
#define ISAM2_UTILS_H


//GTSAM include headers
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/slam/dataset.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/inference/Symbol.h>

// Camera observations of landmarks (i.e. pixel coordinates) will be stored as Point2 (x, y).
#include <gtsam/geometry/Point2.h>
#include "InitialParameters.h"

using namespace gtsam;

void setISAM2Parameters(ISAM2Params& parameters);

void setArucoMarkerCornerMatrices(Eigen::Matrix4d& M_H_TLTR, Eigen::Matrix4d& M_H_TRBR, Eigen::Matrix4d& M_H_BRBL, Eigen::Matrix4d& M_H_BLTL);

void setWorldCameraLandmarkMatrices( Eigen::Matrix4d& oHc, Eigen::Matrix4d& lH_tl, Eigen::Matrix4d& lH_tr, Eigen::Matrix4d& lH_br, Eigen::Matrix4d& lH_bl);

bool isMarkerDetectedFirstTime(int id, std::vector<int>& aruco_marker_indexes);



#endif //ISAM2_UTILS_H
