//
// Created by socrob on 09-04-2019.
//

#ifndef ISAM2_ARUCOLANDMARKS_H
#define ISAM2_ARUCOLANDMARKS_H

#include "InitialParameters.h"
#include "ArUcoMarkers.h"
#include "Utils.h"

class ArUcoLandmark {

public:

    aruco::Marker aruco_marker_; //the main marker
    unsigned int landmark_index_[4]; //storing the landmark indexes of the four corners
    Eigen::VectorXd corner_points_;

    //coordinates of the corner points in world frame
    gtsam::Pose3 wHtl_; //top-left in world frame
    gtsam::Pose3 wHtr_; //top-right
    gtsam::Pose3 wHbr_; //bottom-right
    gtsam::Pose3 wHbl_; //bottom-left

    //postion and orientation transfor from the camera to the marker
    gtsam::Pose3 cHm_;

    //ArUco Marker pose
    gtsam::Pose3 wHm_;
    Eigen::Matrix4d M_H_TLTR_, M_H_TRBR_, M_H_BRBL_, M_H_BLTL_;


public:

    //Default constructor
    ArUcoLandmark(aruco::Marker aruco_marker){
        aruco_marker_ = aruco_marker;
        corner_points_ =  (Eigen::VectorXd(8)
                <<  aruco_marker_[0].x, aruco_marker_[0].y,
                    aruco_marker_[1].x, aruco_marker_[1].y,
                    aruco_marker_[2].x, aruco_marker_[2].y,
                    aruco_marker_[3].x, aruco_marker_[3].y).finished();

        //set the transform from camera to marker
        cHm_ = setCameraTransform();
        setArucoMarkerCornerMatrices(M_H_TLTR_, M_H_TRBR_, M_H_BRBL_, M_H_BLTL_);

    }

    unsigned int getLandmarkIndex(vector<ArUcoLandmark>& vector_aruco_landmarks); //method to check if the landmark has already been observed. if yes getting the index, if not assigning new value.
    void setLandmarkIndex(int index);
    void setCornerPointsInWorldFrame(gtsam::Pose3 wHo);
    void setArUcoMarkerPose(gtsam::Pose3 current_robot_pose);

    gtsam::Pose3 setCameraTransform();

    void printCornerPoints();
    void printCameraTransforms();

    //TODO(ALG): Add the function to visualize the marker boundaries in the image space






};


#endif //ISAM2_ARUCOLANDMARKS_H
