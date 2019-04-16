//
// Created by socrob on 09-04-2019.
//

#include "ArUcoLandmark.h"
#include "InitialParameters.h"
#include "Utils.h"
#include <opencv2/core.hpp>
#include <opencv-3.3.1-dev/opencv/cxeigen.hpp>


unsigned int ArUcoLandmark::getLandmarkIndex(vector<ArUcoLandmark>& vector_aruco_landmarks) {
    //find if the marker id is equal to any of the ids
    int i = 0;

    if(vector_aruco_landmarks.empty()){
        setLandmarkIndex(0);
        vector_aruco_landmarks.push_back(*this);
        return 0;
    }
    for(; i< vector_aruco_landmarks.size(); i++){

        if(vector_aruco_landmarks[i].aruco_marker_.id == this->aruco_marker_.id)
            return this->landmark_index_[0];

    }


    //add to the vector_aruco_landmarks the current ArUcoLandmark object
    vector_aruco_landmarks.push_back(*this);
    setLandmarkIndex(vector_aruco_landmarks[i].landmark_index_[0] + 4);

    return this->landmark_index_[0];  //return the newly set aruco markers landmark index.

}


void ArUcoLandmark::setLandmarkIndex(int index) {
    this->landmark_index_[0] = index;
    this->landmark_index_[1] = index + 1;
    this->landmark_index_[2] = index + 2;
    this->landmark_index_[3] = index + 3;
}

void ArUcoLandmark::setCornerPointsInWorldFrame(gtsam::Pose3 current_pose_robot) {

    Eigen::Matrix4d wHo, cHl; //object in the world frame

    Eigen::Matrix4d oHc, lH_tl, lH_tr, lH_br, lH_bl;
    setWorldCameraLandmarkMatrices(oHc, lH_tl, lH_tr, lH_br, lH_bl);

    wHo = current_pose_robot.matrix();
    cHl = this->cHm_.matrix();

    this->wHtl_ = Pose3(wHo * oHc * cHl * lH_tl);
    this->wHtr_ = Pose3(wHo * oHc * cHl * lH_tr);
    this->wHbr_ = Pose3(wHo * oHc * cHl * lH_br);
    this->wHbl_ = Pose3(wHo * oHc * cHl * lH_bl);

}


gtsam::Pose3 ArUcoLandmark::setCameraTransform() {

    double position_transform[3], orientation_transform[3];
    this->aruco_marker_.OgreGetPoseParameters(position_transform, orientation_transform);

    cv::Mat rotation_matrix_cv;
    cv::Rodrigues(this->aruco_marker_.Rvec, rotation_matrix_cv);

    Eigen::Matrix3d rotation_matrix_eigen;
    cv::cv2eigen(rotation_matrix_cv, rotation_matrix_eigen);

    return gtsam::Pose3(gtsam::Rot3(rotation_matrix_eigen),
            gtsam::Point3(position_transform[0], position_transform[1], position_transform[2]));

}

void ArUcoLandmark::printCornerPoints() {
    std::cout<<" Pixel coordinates of the corners: "<<std::endl;
    std::cout<<this->corner_points_;

}

void ArUcoLandmark::printCameraTransforms() {
    std::cout<<" Camera Transform: "<<endl;
    std::cout<<this->cHm_.matrix();
    std::cout<<std::endl;
}

void ArUcoLandmark::setArUcoMarkerPose(gtsam::Pose3 current_robot_pose) {

    Eigen::Matrix4d wHo = current_robot_pose.matrix();
    Eigen::Matrix4d cHm = this->cHm_.matrix();
    Eigen::Matrix4d oHc, lH_tl, lH_tr, lH_br, lH_bl;
    setWorldCameraLandmarkMatrices(oHc, lH_tl, lH_tr, lH_br, lH_bl);

    this->wHm_ =  Pose3(wHo*oHc*cHm);

}

