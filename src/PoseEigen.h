//
// Created by socrob on 09-04-2019.
//

#ifndef ISAM2_POSEEIGEN_H
#define ISAM2_POSEEIGEN_H

#include <eigen3/Eigen/Dense>

class PoseEigen {
public:
    Eigen::Matrix3d orientation_mat_;
    Eigen::Quaterniond orientation_;

    Eigen::Vector3d position_ ;
public:

    //Constructor
    PoseEigen(Eigen::Vector3d position, Eigen::Quaterniond orientation){
        position_ = position;
        orientation_ = orientation;
        //TODO(ALG): Convert quaternion to 3d rotation matrix
        //orientation_mat_ =

    }

    PoseEigen(Eigen::Vector3d position, Eigen::Matrix3d orientation){
        position_ = position;
        orientation_mat_ = orientation;

        //TODO(ALG): Convert 3d matrix to quaternion
        //orientation_ =

    }





};


#endif //ISAM2_POSEEIGEN_H
