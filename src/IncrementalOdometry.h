//
// Created by socrob on 09-04-2019.
//

#ifndef ISAM2_INCREMENTALODOMETRY_H
#define ISAM2_INCREMENTALODOMETRY_H

#include "InitialParameters.h"

using namespace gtsam;


class IncrementalOdometry {
public:

    Pose3 previous_pose_;
    Pose3 current_pose_;
    Eigen::Matrix4d previous2current_pose_; //Transform from previuos to current pose


public:
    //Default constructor
    IncrementalOdometry(Pose3 current_pose, Pose3 previous_pose){
        previous_pose_ = previous_pose;
        current_pose_  = current_pose;
    }


    Pose3 getIncrementalOdometry();

    void setCurrentPose(Pose3 current_pose){
        current_pose_ = current_pose;
    }

    void setPreviousPose(Pose3 previous_pose){
        previous_pose_  = previous_pose;
    }







};


#endif //ISAM2_INCREMENTALODOMETRY_H
