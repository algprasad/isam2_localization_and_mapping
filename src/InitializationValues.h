//
// Created by socrob on 29-05-2019.
//

#ifndef ISAM2_INITIALVALUES_H
#define ISAM2_INITIALVALUES_H

#include "InitialParameters.h"

class InitializationValues {
public:

    //Define the camera calibration parameters
    gtsam::Cal3_S2::shared_ptr K_;// (new gtsam::Cal3_S2(532.13605, 529.40496, 0.0, 472.2739, 376.43908));

    gtsam::Rot3 prior_rotation_;
    gtsam::Point3 prior_point_;
    gtsam::Pose3 prior_pose_;
    gtsam::Vector3 prior_velocity_;

    //IMU bias
    imuBias::ConstantBias prior_imu_bias_; // assume zero initial bias
    imuBias::ConstantBias prev_bias_;
    //NavState for IMU predictions and preintegrated IMU

    // Store previous state for the imu integration and the latest predicted outcome.
    NavState nav_state_prev_state_;
    NavState nav_state_current_state_;


    InitializationValues();





};


#endif //ISAM2_INITIALVALUES_H
