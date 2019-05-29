//
// Created by socrob on 29-05-2019.
//

#include "InitializationValues.h"

InitializationValues::InitializationValues() {

    //Define the camera calibration parameters
    gtsam::Cal3_S2::shared_ptr K(new gtsam::Cal3_S2(532.13605, 529.40496, 0.0, 472.2739, 376.43908));

    //Adding a prior on the initial state
    Rot3 prior_rotation = Rot3::Quaternion(1, 0, 0,0);///This is used to make the quaternion w, x, y, z (Since w is entered last in the entry and we want it first)
    Point3 prior_point(0,0,0);///This means get the first three values
    Pose3 prior_pose(prior_rotation, prior_point);
    Vector3 prior_velocity(0.1, 0.1, 0.1); ///This means get the last three values

    //IMU bias
    imuBias::ConstantBias prior_imu_bias; // assume zero initial bias
    imuBias::ConstantBias prev_bias = prior_imu_bias;

    //NavState for IMU predictions and preintegrated IMU
    // Store previous state for the imu integration and the latest predicted outcome.
    NavState nav_state_prev_state(prior_pose, prior_velocity);
    NavState nav_state_current_state = nav_state_prev_state;


    K_ = K;
    prior_rotation_ = prior_rotation;
    prior_point_ = prior_point;
    prior_pose_ = prior_pose;
    prior_velocity_ = prior_velocity;

    prior_imu_bias_ = prior_imu_bias;
    prev_bias_ = prev_bias;

    nav_state_current_state_ = nav_state_current_state;
    nav_state_prev_state_ = nav_state_prev_state;




}