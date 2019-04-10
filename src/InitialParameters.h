//
// Created by socrob on 09-04-2019.
//

#ifndef ISAM2_INITIALPARAMETERS_H
#define ISAM2_INITIALPARAMETERS_H

#include <iostream>
#include "ros/ros.h"

#include "sensor_msgs/Imu.h"
#include <Eigen/Geometry>
#include <Eigen/src/Geometry/Quaternion.h>


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
#include <opencv2/opencv.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/PoseStamped.h>

#include <cv_bridge/cv_bridge.h>
#include "Arucoalg.h"
#include "Algutils.h"

#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/ProjectionFactor.h>
#include <gtsam/nonlinear/ISAM2.h>



using namespace std;
using namespace gtsam;

//TODO(ALG): make it get values from a yaml file

namespace initial_values_ns {

    //Define the camera calibration parameters
    gtsam::Cal3_S2::shared_ptr K(new gtsam::Cal3_S2(532.13605, 529.40496, 0.0, 472.2739, 376.43908));

    //Declaring the initial state here
    Eigen::Matrix<double, 10, 1> initial_state = Eigen::Matrix<double, 10, 1>::Zero();

    //Adding a prior on the initial state
    Rot3 prior_rotation = Rot3::Quaternion(1, 0, 0,0);///This is used to make the quaternion w, x, y, z (Since w is entered last in the entry and we want it first)
    Point3 prior_point(initial_state.head<3>());///This means get the first three values
    Pose3 prior_pose(prior_rotation, prior_point);
    Vector3 prior_velocity(initial_state.tail<3>()); ///This means get the last three values

    //IMU bias
    imuBias::ConstantBias prior_imu_bias; // assume zero initial bias
}

namespace noise_values_ns{


    noiseModel::Isotropic::shared_ptr pixel_noise = noiseModel::Isotropic::Sigma(2, 100.0);


    noiseModel::Diagonal::shared_ptr pose_noise_model = noiseModel::Diagonal::Sigmas((Vector(6) << 0.01, 0.01, 0.01, 0.05, 0.05, 0.05).finished()); // rad,rad,rad,m, m, m ///Changed the values.. increased by an order.
    noiseModel::Diagonal::shared_ptr velocity_noise_model = noiseModel::Isotropic::Sigma(3,0.1); // m/s
    noiseModel::Diagonal::shared_ptr bias_noise_model = noiseModel::Isotropic::Sigma(6,1e-3);

    //Odometry Noise
    noiseModel::Diagonal::shared_ptr odometry_noise = noiseModel::Diagonal::Sigmas((Vector(6)<< 0.15, 0.15, 0.15, 0.15, 0.15, 0.15).finished());
    noiseModel::Diagonal::shared_ptr corner_noise = noiseModel::Diagonal::Sigmas((Vector(3)<< 0.001, 0.001, 0.001).finished());


}



#endif //ISAM2_INITIALPARAMETERS_H
