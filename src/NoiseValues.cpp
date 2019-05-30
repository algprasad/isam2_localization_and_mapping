//
// Created by socrob on 29-05-2019.
//

#include "NoiseValues.h"

NoiseValues::NoiseValues() {

    //TODO(YAML): get the noise values from yaml file


    pixel_noise_ = noiseModel::Isotropic::Sigma(2, 50.0);
    pose_noise_model_ = noiseModel::Diagonal::Sigmas((Vector(6) << 0.01, 0.01, 0.01, 0.05, 0.05, 0.05).finished()); // rad,rad,rad,m, m, m ///Changed the values.. increased by an order.
    velocity_noise_model_ = noiseModel::Isotropic::Sigma(3,0.1); // m/s
    bias_noise_model_ = noiseModel::Isotropic::Sigma(6,1e-3);

    //Odometry Noise
    odometry_noise_ = noiseModel::Diagonal::Sigmas((Vector(6)<< 0.15, 0.15, 0.15, 0.15, 0.15, 0.15).finished());
    corner_noise_ = noiseModel::Diagonal::Sigmas((Vector(3)<< 0.001, 0.001, 0.001).finished());

    //Constant Velocity Noise
    constant_velocity_noise_ = noiseModel::Diagonal::Sigmas((Vector(3)<< 0.1, 0.1, 0.1).finished());


}