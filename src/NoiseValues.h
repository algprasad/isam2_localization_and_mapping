//
// Created by socrob on 29-05-2019.
//

#ifndef ISAM2_NOISEVALUES_H
#define ISAM2_NOISEVALUES_H

#include "InitialParameters.h"

class NoiseValues {

public:
    noiseModel::Isotropic::shared_ptr pixel_noise_; // = noiseModel::Isotropic::Sigma(2, 50.0);
    noiseModel::Diagonal::shared_ptr pose_noise_model_; //= noiseModel::Diagonal::Sigmas((Vector(6) << 0.01, 0.01, 0.01, 0.05, 0.05, 0.05).finished()); // rad,rad,rad,m, m, m ///Changed the values.. increased by an order.
    noiseModel::Diagonal::shared_ptr velocity_noise_model_;// = noiseModel::Isotropic::Sigma(3,0.1); // m/s
    noiseModel::Diagonal::shared_ptr bias_noise_model_;// = noiseModel::Isotropic::Sigma(6,1e-3);

    //Odometry Noise
    noiseModel::Diagonal::shared_ptr odometry_noise_;// = noiseModel::Diagonal::Sigmas((Vector(6)<< 0.15, 0.15, 0.15, 0.15, 0.15, 0.15).finished());
    noiseModel::Diagonal::shared_ptr corner_noise_;// = noiseModel::Diagonal::Sigmas((Vector(3)<< 0.001, 0.001, 0.001).finished());

    //Constant Velocity Noise
    noiseModel::Diagonal::shared_ptr constant_velocity_noise_;// = noiseModel::Diagonal::Sigmas((Vector(3)<< 0.1, 0.1, 0.1).finished());



    NoiseValues(); //Set the values of all the noise(member) values from the config.yaml

};


#endif //ISAM2_NOISEVALUES_H
