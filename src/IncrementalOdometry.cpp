//
// Created by socrob on 09-04-2019.
//

#include "IncrementalOdometry.h"


Pose3 IncrementalOdometry::getIncrementalOdometry() {

    Eigen::Matrix4d X_0, X_1;
    X_0 = previous_pose_.matrix();
    X_1 = current_pose_.matrix();

    Eigen::Matrix4d T = X_0.inverse()*X_1;
    return Pose3(T);

}