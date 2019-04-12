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




#endif //ISAM2_INITIALPARAMETERS_H
