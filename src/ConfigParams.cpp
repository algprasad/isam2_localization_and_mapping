//
// Created by socrob on 29-05-2019.
//

#include "ConfigParams.h"
#include <iostream>
#include "Utils.h"



ConfigParams::ConfigParams() {

    std::string config_filename = "/home/socrob/isam2_ws/src/isam2/config/config.yaml";
    cv::FileStorage fs;
    fs.open(config_filename, cv::FileStorage::READ);

    batch_ = (int) fs["BATCH"];
    debug_ = (int) fs["DEBUG"];
    incremental_update_ = (int)(fs["INCREMENTAL_UPDATE"]);
    max_poses_batch_ = (int)(fs["MAX_POSES_BATCH"]);

    rviz_visualizations_  =  (int) fs["RVIZ_VISUALIZATION"]; ;
    image_space_visualizations_ = (int) fs["IMAGE_SPACE_VISUALIZATION"];




}