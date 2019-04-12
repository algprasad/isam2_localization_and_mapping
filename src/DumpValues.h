//
// Created by socrob on 09-04-2019.
//

#ifndef ISAM2_DUMPVALUES_H
#define ISAM2_DUMPVALUES_H

#include <fstream>
#include <iostream>
#include "InitialParameters.h"
#include <opencv2/core/core.hpp>
#include <string>
#include <eigen3/Eigen/Dense>


class DumpValues {
public:
    std::ofstream outfile_poses_ ;
    std::ofstream outfile_landmarks_ ;
    std::string filename_poses_ ;
    std::string filename_landmarks_;

public:
    //Default constructor
    DumpValues(){

        string config_filename = "/home/socrob/isam2_ws/src/isam2/config/config.yaml";
        cv::FileStorage fs;
        fs.open(config_filename, cv::FileStorage::READ);

        filename_poses_ = (string) fs["DumpFilePoses"];
        filename_landmarks_ = (string) fs["DumpFileLandmarks"];

        //open the poses file
        outfile_poses_.open(filename_poses_, std::ios_base::app);
        outfile_landmarks_.open(filename_landmarks_, std::ios_base::app);

    }

    void dump2File(Values batch_estimate, int max_poses, int max_landmarks);




};


#endif //ISAM2_DUMPVALUES_H
