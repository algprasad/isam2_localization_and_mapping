//
// Created by socrob on 29-05-2019.
//

#ifndef ISAM2_CONFIGPARAMS_H
#define ISAM2_CONFIGPARAMS_H

#include <iostream>

class ConfigParams {
public:
    bool batch_ ;
    bool incremental_update_;
    bool debug_;
    bool rviz_visualizations_;
    bool image_space_visualizations_;

    int max_poses_batch_;

    ConfigParams();




};

#endif //ISAM2_CONFIGPARAMS_H
