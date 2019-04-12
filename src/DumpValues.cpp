//
// Created by socrob on 09-04-2019.
//

#include "DumpValues.h"


using symbol_shorthand::X; // Pose3 (x,y,z,r,p,y)
using symbol_shorthand::V; // Vel   (xdot,ydot,zdot)
using symbol_shorthand::B; // Bias  (ax,ay,az,gx,gy,gz)
using symbol_shorthand::L; // Landmark  (ax,ay,az,gx,gy,gz)


void DumpValues::dump2File(Values batch_estimate, int max_poses, int max_landmarks) {
    //dump poses
    for(int i = 0; i < max_poses; i++){
        this->outfile_poses_<<batch_estimate.at<Pose3>(X(i)).x()
        <<"\n"<<batch_estimate.at<Pose3>(X(i)).y()
        <<"\n"<<batch_estimate.at<Pose3>(X(i)).z()<<"\n"<<"\n";

    }

    //dump landmarks
    for(int i = 0; i < max_landmarks; i++){
        this->outfile_landmarks_<<batch_estimate.at<Point3>(L(i)).x()
                            <<"\n"<<batch_estimate.at<Point3>(L(i)).y()
                            <<"\n"<<batch_estimate.at<Point3>(L(i)).z()<<"\n"<<"\n";

    }

}