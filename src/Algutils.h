//
// Created by alg on 18-09-2018.
//
#include <Eigen/Dense>
#include <Eigen/Geometry>

#ifndef ATTITUDE_ESTIMATION_ALGUTILS_H
#define ATTITUDE_ESTIMATION_ALGUTILS_H
#define MARKER_SIZE     0.185
#define XML_FILE_PATH   "/home/socrob/alg_calibration.yml"

//#define OUTFILEX "/home/socrob/iSAM_MOCAP_Results/iSAMX.txt"
#define OUTFILE_POSE "/home/socrob/iSAM_MOCAP_Results/iSAM_poses_9.txt"
#define OUTFILEY "/home/socrob/iSAM_MOCAP_Results/iSAMY.txt"
#define OUTFILEZ "/home/socrob/iSAM_MOCAP_Results/iSAMZ.txt"
#define OUTFILE_LAND "/home/socrob/iSAM_MOCAP_Results/iSAM_corners_9.txt"
#define OUTSVO "svo_3_scale_080.txt"

/*namespace alg_utils{

    Eigen::Matrix4d M_H_TLTR << 1, 0, 0, 0.185,
                                0, 1, 0, 0,
                                0, 0, 1, 0,
                                0, 0, 0, 1;

    Eigen::Matrix4d M_H_TRBR << 1, 0, 0, 0,
                                0, 1, 0, -0.185,
                                0, 0, 1, 0,
                                0, 0, 0, 1;

    Eigen::Matrix4d M_H_BRBL << 1, 0, 0, -0.185,
                                0, 1, 0, 0,
                                0, 0, 1, 0,
                                0, 0, 0, 1;

    Eigen::Matrix4d M_H_BLTL << 1, 0, 0, 0,
                                0, 1, 0, 0.185,
                                0, 0, 1, 0,
                                0, 0, 0, 1;



}*/


#endif //ATTITUDE_ESTIMATION_ALGUTILS_H
