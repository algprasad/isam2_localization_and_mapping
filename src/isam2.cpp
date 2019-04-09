#include <iostream>
#include "ros/ros.h"
#include "PoseEigen.h"
#include "RosHandler.h"
#include "InitialParameters.h"
#include "Utils.h"


using namespace std;
using namespace gtsam;
//using namespace initial_values;

using symbol_shorthand::X; // Pose3 (x,y,z,r,p,y)
using symbol_shorthand::V; // Vel   (xdot,ydot,zdot)
using symbol_shorthand::B; // Bias  (ax,ay,az,gx,gy,gz)
using symbol_shorthand::L; // Landmark  (ax,ay,az,gx,gy,gz)

int main(int argc, char** argv) {

    ros::init(argc, argv, "isam2");
    
    //manages subscription and callbacks
    RosHandler ros_handler(argc, argv);

    ISAM2Params parameters;
    setISAM2Parameters(parameters);

    //TODO(ALG): Rearrange these to look better and organized.

    //defining ISAM2 object with the set ISAM2Parameters
    gtsam::ISAM2 isam2(parameters);

    Values initial_values;
    initial_values.insert(X(0), initial_values::prior_pose);

    NonlinearFactorGraph graph;

    graph.add(PriorFactor<Pose3>(X(0), initial_values::prior_pose, noise_values::pose_noise_model));
    
    //Set the values of all constant matrices needed in the program
    Eigen::Matrix4d M_H_TLTR, M_H_TRBR, M_H_BRBL, M_H_BLTL;
    setArucoMarkerCornerMatrices(M_H_TLTR, M_H_TRBR, M_H_BRBL, M_H_BLTL);

    Eigen::Matrix4d oHc, lH_tl, lH_tr, lH_br, lH_bl;
    setWorldCameraLandmarkMatrices(oHc, lH_tl, lH_tr, lH_br, lH_bl);



    unsigned int pose_number = 0; //starting the count for pose_numbers
    ros::Rate rate(20);



    while(ros::ok()) {
        if(ros_handler.new_imu_){
            //deliberately left empty as imu values will be included in the visual inertial odometry
        }

        if(ros_handler.new_image_){
            pose_number++;
            getIncrementalOdometry();

            //Add to the factor.



        }
        
        
        
        //Use something else
        Eigen::Matrix4d wHo, cHl;

        //TODO(Make the graph)
        //if new imu is available add it to the graph

        //if new image is available. Get incremental odometry and aruco corners. Every calculation is delegated to the function.


        //TODO(Solve the graph)


        //TODO(Get updated values)









        ros_handler.resetBools();
        ros::spinOnce();
        rate.sleep();

    }

    return 0;

}
