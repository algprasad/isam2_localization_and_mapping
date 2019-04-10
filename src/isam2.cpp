#include <iostream>
#include "ros/ros.h"
#include "PoseEigen.h"
#include "RosHandler.h"
#include "InitialParameters.h"
#include "Utils.h"
#include "IncrementalOdometry.h"
#include "ArUcoMarkers.h"
#include "ArUcoLandmark.h"



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
    initial_values.insert(X(0), initial_values_ns::prior_pose);

    NonlinearFactorGraph graph;

    graph.add(PriorFactor<Pose3>(X(0), initial_values_ns::prior_pose, noise_values_ns::pose_noise_model));
    
    //Set the values of all constant matrices needed in the program
    Eigen::Matrix4d M_H_TLTR, M_H_TRBR, M_H_BRBL, M_H_BLTL;
    setArucoMarkerCornerMatrices(M_H_TLTR, M_H_TRBR, M_H_BRBL, M_H_BLTL);

    Eigen::Matrix4d oHc, lH_tl, lH_tr, lH_br, lH_bl;
    setWorldCameraLandmarkMatrices(oHc, lH_tl, lH_tr, lH_br, lH_bl);


    //Standard Pose3 current and previous values to be used throughout
    Pose3 previous_svo_pose  = initial_values_ns::prior_pose;
    Pose3 current_svo_pose = previous_svo_pose;

    IncrementalOdometry incremental_odometry(current_svo_pose, previous_svo_pose);




    unsigned int pose_number = 0; //starting the count for pose_numbers
    ros::Rate rate(20);

    //list to keep track of all ArUco landmarks
    vector<ArUcoLandmark> vector_aruco_landmarks;


    while(ros::ok()) {
        //get current image
        cv::Mat current_image = cv_bridge::toCvCopy(ros_handler.getRosImage(), "mono8")->image;


        if(ros_handler.new_imu_){
            //deliberately left empty as imu values will be included in the visual inertial odometry

        }

        if(ros_handler.new_image_){
            pose_number++;
            //assigning values from the rostopic too in the current pose
            current_svo_pose = Pose3( Rot3(ros_handler.getRosOdometry().pose.orientation.w,
                                           ros_handler.getRosOdometry().pose.orientation.x,
                                           ros_handler.getRosOdometry().pose.orientation.y,
                                           ros_handler.getRosOdometry().pose.orientation.z),

                                        Point3(ros_handler.getRosOdometry().pose.position.x,
                                            ros_handler.getRosOdometry().pose.position.y,
                                            ros_handler.getRosOdometry().pose.position.z));

            incremental_odometry.setCurrentPose(current_svo_pose);
            incremental_odometry.setPreviousPose(previous_svo_pose);

            Pose3 inc_odom = incremental_odometry.getIncrementalOdometry(); //returns a Pose3

            // Create odometry (Between) factors between consecutive poses
            graph.emplace_shared<BetweenFactor<Pose3> >(X(pose_number -1), X(pose_number), inc_odom, noise_values_ns::odometry_noise);

            //Add initial values to the new pose
            initial_values.insert(X(pose_number), Pose3(previous_svo_pose.matrix()*inc_odom.matrix()));

            previous_svo_pose = current_svo_pose;

        }

        //ArUco object
        ArUcoMarkers aruco_marker;  //keeps track of all markers in the frame
        if(aruco_marker.hasMarkers(current_image) && ros_handler.new_image_){

            //For every aruco marker detected add the corners to the factor graph and initialize the values
            for(int i = 0; i < aruco_marker.markers_.size(); i++){

                //Transfer the body of this loop to a method of ArUcoLandmark
                //ArUcoLandmark represents one landmark marker whereas ArUcoMarkers represents all the markers detected in one scene.
                ArUcoLandmark aruco_landmark(aruco_marker.markers_[i]);

                ///Assigning measurement pixels here for all 4 corners
                Point2 measurement_tl(aruco_landmark.corner_points_[0], aruco_landmark.corner_points_[1]);
                Point2 measurement_tr(aruco_landmark.corner_points_[2], aruco_landmark.corner_points_[3]);
                Point2 measurement_br(aruco_landmark.corner_points_[4], aruco_landmark.corner_points_[5]);
                Point2 measurement_bl(aruco_landmark.corner_points_[6], aruco_landmark.corner_points_[7]);


                //assign the landmark index based on if the landmark has been detected before or not
                int landmark_index = aruco_landmark.getLandmarkIndex(vector_aruco_landmarks); //returns the landmark index of the TL corner


                //add the landmark factor to the factor graph
                graph.emplace_shared<GenericProjectionFactor<Pose3, Point3, Cal3_S2> >(measurement_tl, noise_values_ns::pixel_noise,
                                                                                       Symbol('x', pose_number),
                                                                                       Symbol('l', landmark_index), initial_values_ns::K);
                graph.emplace_shared<GenericProjectionFactor<Pose3, Point3, Cal3_S2> >(measurement_tr, noise_values_ns::pixel_noise,
                                                                                       Symbol('x', pose_number),
                                                                                       Symbol('l', landmark_index + 1), initial_values_ns::K);
                graph.emplace_shared<GenericProjectionFactor<Pose3, Point3, Cal3_S2> >(measurement_br, noise_values_ns::pixel_noise,
                                                                                       Symbol('x', pose_number),
                                                                                       Symbol('l', landmark_index + 2), initial_values_ns::K);
                graph.emplace_shared<GenericProjectionFactor<Pose3, Point3, Cal3_S2> >(measurement_bl, noise_values_ns::pixel_noise,
                                                                                       Symbol('x', pose_number),
                                                                                       Symbol('l', landmark_index + 3), initial_values_ns::K);



                //TODO(ALG): add the pose3 aruco factor connecting all the corner values to a centric pose3 figure with BetweenConstraints
                //TODO(ALG): add factors between the corners


                //initialize the landmark corners if detected for the first time
                //Note for 10 April: make something in the ArUcoLandmark object that tells you if the object has been detected for the first time. Then use the date to initialise the values of the aruco landmark corners


            }
        }
        
        
        

        //TODO(Solve the graph)


        //TODO(Get updated values)

        ros_handler.resetBools();
        ros::spinOnce();
        rate.sleep();

    }

    return 0;

}
