#include <iostream>
#include "ros/ros.h"
#include "PoseEigen.h"
#include "RosHandler.h"
#include "InitialParameters.h"
#include "Utils.h"
#include "IncrementalOdometry.h"
#include "ArUcoMarkers.h"
#include "ArUcoLandmark.h"
#include "DumpValues.h"
#include "RVizVisualizations.h"


#include <opencv2/core/core.hpp>
#include "ConfigParams.h"
#include "InitializationValues.h"
#include "NoiseValues.h"





using namespace std;
using namespace gtsam;

using symbol_shorthand::X; // Pose3 (x,y,z,r,p,y)
using symbol_shorthand::V; // Vel   (xdot,ydot,zdot)
using symbol_shorthand::B; // Bias  (ax,ay,az,gx,gy,gz)
using symbol_shorthand::L; // Landmark  (ax,ay,az,gx,gy,gz)
using symbol_shorthand::M; //ArUco Marker


int main(int argc, char** argv) {

    ros::init(argc, argv, "isam2");
    
    //manages subscription and callbacks
    RosHandler ros_handler(argc, argv);

    //set the confiuration from the config file
    ConfigParams config_params;

    InitializationValues initialization_values;
    NoiseValues noise_values;


    ISAM2Params parameters;
    setISAM2Parameters(parameters);
    ISAM2 isam2(parameters);

    LevenbergMarquardtParams LMParams;
    setLMParameters(LMParams);


    Values initial_values;
    initial_values.insert(X(0), initialization_values.prior_pose_);
    initial_values.insert(V(0), initialization_values.prior_velocity_);
    initial_values.insert(B(0), initialization_values.prior_imu_bias_);


    boost::shared_ptr<PreintegratedCombinedMeasurements::Params> p = PreintegratedCombinedMeasurements::Params::MakeSharedD(0.0);
    setPreintegratedCombinedMeasurements(p);
    PreintegrationType *imu_preintegrated;
    imu_preintegrated = new PreintegratedImuMeasurements(p);



    NonlinearFactorGraph graph;
    graph.add(PriorFactor<Pose3>(X(0), initialization_values.prior_pose_, noise_values.pose_noise_model_));
    graph.add(PriorFactor<Vector3>(V(0), initialization_values.prior_velocity_, noise_values.velocity_noise_model_));
    graph.add(PriorFactor<imuBias::ConstantBias>(B(0), initialization_values.prior_imu_bias_, noise_values.bias_noise_model_));

    Values current_estimate = initial_values;

    //Visual-Odometry SVO Pose3 current and previous values
    Pose3 previous_svo_pose  = initialization_values.prior_pose_;
    Pose3 current_svo_pose = previous_svo_pose;

    IncrementalOdometry incremental_odometry(current_svo_pose, previous_svo_pose);

    //list to keep track of all ArUco landmarks
    vector<ArUcoLandmark> vector_aruco_landmarks;
    vector<int> vector_aruco_marker_ids;

    //Standard Current Pose and Previous Pose to be used throughout
    Pose3 previous_pose = initialization_values.prior_pose_;
    Pose3 current_pose = previous_pose;

    //RViz Visualization object for pose transform and corners
    RVizVisualizations rviz_current_pose_corner;


    unsigned int pose_number = 0; //starting the count for pose_numbers
    ros::Rate rate(20);
    while(ros::ok()) {
        //get current image
        cv::Mat current_image = ros_handler.current_image_;

        if(ros_handler.new_imu_){
            //dt is how often the imu is available. since it is subscribing at 20Hz dt has been put to 0.05
            double dt = 0.05;

            imu_preintegrated-> integrateMeasurement(Vector3(ros_handler.ros_imu_.linear_acceleration.x,
                                                             ros_handler.ros_imu_.linear_acceleration.y,
                                                             ros_handler.ros_imu_.linear_acceleration.z),

                                                     Vector3(ros_handler.ros_imu_.angular_velocity.x,
                                                             ros_handler.ros_imu_.angular_velocity.y,
                                                             ros_handler.ros_imu_.angular_velocity.z),
                                                             dt);

        }

        /*if(ros_handler.new_image_){
            pose_number++;

            ///assigning values from the visual-odometry rostopic in the current pose
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

            ///Create odometry (Between) factors between consecutive poses
            graph.emplace_shared<BetweenFactor<Pose3> >(X(pose_number -1), X(pose_number), inc_odom, noise_values.::odometry_noise);

            initial_values.insert(X(pose_number), Pose3(previous_pose.matrix()*inc_odom.matrix()));

            previous_svo_pose = current_svo_pose;

        }*/

        ///ArUco object
        ///ArUcoLandmark represents one landmark marker whereas ArUcoMarkers represents all the markers detected in one scene.
        ArUcoMarkers aruco_marker;  //keeps track of all markers in the frame
        if(ros_handler.new_image_ && aruco_marker.hasMarkers(current_image)){
            pose_number++;

            //Add the Preintegrated IMU Factor---> Only to be added when there is a measurement of landmarks. Othertimes just integrate it and make the factor.
            PreintegratedImuMeasurements *preint_imu = dynamic_cast<PreintegratedImuMeasurements*>(imu_preintegrated);
            ImuFactor imu_factor(X(pose_number-1), V(pose_number-1),
                                 X(pose_number  ), V(pose_number  ),
                                 B(pose_number-1),
                                 *preint_imu);


            graph.add(imu_factor);

            ///Adding the constant IMU Bias
            imuBias::ConstantBias zero_bias(Vector3(0, 0, 0), Vector3(0, 0, 0));
            graph.add(BetweenFactor<imuBias::ConstantBias>(B(pose_number-1),
                                                           B(pose_number ),
                                                           zero_bias, noise_values.bias_noise_model_));

            ///Adding Constant Velocity factor between the Velocities
            Vector3 zero_difference_velocity(0, 0, 0);
            graph.add(BetweenFactor<Vector3>(V(pose_number -1),
                                             V(pose_number),
                                             zero_difference_velocity,
                                             noise_values.constant_velocity_noise_));


            //Add initial values for X, V, B
            initialization_values.nav_state_current_state_ = imu_preintegrated->predict(initialization_values.nav_state_prev_state_, initialization_values.prev_bias_);
            initial_values.insert(X(pose_number), initialization_values.nav_state_current_state_.pose());
            initial_values.insert(V(pose_number), initialization_values.nav_state_current_state_.v());
            initial_values.insert(B(pose_number), initialization_values.prev_bias_);


            //For every aruco marker detected add the corners to the factor graph and initialize the values
            for(int i = 0; i < aruco_marker.markers_.size(); i++){

                ArUcoLandmark aruco_landmark(aruco_marker.markers_[i]);
                ///Assigning measurement pixels here for all 4 corners
                Point2 measurement_tl(aruco_landmark.corner_points_[0], aruco_landmark.corner_points_[1]);
                Point2 measurement_tr(aruco_landmark.corner_points_[2], aruco_landmark.corner_points_[3]);
                Point2 measurement_br(aruco_landmark.corner_points_[4], aruco_landmark.corner_points_[5]);
                Point2 measurement_bl(aruco_landmark.corner_points_[6], aruco_landmark.corner_points_[7]);


                //assign the landmark index based on if the landmark has been detected before or not.
                unsigned int landmark_index = aruco_landmark.getLandmarkIndex(vector_aruco_landmarks); //returns the landmark index of the TL corner


                //add the landmark factor to the factor graph
                graph.emplace_shared<GenericProjectionFactor<Pose3, Point3, Cal3_S2> >(measurement_tl, noise_values.pixel_noise_,
                                                                                       Symbol('x', pose_number),
                                                                                       Symbol('l', landmark_index), initialization_values.K_);

                graph.emplace_shared<GenericProjectionFactor<Pose3, Point3, Cal3_S2> >(measurement_tr, noise_values.pixel_noise_,
                                                                                       Symbol('x', pose_number),
                                                                                       Symbol('l', landmark_index + 1), initialization_values.K_);

                graph.emplace_shared<GenericProjectionFactor<Pose3, Point3, Cal3_S2> >(measurement_br, noise_values.pixel_noise_,
                                                                                       Symbol('x', pose_number),
                                                                                       Symbol('l', landmark_index + 2), initialization_values.K_);

                graph.emplace_shared<GenericProjectionFactor<Pose3, Point3, Cal3_S2> >(measurement_bl, noise_values.pixel_noise_,
                                                                                       Symbol('x', pose_number),
                                                                                       Symbol('l', landmark_index + 3), initialization_values.K_);



                if(isMarkerDetectedFirstTime(aruco_landmark.aruco_marker_.id, vector_aruco_marker_ids)){

                    if (config_params.batch_)
                        aruco_landmark.setCornerPointsInWorldFrame(initial_values.at<Pose3>(X(pose_number)));//depending on if its batch or incremental this would change

                    else aruco_landmark.setCornerPointsInWorldFrame(previous_pose); //sending the previous pose assuming previous pose is the last well known location of the robot


                    //Adding prior to the landmarks
                    static auto kPointPrior = noiseModel::Isotropic::Sigma(3, 0.05); //Need to understand its purpose. Try without.
                    graph.emplace_shared<PriorFactor<Point3> >(Symbol('l', landmark_index), Point3(aruco_landmark.wHtl_.translation()),
                            kPointPrior);

                    graph.emplace_shared<PriorFactor<Point3> >(Symbol('l', landmark_index + 1), Point3(aruco_landmark.wHtr_.translation()),
                            kPointPrior);

                    graph.emplace_shared<PriorFactor<Point3> >(Symbol('l', landmark_index + 2), Point3(aruco_landmark.wHbr_.translation()),
                            kPointPrior);

                    graph.emplace_shared<PriorFactor<Point3> >(Symbol('l', landmark_index + 3), Point3(aruco_landmark.wHbl_.translation()),
                            kPointPrior);



                    //Assigning the initial values to the corner points in world frame
                    initial_values.insert<Point3>(Symbol('l', landmark_index), Point3(aruco_landmark.wHtl_.translation()));
                    initial_values.insert<Point3>(Symbol('l', landmark_index + 1), Point3(aruco_landmark.wHtr_.translation()));
                    initial_values.insert<Point3>(Symbol('l', landmark_index + 2), Point3(aruco_landmark.wHbr_.translation()));
                    initial_values.insert<Point3>(Symbol('l', landmark_index + 3), Point3(aruco_landmark.wHbl_.translation()));


                    //Adding factor between corners
                    graph.emplace_shared<BetweenFactor<Point3> >(L(landmark_index), L(landmark_index + 1), Point3((Pose3(aruco_landmark.wHtl_.matrix()*aruco_landmark.M_H_TLTR_)).translation()), noise_values.corner_noise_);
                    graph.emplace_shared<BetweenFactor<Point3> >(L(landmark_index + 1), L(landmark_index + 2), Point3((Pose3(aruco_landmark.wHtl_.matrix()*aruco_landmark.M_H_TRBR_)).translation()), noise_values.corner_noise_);
                    graph.emplace_shared<BetweenFactor<Point3> >(L(landmark_index + 2), L(landmark_index + 3), Point3((Pose3(aruco_landmark.wHtl_.matrix()*aruco_landmark.M_H_BRBL_)).translation()), noise_values.corner_noise_);
                    graph.emplace_shared<BetweenFactor<Point3> >(L(landmark_index + 3), L(landmark_index), Point3((Pose3(aruco_landmark.wHtl_.matrix()*aruco_landmark.M_H_BLTL_)).translation()), noise_values.corner_noise_);



                    if(config_params.debug_){
                        aruco_landmark.printCornerPoints();
                        aruco_landmark.printCameraTransforms();

                    }

                }

            }

        }

        if(config_params.incremental_update_){


            isam2.update(graph, initial_values);
            isam2.update();

            current_estimate = isam2.calculateEstimate();
            previous_pose = current_estimate.at<Pose3>(X(pose_number));

            cout<<current_estimate.at<Pose3>(X(pose_number)).x()<<" "<<
                current_estimate.at<Pose3>(X(pose_number)).y()<<" "<<
                current_estimate.at<Pose3>(X(pose_number)).z()<<"\n";


            ///Update the prev_state of NavState for IMU prediction
            // Overwrite the beginning of the preintegration for the next step.
            initialization_values.nav_state_prev_state_ = NavState(current_estimate.at<Pose3>(X(pose_number)),
                                  current_estimate.at<Vector3>(V(pose_number)));
            initialization_values.prev_bias_ = current_estimate.at<imuBias::ConstantBias>(B(pose_number));

            // Reset the preintegration object.
            imu_preintegrated->resetIntegrationAndSetBias(initialization_values.prev_bias_);

            //Publishing Pose Transforms for RViz Visualization
            rviz_current_pose_corner.assign_current_pose(previous_pose);
            rviz_current_pose_corner.assign_corner_position(current_estimate, vector_aruco_marker_ids.size()*4);


            //Publishing Landmark corners for RViz Visualization
            ros_handler.publishCurrentTFPose(rviz_current_pose_corner.pose_transform_);
            ros_handler.publishMarkerCorners(rviz_current_pose_corner.corner_points_);


            graph.resize(0);
            initial_values.clear();

        }


        //Solve the batch if number of poses > 300
        if(config_params.batch_ && pose_number > config_params.max_poses_batch_){

            LevenbergMarquardtOptimizer optimizer(graph, initial_values, LMParams);
            Values batch_estimate = optimizer.optimize();

            //Print graph
            graph.print();

            //Dump the pose and landmark values
            DumpValues dump_pose_landmark_values;
            dump_pose_landmark_values.dump2File(batch_estimate, config_params.max_poses_batch_, vector_aruco_marker_ids.size()*4 ); // multiplied by four as number of corner points = number of markers * 4

            break;

        }

        ros_handler.resetBools();
        ros::spinOnce();
        rate.sleep();

    }

    return 0;

}
