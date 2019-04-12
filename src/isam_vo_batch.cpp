//
// Created by ALG on 23-01-2019.
//

//
// Created by socrob on 08-01-2019.
//

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
#include <aruco.h>
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

//tf include files
#include <tf/transform_broadcaster.h>

//visualization_msgs
#include <visualization_msgs/Marker.h>
#include <opencv-3.3.1-dev/opencv/cxeigen.hpp>

//Visualizing the predicted and calculated corners
#include "VisualizationCorners.h"

//for dumping the pose values into a different file
#include <fstream>

///Not using the combined IMU (IMU(Preintegrated) + GPS) in this program. Using just the preintegrated IMU.





using namespace std;
using namespace gtsam;



using symbol_shorthand::X; // Pose3 (x,y,z,r,p,y)
using symbol_shorthand::V; // Vel   (xdot,ydot,zdot)
using symbol_shorthand::B; // Bias  (ax,ay,az,gx,gy,gz)
using symbol_shorthand::L; // Landmark  (ax,ay,az,gx,gy,gz)

//Todo: Make one for landmark too
//using symbol_shorthand::L;

Eigen::Quaterniond orientation_curr(1, 0, 0, 0);
Eigen::Vector3d acc_curr, angvel_curr;

// This will either be PreintegratedImuMeasurements (for ImuFactor) or
// PreintegratedCombinedMeasurements (for CombinedImuFactor).
PreintegrationType *imu_preintegrated_;

int header_seq;
//Using a bool value to check if new imu value was used
//Todo : The bool new_imu thing could be a failure point.. This is being used to make sure the imu values are fresh.
// There should be other ways of doing this.
// Like passing values in the callback function and putting to 0 the previous acceleration and angular velocity values.
bool new_imu = false;
bool new_image = false;
bool new_odom = false;




//global variable for incremental odom
geometry_msgs::PoseStamped svo_pose;

///temp condition for simulated data
int temp_it = 0;

struct ArucoCorners {
    int aruco_corners_index;
    Eigen::Matrix4d orientation;

};

///A simple array to store all the ids of detected landmarks would also have been fine, but okay.
struct detected_landmarks{
    bool detected;
    int id;
    int actual_index;   ///only the TL index is stored. TR, BR, BL are all assumed to have position. so the actual index has values like 1, 5, 9, etc 2,3,4 is for TR, BR, BL and so on
};

//list of detected landmarks
vector<detected_landmarks> list_detected_landmarks;

///The image frame which holds the value of current camera image
cv::Mat main_image(1048, 1328, CV_8UC3, cv::Scalar(0,0,0));


bool isDetected(int id){

    for(int i=0; i<list_detected_landmarks.size(); i++){
        if(id == list_detected_landmarks[i].id)
            return true;
    }

    return false;
}

int getIndex(int id){
    for(int i=0; i<list_detected_landmarks.size(); i++){
        if(id == list_detected_landmarks[i].id)
            return list_detected_landmarks[i].actual_index; ///returning the TL index of the detected landmark
    }

}

void odomConstraintCallback(const geometry_msgs::PoseStampedConstPtr& msg){

    svo_pose.pose.position.x = msg->pose.position.x;
    svo_pose.pose.position.y = msg->pose.position.y;
    svo_pose.pose.position.z = msg->pose.position.z;

    svo_pose.pose.orientation.x = msg->pose.orientation.x;
    svo_pose.pose.orientation.y = msg->pose.orientation.y;
    svo_pose.pose.orientation.z = msg->pose.orientation.z;
    svo_pose.pose.orientation.w = msg->pose.orientation.w;

    new_odom = true;


}

Pose3 getIncOdom(const Pose3 prev_svo_pose, Eigen::Matrix4d& T){

    Eigen::Matrix4d X_0, X_1;
    X_0 = prev_svo_pose.matrix();
    X_1 = Pose3(Rot3(svo_pose.pose.orientation.w, svo_pose.pose.orientation.x, svo_pose.pose.orientation.y, svo_pose.pose.orientation.z),
                Point3(svo_pose.pose.position.x, svo_pose.pose.position.y, svo_pose.pose.position.z)).matrix();

    T = X_0.inverse()*X_1;
    return Pose3(T);

}

void imageCallback(const sensor_msgs::ImageConstPtr& msg){

    main_image = cv_bridge::toCvCopy(msg, "mono8")->image;
    //cout<<"New image! "<<endl;
    //cout<<"Size: "<<main_image.rows<<" "<<main_image.cols<<endl;

    new_image = true;

}

void IMUCallback(const sensor_msgs::Imu::ConstPtr& msg)
{

    orientation_curr.x()= msg->orientation.x;
    orientation_curr.y()= msg->orientation.y;
    orientation_curr.z()= msg->orientation.z;
    orientation_curr.w()= msg->orientation.w;

    acc_curr[0]= msg->linear_acceleration.x;
    acc_curr[1]= msg->linear_acceleration.y;
    acc_curr[2]= msg->linear_acceleration.z;

    angvel_curr[0] = msg->angular_velocity.x;
    angvel_curr[1] = msg->angular_velocity.y;
    angvel_curr[2] = msg->angular_velocity.z;


    ////header message for comparison;
    header_seq = msg->header.seq;

    //cout<<angvel_curr<<endl;
    //cout<<"New IMU"<<endl;

    ///Telling the main program that new imu measurement has arrived
    new_imu = true;


}

Pose3 getArucoMarkerOrientation(vector<ArucoCorners> vec_aruco_markers, int land_index ){
    Eigen::Matrix4d current_marker_orientation;
    for(int i=0; i<vec_aruco_markers.size(); i++) {
        if (vec_aruco_markers[i].aruco_corners_index == land_index) {
            current_marker_orientation = vec_aruco_markers[i].orientation;
        }
    }
    current_marker_orientation(0,3) = 0.0;
    current_marker_orientation(1,3) = 0.0;
    current_marker_orientation(2,3) = 0.0;

    return Pose3(current_marker_orientation);
}

int main(int argc, char** argv) {

    ros::init(argc, argv, "isam_ros");
    ros::NodeHandle nh;

    ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);

    //Publishing and Subscribing
    ros::Subscriber sub = nh.subscribe("mavros/imu/data", 1000, IMUCallback);

    //Subscriber to svo cam topic
    ros::Subscriber sub1 = nh.subscribe("/svo/pose_cam/0", 1000, odomConstraintCallback);

    //Publishing the orientation msg
    ros::Publisher pub = nh.advertise<geometry_msgs::PoseStamped>
            ("space_cobot/isam_ros/orientation", 1000);

    ros::Subscriber image_sub = nh.subscribe("/camera/image_rect_color", 1000, imageCallback);

    ros::Rate rate(20);

    ///Declaring the initial state here
    Eigen::Matrix<double,10,1> initial_state = Eigen::Matrix<double,10,1>::Zero();


    /// Define the camera calibration parameters
    Cal3_S2::shared_ptr K(new Cal3_S2(532.13605, 529.40496, 0.0, 472.2739, 376.43908));

    // Define the camera observation noise model                                    ///dims, value
    // noiseModel::Isotropic::shared_ptr measurementNoise = noiseModel::Isotropic::Sigma(8, 1.0); // one pixel in u and v
    noiseModel::Isotropic::shared_ptr pixel_noise = noiseModel::Isotropic::Sigma(2, 100.0);

    ///ArUco object that has marker size and camera parameters
    Arucoalg ar_object;
    ISAM2Params parameters;

    ////////////////////FILESSTREAM
    ofstream outFileSVO;
    ofstream outfile_poses, outfile_landmarks_corners;

    outFileSVO.open(OUTSVO, std::ios_base::app);

    outfile_poses.open(OUTFILE_POSE, std::ios_base::app);
    outfile_landmarks_corners.open(OUTFILE_LAND, std::ios_base::app);


    //TODO(Need to tune the parameters.)----> Understand the meaning of each parameter

    //parameters.optimizationParams = ISAM2DoglegParams();
    //parameters.relinearizeThreshold = 0.1;
    //parameters.relinearizeSkip = 5;
    //parameters.
    parameters.enableRelinearization = true;

    ///Batch:  assigning params
    LevenbergMarquardtParams LMParams;
    //LMParams.setMaxIterations(100);
    //LMParams.setAbsoluteErrorTol(10000);
    LMParams.setVerbosity("ERROR");
    LMParams.setlambdaUpperBound(1e30);
    LMParams.setRelativeErrorTol(1e-25);
    //LMParams.set

    //changing the factorization method to QR
    //parameters.factorization=ISAM2Params::QR;
    ISAM2 isam(parameters);

    //Assemble initial quaternion through gtsam constructor ::quaternion(w,x,y,z);
    //Todo: Need to change the values of the prior pose depending on the task

    ///Adding a prior on the initial state
    Rot3 prior_rotation = Rot3::Quaternion(1,0, 0, 0);///This is used to make the quaternion w, x, y, z (Since w is entered last in the entry and we want it first)
    Point3 prior_point(initial_state.head<3>());///This means get the first three values
    Pose3 prior_pose(prior_rotation, prior_point);
    Vector3 prior_velocity(initial_state.tail<3>()); ///This means get the last three values
    imuBias::ConstantBias prior_imu_bias; // assume zero initial bias

    Values initial_values;
    unsigned int pose_number = 0;
    initial_values.insert(X(pose_number), prior_pose);



    // Assemble prior noise model and add it the graph.
    //Changed the values of the noise model
    noiseModel::Diagonal::shared_ptr pose_noise_model = noiseModel::Diagonal::Sigmas((Vector(6) << 0.01, 0.01, 0.01, 0.05, 0.05, 0.05).finished()); // rad,rad,rad,m, m, m ///Changed the values.. increased by an order.
    noiseModel::Diagonal::shared_ptr velocity_noise_model = noiseModel::Isotropic::Sigma(3,0.1); // m/s
    noiseModel::Diagonal::shared_ptr bias_noise_model = noiseModel::Isotropic::Sigma(6,1e-3);

    //Odometry Noise
    // For simplicity, we will use the same noise model for each odometry factor
    noiseModel::Diagonal::shared_ptr odometryNoise = noiseModel::Diagonal::Sigmas((Vector(6)<< 0.15, 0.15, 0.15, 0.15, 0.15, 0.15).finished());
    noiseModel::Diagonal::shared_ptr corner_noise = noiseModel::Diagonal::Sigmas((Vector(3)<< 0.001, 0.001, 0.001).finished());


    ///This graph is the main graph where all the factors are added and all computations are done
    //// Add all prior factors (pose, velocity, bias) to the graph.
    NonlinearFactorGraph graph;

    graph.add(PriorFactor<Pose3>(X(pose_number), prior_pose, pose_noise_model));

    /*///The correct prior on landmark
    static auto kPointPrior = noiseModel::Isotropic::Sigma(3, 0.1);
    graph.emplace_shared<PriorFactor<Point3> >(Symbol('l', 0), prior_point, kPointPrior);*/


    // We use the sensor specs to build the noise model for the IMU factor.
    double accel_noise_sigma = 0.324;


    double gyro_noise_sigma = 0.000205689024915;
    double accel_bias_rw_sigma = 0.4905;
    double gyro_bias_rw_sigma = 0.000001454441043;
    Matrix33 measured_acc_cov = Matrix33::Identity(3,3) * pow(accel_noise_sigma,2);
    Matrix33 measured_omega_cov = Matrix33::Identity(3,3) * pow(gyro_noise_sigma,2);
    Matrix33 integration_error_cov = Matrix33::Identity(3,3)*1e-8; // error committed in integrating position from velocities
    Matrix33 bias_acc_cov = Matrix33::Identity(3,3) * pow(accel_bias_rw_sigma,2);
    Matrix33 bias_omega_cov = Matrix33::Identity(3,3) * pow(gyro_bias_rw_sigma,2);
    Matrix66 bias_acc_omega_int = Matrix::Identity(6,6)*1e-5; // error in the bias used for preintegration

    boost::shared_ptr<PreintegratedCombinedMeasurements::Params> p = PreintegratedCombinedMeasurements::Params::MakeSharedD(0.0);
    // PreintegrationBase params:
    //double kGravity = 9.81;
    //auto params = PreintegrationParams::MakeSharedU(kGravity);
    p->accelerometerCovariance = measured_acc_cov; // acc white noise in continuous
    p->integrationCovariance = integration_error_cov; // integration uncertainty continuous
    // should be using 2nd order integration
    // PreintegratedRotation params:
    p->gyroscopeCovariance = measured_omega_cov; // gyro white noise in continuous
    // PreintegrationCombinedMeasurements params:
    p->biasAccCovariance = bias_acc_cov; // acc bias in continuous
    p->biasOmegaCovariance = bias_omega_cov; // gyro bias in continuous
    p->biasAccOmegaInt = bias_acc_omega_int;

    imu_preintegrated_ = new PreintegratedImuMeasurements(p, prior_imu_bias);

    // Store previous state for the imu integration and the latest predicted outcome.
    ///NEW EDITING 04/01
    Pose3 prev_state = prior_pose;
    Pose3 prop_state = prev_state;
    Pose3 prev_svo_pose = prior_pose;
    //NavState prev_state(prior_pose, prior_velocity);
    //NavState prop_state = prev_state;
    imuBias::ConstantBias prev_bias = prior_imu_bias;

    // Keep track of the total error over the entire run for a simple performance metric.
    double current_position_error = 0.0, current_orientation_error = 0.0;

    double output_time = 0.0;
    double dt = 0.05;  // The real system has noise, but here, results are nearly
    // exactly the same, so keeping this for simplicity.


    ///This is to get new initialization poses everytime
    Pose3 ini_pose_alg = prior_pose;


    //MAX LAND INDEX
    unsigned int max_land_index= 0;

    //To check if the landmark is getting initiallized multiple times
    static int land_initialization = 0;

    bool first_ever = false;

    //Object for visualizing corners
    VisualizationCorners vis_corn;
    bool untouched_corners = true;

    ///Vector for storing the aruco corners
    vector<ArucoCorners> aruco_corners_vector;


    ///////////////////////////////////////////////////////////////////////////////////////////

    Eigen::Matrix4d M_H_TLTR, M_H_TRBR, M_H_BRBL, M_H_BLTL;

    M_H_TLTR<< 1, 0, 0, 0.185,
            0, 1, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1;

    M_H_TRBR << 1, 0, 0, 0,
            0, 1, 0, -0.185,
            0, 0, 1, 0,
            0, 0, 0, 1;

    M_H_BRBL << 1, 0, 0, -0.185,
            0, 1, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1;

    M_H_BLTL << 1, 0, 0, 0,
            0, 1, 0, 0.185,
            0, 0, 1, 0,
            0, 0, 0, 1;
////////////////////////////////////////////////////////////////////////////////////////////



    while(ros::ok()) {

        //for visualizing point of the landmark
        visualization_msgs::Marker points;
        points.header.frame_id = "/world";
        points.header.stamp = ros::Time::now();
        points.ns = "points";
        points.action = visualization_msgs::Marker::ADD;
        points.pose.orientation.w = 1.0;
        points.id = 0;
        points.type = visualization_msgs::Marker::POINTS;
        // POINTS markers use x and y scale for width/height respectively
        points.scale.x = 0.05;
        points.scale.y = 0.05;
        // Points are green
        points.color.g = 1.0f;
        points.color.a = 1.0;

        if(new_imu) { //The new imu factor is not added although calculated
            Eigen::Matrix<double, 6, 1> imu = Eigen::Matrix<double, 6, 1>::Zero();
            ///Getting the imu measurement and putting in imu vector
            for (int i = 0; i < 3; i++) {

                imu[i] = 0;
                imu[i + 3] = angvel_curr[i];

            }

            /// Adding the IMU preintegration.
            //TODO: this has the acceleration values too. The acceleration values are given as 0. which is not ideal.
            //TODO: The temp (but bad) solution employed is to put the accel noise to be very high.
            //TODO: Make a separate factor for just adding the angular velocities between poses.

            imu_preintegrated_->integrateMeasurement(imu.head<3>(), imu.tail<3>(), dt);
        }

        if(new_odom){
            pose_number++;
            Eigen::Matrix4d X1, X2, Trans;
            //This is where we get the transform from previous to current pose to be used in Between_Factor
            //TODO(ALG): Possible source of error. See how inc_odom is constructed and get it verified.
            Pose3 inc_odom = getIncOdom(prev_svo_pose, Trans);

            //Setting previous svo_pose to the current one after getting the transform from previous to current
            prev_svo_pose = Pose3(Rot3(svo_pose.pose.orientation.w, svo_pose.pose.orientation.x, svo_pose.pose.orientation.y, svo_pose.pose.orientation.z),
                                  Point3(svo_pose.pose.position.x, svo_pose.pose.position.y, svo_pose.pose.position.z));


            // Create odometry (Between) factors between consecutive poses
            graph.emplace_shared<BetweenFactor<Pose3> >(X(pose_number -1), X(pose_number), inc_odom, odometryNoise);

            //Add initial values. Here adding initial values is important because the factor function doesn't add the initial values

            X1 = prev_state.matrix();
            //TODO(ALG): Possible source of error. Recheck this.
            X2 = X1*Trans;

            Pose3 temp_ini_new_state(X2);
            initial_values.insert(X(pose_number), temp_ini_new_state);

            //TODO: Dump the svo values to a file fo comparison with the ground truth
            outFileSVO<<svo_pose.pose.position.x<<","<<svo_pose.pose.position.y<<","<<svo_pose.pose.position.z<<"\n";

            new_odom = false;


        }

        ///postiion and orientation transform from camera to ArUco marker
        ///declaring it here so it can be accessed from outside the for loop of the markers _size iterating over all the markers
        double position_transform[3], orientation_transform[3];
        for(int itj = 0; itj< 3; itj++) {
            position_transform[itj] = 0;
            orientation_transform[itj] = 0;
        }


        /****************************************************************************************************************
         *************THE LANDMARK MEASUREMENTS START HERE
         * *************************************************************************************************************/

        //Initialise the corner vals with 0s
        Eigen::VectorXd corner_vals;
        vector<aruco::Marker> Markers;
        ar_object.MDetector.detect(main_image, Markers, ar_object.getCamParam(), MARKER_SIZE);
        //cout<<"Marker Size: "<<Markers.size()<<endl;
        Eigen::Quaterniond alg_gtsam_quat;
        Eigen::Vector3d alg_gtsam_pos;


        if (new_image && !Markers.empty()) { ///----------------------> that means we have a measurement


            ///looping over al the markers detected
            for (int j = 0; j < Markers.size(); j++) {


                Markers[j].OgreGetPoseParameters(position_transform, orientation_transform);


                cout << "Marker ID: " << Markers[j].id << " at position " << position_transform[0] << " "
                     << position_transform[1] << " " << position_transform[2] << " and orientation: "
                     << orientation_transform[0] << " " << orientation_transform[1] << " " << orientation_transform[2]
                     << endl;

                ///Corner values are assigned here
                corner_vals = (Eigen::VectorXd(8)
                        << Markers[j][0].x, Markers[j][0].y, Markers[j][1].x, Markers[j][1].y, Markers[j][2].x, Markers[j][2].y, Markers[j][3].x, Markers[j][3].y).finished();

                ///Assigning measurement pixels here for all 4 corners
                Point2 measurementTL(corner_vals[0], corner_vals[1]);
                Point2 measurementTR(corner_vals[2], corner_vals[3]);
                Point2 measurementBR(corner_vals[4], corner_vals[5]);
                Point2 measurementBL(corner_vals[6], corner_vals[7]);

                ///This is the landmark index that we input to the factor graph . i.e. depending on the corner (TL, BL etc) we assign it the correct value
                unsigned int land_index = 0;
                bool first_time_detected = false;

                ///if this is the first landmark ever
                if (list_detected_landmarks.empty()) {

                    first_ever = true;

                    first_time_detected = true;
                    detected_landmarks temp;
                    temp.id = Markers[j].id;
                    temp.detected = true;
                    temp.actual_index = 0;
                    list_detected_landmarks.push_back(temp);

                }

                ///detecting for the first time

                if (!isDetected(Markers[j].id)) {
                    first_time_detected = true;
                    detected_landmarks temp;
                    temp.id = Markers[j].id;
                    temp.detected = true;
                    temp.actual_index = list_detected_landmarks.back().actual_index + 4;
                    list_detected_landmarks.push_back(temp);


                }

                ///After making sure the landmark is added/present in the list of detected landmarks, we get the TL index
                land_index = getIndex(Markers[j].id);

                //To get the max index outside the scope and visualize on rviz
                if (land_index + 4 > max_land_index)
                    max_land_index = land_index + 4;



                ///Depending on the TL index we add the 4 factors to the graph
                graph.emplace_shared<GenericProjectionFactor<Pose3, Point3, Cal3_S2> >(measurementTL, pixel_noise,
                                                                                       Symbol('x', pose_number),
                                                                                       Symbol('l', land_index), K);
                graph.emplace_shared<GenericProjectionFactor<Pose3, Point3, Cal3_S2> >(measurementTR, pixel_noise,
                                                                                       Symbol('x', pose_number),
                                                                                       Symbol('l', land_index + 1), K);
                graph.emplace_shared<GenericProjectionFactor<Pose3, Point3, Cal3_S2> >(measurementBR, pixel_noise,
                                                                                       Symbol('x', pose_number),
                                                                                       Symbol('l', land_index + 2), K);
                graph.emplace_shared<GenericProjectionFactor<Pose3, Point3, Cal3_S2> >(measurementBL, pixel_noise,
                                                                                       Symbol('x', pose_number),
                                                                                       Symbol('l', land_index + 3), K);



                //TODO(ALG): add constraints between aruco marker corners
                //TODO(ALG): Get the orientation of the ArUco marker
                //TODO(ALG): For now lets just fix the orientation as the first orientation we saw.

                //TODO(ALG): Get the marker orientation in world frame of the particular land_index
                //////////////////////----------WRONG CALCULATIONS!!!!!!!!!!!!!!!!!!!!!!!!!!!!-------------
                /////////////////////-------This is just giving the value of the point(0.185, 0,0  of the marker) in the world frame.
                Pose3 temp_current_marker_pose = getArucoMarkerOrientation(aruco_corners_vector, land_index);

                graph.emplace_shared<BetweenFactor<Point3> >(L(land_index), L(land_index + 1), Point3((Pose3(temp_current_marker_pose.matrix()*M_H_TLTR)).translation()), corner_noise);
                graph.emplace_shared<BetweenFactor<Point3> >(L(land_index + 1), L(land_index + 2), Point3((Pose3(temp_current_marker_pose.matrix()*M_H_TRBR)).translation()), corner_noise);
                graph.emplace_shared<BetweenFactor<Point3> >(L(land_index + 2), L(land_index + 3), Point3((Pose3(temp_current_marker_pose.matrix()*M_H_BRBL)).translation()), corner_noise);
                graph.emplace_shared<BetweenFactor<Point3> >(L(land_index + 3), L(land_index), Point3((Pose3(temp_current_marker_pose.matrix()*M_H_BLTL)).translation()), corner_noise);

                //Making an object of VisualizationCorners so as to visualize the predicted and measured values

                //temp to avoid this visualization
                //untouched_corners = true;
                if(!untouched_corners){
                    static VisualizationCorners temp_vis_corn(pose_number, land_index, measurementTL, measurementTR, measurementBR, measurementBL, main_image, K);
                    vis_corn = temp_vis_corn;
                    untouched_corners = false;
                }

                ///Adding initial values to each of the landmarks which are detect for the first time
                if (first_time_detected) {

                    land_initialization++;

                    ///If its detected for the first time then we need to add an initial value to the current pose
                    //TODO: Get the initial values of each of the corner points (Point3 values) in the world coordintes

                    Eigen::Matrix4d wHo;
                    Eigen::Matrix4d oHc, cHl, lH_tl, lH_tr, lH_br, lH_bl;

                    wHo = prop_state.matrix();
                    cout << "This landmark " << Markers[j].id << "has been detected for the first time " << endl;


                    cv::Mat rotation_matrix_cv;
                    cv::Rodrigues(Markers[j].Rvec, rotation_matrix_cv);

                    Eigen::Matrix3d rotation_matrix_eigen;
                    cv::cv2eigen(rotation_matrix_cv, rotation_matrix_eigen);

                    /*Pose3 temp_ini_pose = Pose3(Rot3::Rodrigues(orientation_transform[0], orientation_transform[1],
                                                                orientation_transform[2]),
                                                Point3(position_transform[0], position_transform[1],
                                                       position_transform[2]));*/

                    Pose3 temp_ini_pose = Pose3(Rot3(rotation_matrix_eigen),
                                                Point3(position_transform[0], position_transform[1],
                                                       position_transform[2]));


                    //TODO(ALG): Put this in a class. "Constants". Which will have all the values of the constants. There should be literally no calculations in the main()

                    wHo = prop_state.matrix();
                    /*oHc << 0, 0, 1, 0,
                            -1, 0, 0, 0,
                            0, -1, 0, 0,
                            0, 0, 0, 1;*/

                    //The revelation of markers being upside down gave rise to this correction of camera frame
                    oHc << 0, 0, 1, 0,
                            1, 0, 0, 0,
                            0, 1, 0, 0,
                            0, 0, 0, 1;


                    cHl = temp_ini_pose.matrix();

                    lH_tl << 1, 0, 0, -MARKER_SIZE / 2,
                            0, 1, 0, MARKER_SIZE / 2,
                            0, 0, 1, 0,
                            0, 0, 0, 1;

                    lH_tr << 1, 0, 0, MARKER_SIZE / 2,
                            0, 1, 0, MARKER_SIZE / 2,
                            0, 0, 1, 0,
                            0, 0, 0, 1;

                    lH_br << 1, 0, 0, MARKER_SIZE / 2,
                            0, 1, 0, -MARKER_SIZE / 2,
                            0, 0, 1, 0,
                            0, 0, 0, 1;

                    lH_bl << 1, 0, 0, -MARKER_SIZE / 2,
                            0, 1, 0, -MARKER_SIZE / 2,
                            0, 0, 1, 0,
                            0, 0, 0, 1;

                    Pose3 wH_tl = Pose3(wHo * oHc * cHl * lH_tl);
                    Pose3 wH_tr = Pose3(wHo * oHc * cHl * lH_tr);
                    Pose3 wH_br = Pose3(wHo * oHc * cHl * lH_br);
                    Pose3 wH_bl = Pose3(wHo * oHc * cHl * lH_bl);

                    //TODO(ALG): Get marker orientation in world frame and save it in the different data structure
                    ArucoCorners temp;
                    temp.aruco_corners_index = land_index;
                    temp.orientation = wH_tl.matrix();
                    aruco_corners_vector.push_back(temp);

                    cout << "Putting initial values to the landmarks of land index" << land_index << ", "
                         << land_index + 1 << ", " << land_index + 2 << ", " << land_index + 3 << endl;
                    cout << "One example of values being put is " << Point3(wH_tl.translation()) << endl;


                    /*// Prior on the Landmark 0
                    // Add a prior on landmark l0
                    if (first_ever){
                        cout<<"Giving prior to the first ever landmark"<<endl;

                        static auto kPointPrior = noiseModel::Isotropic::Sigma(3, 0.1);
                        graph.emplace_shared<PriorFactor<Point3> >(Symbol('l', 0), Point3(wH_tl.translation()),
                                                                   kPointPrior);

                        first_ever = false;
                    }*/


                    ///GIVING PRIORS TO EVERY LANSMARK HERE
                    static auto kPointPrior = noiseModel::Isotropic::Sigma(3, 0.05);
                    graph.emplace_shared<PriorFactor<Point3> >(Symbol('l', land_index), Point3(wH_tl.translation()),
                                                               kPointPrior);

                    graph.emplace_shared<PriorFactor<Point3> >(Symbol('l', land_index + 1), Point3(wH_tr.translation()),
                                                               kPointPrior);

                    graph.emplace_shared<PriorFactor<Point3> >(Symbol('l', land_index + 2), Point3(wH_br.translation()),
                                                               kPointPrior);

                    graph.emplace_shared<PriorFactor<Point3> >(Symbol('l', land_index + 3), Point3(wH_bl.translation()),
                                                               kPointPrior);





                    // Adding initial values to the landmarks
                    initial_values.insert<Point3>(Symbol('l', land_index), Point3(wH_tl.translation()));
                    initial_values.insert<Point3>(Symbol('l', land_index + 1), Point3(wH_tr.translation()));
                    initial_values.insert<Point3>(Symbol('l', land_index + 2), Point3(wH_br.translation()));
                    initial_values.insert<Point3>(Symbol('l', land_index + 3), Point3(wH_bl.translation()));

                }

            }
        }

        ///If the number of nodes is more than 500 just do a batch operation and get out.
        if(pose_number > 300){

            //TODO(Batch): if the current thing wont run miultiple iterations, run this in a loop
            LevenbergMarquardtOptimizer optimizer(graph, initial_values, LMParams);
            Values currentEstimate = optimizer.optimize();

            //Print graph
            cout<<"The graph is:::::"<<endl;
            graph.print();


            ///dumping the values into files
            for(int i=0;i< 300; i++){
                //outFileX<<currentEstimate.at<Pose3>(X(i)).x()<<"\n";
                //outFileY<<currentEstimate.at<Pose3>(X(i)).y()<<"\n";
                //outFileZ<<currentEstimate.at<Pose3>(X(i)).z()<<"\n";

                outfile_poses<<currentEstimate.at<Pose3>(X(i)).x()<<"\n"<<currentEstimate.at<Pose3>(X(i)).y()<<"\n"<<currentEstimate.at<Pose3>(X(i)).z()<<"\n"<<"\n";
            }

            ///dumping the landmark positions too
            for(int i = 0; i<max_land_index; i++){
                outfile_landmarks_corners<<currentEstimate.at<Point3>(L(i)).x()<<"\n"<<currentEstimate.at<Point3>(L(i)).y()<<"\n"<<currentEstimate.at<Point3>(L(i)).z()<<"\n"<<"\n";
            }

            break;
        }


        //isam.update(graph, initial_values);
        //isam.update();
        //Values currentEstimate = isam.calculateEstimate();

        //LevenbergMarquardtOptimizer optimizer(graph, initial_values);
        //Values currentEstimate = optimizer.optimize();



        /// Reset the preintegration object.
        //imu_preintegrated_->resetIntegrationAndSetBias(prev_bias);


        prev_state = initial_values.at<Pose3>(X(pose_number));

        Vector3 gtsam_position = prev_state.translation();
        Quaternion gtsam_quat = prev_state.rotation().toQuaternion();

        alg_gtsam_quat = gtsam_quat;
        alg_gtsam_pos = gtsam_position;


        /*//Assigning the position values to the MARKER points
        //MARKER POINTS
        geometry_msgs::Point pp;
        for(int i=0; i<max_land_index; i++){

            pp.x = currentEstimate.at<Point3>(L(i)).x();
            pp.y = currentEstimate.at<Point3>(L(i)).y();
            pp.z = currentEstimate.at<Point3>(L(i)).z();
            std_msgs::ColorRGBA c;

            //landmark#1 is green
            if(i >= 0 && i<4)
                c.g = 1.0;
            //landmark #2 is red
            if(i>=4 && i< 8)
                c.r = 1.0;

            //landmark#3 is blue
            if(i>=8 && i < 12)
                c.b = 1.0;

            //landmark #4 is light orange
            if(i>=12 && i < 16) {
                c.r = 0.5;
                c.g = 0.5;
            }

            //landmark#5 is purple
            if(i >=16 && i< 20) {
                c.g = 0.8;
                c.b = 0.8;
            }

            //landmark#6 is red + blue
            if(i>= 20 && i <24){
                c.r = 0.8;
                c.b = 0.8;
            }

            //landmark #7 is RED + green
            if (i>= 24 && i < 28){
                c.r= 0.9;
                c.g = 0.45;
            }

            c.a = 1;

            points.points.push_back(pp);
            points.colors.push_back(c);

        }*/


        // Clear the factor graph and values for the next iteration
        //graph.resize(0);

        //TODO: If error , you might have to uncomment this
        //initial_values.clear();



        /// There are three rates at play here. (1) ROS rate (2) IMU rate (3) Camera FPS
        ///I need to make sure that image used is not from the previous imu state and only from the current state. every five poses is being assigned to the same landmarks and all poses after that because main_image is a global variable which will make new factors between poses and landmarks which shouldnt be there.
        /////The temporary solution is to put the image to black when no new image is being captured. That is, since imu rate is faster than fps, we just put the image_black for all the in between imu poses.


        // cout<< "Current Orientation: ";
        // cout<<alg_gtsam_quat.x()<<" "<<alg_gtsam_quat.y()<<" "<<alg_gtsam_quat.z()<<" "<<alg_gtsam_quat.w()<<endl;

        /*cout<<"Current Position: "<<endl;
        cout<<alg_gtsam_pos<<endl;

        geometry_msgs::PoseStamped orientation_msg;
        orientation_msg.pose.position.x = alg_gtsam_pos(0);
        orientation_msg.pose.position.y = alg_gtsam_pos(1);
        orientation_msg.pose.position.z = alg_gtsam_pos(2);

        orientation_msg.pose.orientation.x = alg_gtsam_quat.x();
        orientation_msg.pose.orientation.y = alg_gtsam_quat.y();
        orientation_msg.pose.orientation.z = alg_gtsam_quat.z();
        orientation_msg.pose.orientation.w = alg_gtsam_quat.w();*/

        //Visualization in rviz
        //TODO(ALG): Put this in a function. A class of visualization.

        ///Publish tf for Pose
        //Broadcasting the pose
        /*static tf::TransformBroadcaster br_pose;
        tf::Transform transform_pose;
        transform_pose.setOrigin(tf::Vector3( alg_gtsam_pos(0),  alg_gtsam_pos(1), alg_gtsam_pos(2)));
        tf::Quaternion q_pose(alg_gtsam_quat.x(), alg_gtsam_quat.y(), alg_gtsam_quat.z(), alg_gtsam_quat.w());
        //q_pose.setRPY(pose_nodes[i]->value().roll(), pose_nodes[i]->value().pitch(), pose_nodes[i]->value().yaw());

        transform_pose.setRotation(q_pose);
        br_pose.sendTransform(tf::StampedTransform(transform_pose, ros::Time::now(), "world", "space_cobot"));

        //Publish POINTS visualization_msgs for landmark corners
        marker_pub.publish(points);*/


        ///Publish tf for ground truth.



        //publish the data
        //pub.publish(orientation_msg);

        new_imu = false;
        new_image = false;
        new_odom = false;


        //Print the image
        //Calculate the corners
        /*if(!untouched_corners){

            vis_corn.calculatePredicted( currentEstimate.at<Pose3>(X(vis_corn.getPoseIndex())),
                                         currentEstimate.at<Point3>(L(vis_corn.getTLindex())),
                                         currentEstimate.at<Point3>(L(vis_corn.getTLindex() + 1)),
                                         currentEstimate.at<Point3>(L(vis_corn.getTLindex() + 2)),
                                         currentEstimate.at<Point3>(L(vis_corn.getTLindex() + 3)));

            cv::namedWindow("Corners");
            cv::imshow("Corners", vis_corn.drawOnImg());
            cv::waitKey(0);

        }*/

        //print the factor graph


        ros::spinOnce();
        rate.sleep();



    }

    //closing all the files
    outfile_landmarks_corners.close();
    outfile_poses.close();
    outFileSVO.close();


    return 0;

}
