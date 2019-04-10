//
// Created by socrob on 09-04-2019.
//

#ifndef ISAM2_ARUCOLANDMARKS_H
#define ISAM2_ARUCOLANDMARKS_H

#include "InitialParameters.h"
#include "ArUcoMarkers.h"
class ArUcoLandmark {

public:

    aruco::Marker aruco_marker_; //the main marker
    int landmark_index_[4]; //storing the landmark indexes of the four corners
    Eigen::VectorXd corner_points_;


public:

    //Default constructor
    ArUcoLandmark(aruco::Marker aruco_marker){
        aruco_marker_ = aruco_marker;
        corner_points_ =  (Eigen::VectorXd(8)
                <<  aruco_marker_[0].x, aruco_marker_[0].y,
                    aruco_marker_[1].x, aruco_marker_[1].y,
                    aruco_marker_[2].x, aruco_marker_[2].y,
                    aruco_marker_[3].x, aruco_marker_[3].y).finished();

    }

    int getLandmarkIndex(vector<ArUcoLandmark>& vector_aruco_landmarks); //method to check if the landmark has already been observed. if yes getting the index, if not assigning new value.
    void setLandmarkIndex(int index);






};


#endif //ISAM2_ARUCOLANDMARKS_H
