//
// Created by socrob on 09-04-2019.
//

#include "ArUcoLandmark.h"

int ArUcoLandmark::getLandmarkIndex(vector<ArUcoLandmark>& vector_aruco_landmarks) {
    //find if the marker id is equal to any of the ids
    int i = 0;
    for(; i< vector_aruco_landmarks.size(); i++){

        if(vector_aruco_landmarks[i].aruco_marker_.id == this->aruco_marker_.id)
            return this->landmark_index_[0];

    }

    setLandmarkIndex(vector_aruco_landmarks[i].landmark_index_[0] + 4);

    //add to the vector_aruco_landmarks the current ArUcoLandmark object
    vector_aruco_landmarks.push_back(*this);
    return vector_aruco_landmarks[i].landmark_index_[0] + 4;  //return the last aruco_corner points landmark number (tl) + 4.

}


void ArUcoLandmark::setLandmarkIndex(int index) {
    this->landmark_index_[0] = index;
    this->landmark_index_[1] = index + 1;
    this->landmark_index_[2] = index + 2;
    this->landmark_index_[3] = index + 3;
}