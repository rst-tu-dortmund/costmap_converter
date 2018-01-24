// Based on https://github.com/Smorodov/Multitarget-tracker/tree/master/Tracker, GPLv3
// Refer to README.md in this directory.

#include <costmap_converter_dynamic_obstacles/multitarget_tracker/Kalman.h>
#include "opencv2/opencv.hpp"
#include <iostream>
#include <vector>

//---------------------------------------------------------------------------
//---------------------------------------------------------------------------
TKalmanFilter::TKalmanFilter(Point_t pt, track_t deltatime)
{
  // time increment (lower values makes target more "massive")
  dt = deltatime;

  // 6 state variables [x y z xdot ydot zdot], 3 measurements [x y z]
  kalman = new cv::KalmanFilter(6, 3, 0);
  // Transition cv::Matrix
  kalman->transitionMatrix = (cv::Mat_<track_t>(6, 6) <<
                              1, 0, 0, dt,  0,  0,
                              0, 1, 0,  0, dt,  0,
                              0, 0, 1,  0,  0, dt,
                              0, 0, 0,  1,  0,  0,
                              0, 0, 0,  0,  1,  0,
                              0, 0, 0,  0,  0,  1);
  // init...
  LastPosition = pt;
  kalman->statePre.at<track_t>(0) = pt.x;
  kalman->statePre.at<track_t>(1) = pt.y;
  kalman->statePre.at<track_t>(2) = pt.z;

  kalman->statePre.at<track_t>(3) = 0;
  kalman->statePre.at<track_t>(4) = 0;
  kalman->statePre.at<track_t>(5) = 0;

  kalman->statePost.at<track_t>(0) = pt.x;
  kalman->statePost.at<track_t>(1) = pt.y;
  kalman->statePost.at<track_t>(2) = pt.z;

  // Nur x, y und z Koordinaten messbar!
  kalman->measurementMatrix = (cv::Mat_<track_t>(3, 6) <<
                               1, 0, 0, 0, 0, 0,
                               0, 1, 0, 0, 0, 0,
                               0, 0, 1, 0, 0, 0);

  float sigma = 0.5; // Assume standard deviation for acceleration, todo: dynamic reconfigure
  float c1 = pow(dt,4.0)/4.0;
  float c2 = pow(dt,2.0);
  float c3 = pow(dt,3.0)/2.0;
  kalman->processNoiseCov = sigma*sigma*(cv::Mat_<float>(6, 6) <<
                             c1,  0,  0, c3,  0,  0,
                              0, c1,  0,  0, c3,  0,
                              0,  0, c1,  0,  0, c3,
                             c3,  0,  0, c2,  0,  0,
                              0, c3,  0,  0, c2,  0,
                              0,  0, c3,  0,  0, c2);

  cv::setIdentity(kalman->measurementNoiseCov, cv::Scalar::all(5e-2));

  cv::setIdentity(kalman->errorCovPost, cv::Scalar::all(1000000));
}
//---------------------------------------------------------------------------
TKalmanFilter::~TKalmanFilter() { delete kalman; }

//---------------------------------------------------------------------------
void TKalmanFilter::Prediction()
{
  cv::Mat prediction = kalman->predict();
  LastPosition = Point_t(prediction.at<track_t>(0), prediction.at<track_t>(1), prediction.at<track_t>(2));
  LastVelocity = Point_t(prediction.at<track_t>(3), prediction.at<track_t>(4), prediction.at<track_t>(5));
}

//---------------------------------------------------------------------------
Point_t TKalmanFilter::Update(Point_t p, bool DataCorrect)
{
  cv::Mat measurement(3, 1, Mat_t(1));
  if (!DataCorrect)
  {
    measurement.at<track_t>(0) = LastPosition.x; // update using prediction
    measurement.at<track_t>(1) = LastPosition.y;
    measurement.at<track_t>(2) = LastPosition.z;
  }
  else
  {
    measurement.at<track_t>(0) = p.x; // update using measurements
    measurement.at<track_t>(1) = p.y;
    measurement.at<track_t>(2) = p.z;
  }
  // Correction
  cv::Mat estimated = kalman->correct(measurement);
  LastPosition.x = estimated.at<track_t>(0); // update using measurements
  LastPosition.y = estimated.at<track_t>(1);
  LastPosition.z = estimated.at<track_t>(2);
  LastVelocity.x = estimated.at<track_t>(3);
  LastVelocity.y = estimated.at<track_t>(4);
  LastVelocity.z = estimated.at<track_t>(5);
  return LastPosition;
}
//---------------------------------------------------------------------------
