// nach https://github.com/Smorodov/Multitarget-tracker/tree/master/Tracker, GPLv3

#pragma once
#include "defines.h"
#include <opencv/cv.h>

// http://www.morethantechnical.com/2011/06/17/simple-kalman-filter-for-tracking-using-opencv-2-2-w-code/
class TKalmanFilter
{
public:
  TKalmanFilter(Point_t p, track_t deltatime = 0.2, track_t Accel_noise_mag = 0.5);
  ~TKalmanFilter();
  void Prediction();
  Point_t Update(Point_t p, bool DataCorrect);
  cv::KalmanFilter* kalman;
  track_t dt;
  Point_t LastPosition;
  Point_t LastVelocity;
};
