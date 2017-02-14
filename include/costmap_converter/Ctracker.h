// nach https://github.com/Smorodov/Multitarget-tracker/tree/master/Tracker, GPLv3

#pragma once
#include "Kalman.h"
#include "HungarianAlg.h"
#include "defines.h"
#include <iostream>
#include <vector>
#include <memory>
#include <array>

// --------------------------------------------------------------------------
class CTrack
{
public:
  CTrack(const Point_t& p, const std::vector<cv::Point>& contour, track_t dt, track_t Accel_noise_mag, size_t trackID)
      : track_id(trackID), skipped_frames(0), prediction(p), lastContour(contour), KF(p, dt, Accel_noise_mag)
  {
  }

  track_t CalcDist(const Point_t& p)
  {
    Point_t diff = prediction - p;
    return std::sqrt(diff.x * diff.x + diff.y * diff.y + diff.z * diff.z);
  }

  void Update(const Point_t& p, const std::vector<cv::Point>& contour, bool dataCorrect, size_t max_trace_length)
  {
    KF.Prediction();
    prediction = KF.Update(p, dataCorrect);

    if (dataCorrect)
    {
      lastContour = contour;
    }

    if (trace.size() > max_trace_length)
    {
      trace.erase(trace.begin(), trace.end() - max_trace_length);
    }

    trace.push_back(prediction);
  }

  std::vector<cv::Point> getLastContour() const
  {
    //TODO: Umrechnung in m
    // lastContour [px] * costmapResolution [m/px] = lastContour [m/s]
    return lastContour;
  }

  Point_t getEstimatedVelocity() const
  {
    // TODO: Umrechnung in m/s!
    // KF.LastVelocity [px/s] * costmapResolution [m/px] = vel [m/s]
    return KF.LastVelocity;
  }

  std::vector<Point_t> trace;
  size_t track_id;
  size_t skipped_frames;

private:
  Point_t prediction;
  std::vector<cv::Point> lastContour; // Contour liegt immer auf ganzen Pixeln -> Integer Punkt
  TKalmanFilter KF;
};

// --------------------------------------------------------------------------
class CTracker
{
public:
  CTracker(track_t dt_, track_t Accel_noise_mag_, track_t dist_thres_ = 60, size_t maximum_allowed_skipped_frames_ = 10,
           size_t max_trace_length_ = 10);
  ~CTracker(void);

  std::vector<std::unique_ptr<CTrack>> tracks;
  void Update(const std::vector<Point_t>& detectedCentroid, const std::vector<std::vector<cv::Point> > &contour);

private:
  // time for one step of the filter
  track_t dt;

  track_t Accel_noise_mag;
  // distance threshold. Pairs of points with higher distance are not considered in the assignment problem
  track_t dist_thres;
  // Maximum number of frames the track is maintained without seeing the object in the measurement data
  size_t maximum_allowed_skipped_frames;
  // Maximum trace length
  size_t max_trace_length;
  // ID for the upcoming CTrack object
  size_t NextTrackID;
};
