// Based on https://github.com/Smorodov/Multitarget-tracker/tree/master/Tracker, GPLv3
// Refer to README.md in this directory.

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
  CTrack(const Point_t& p, const std::vector<cv::Point>& contour, track_t dt, size_t trackID)
      : track_id(trackID),
        skipped_frames(0),
        prediction(p),
        lastContour(contour),
        KF(p, dt)
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

  // Returns contour in [px], not in [m]!
  std::vector<cv::Point> getLastContour() const
  {
    return lastContour;
  }

  // Returns velocity in [px/s], not in [m/s]!
  Point_t getEstimatedVelocity() const
  {
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
  struct Params{
    track_t dt; // time for one step of the filter
    track_t dist_thresh;// distance threshold. Pairs of points with higher distance are not considered in the assignment problem
    int max_allowed_skipped_frames; // Maximum number of frames the track is maintained without seeing the object in the measurement data
    int max_trace_length; // Maximum trace length
  };

  CTracker(const Params& parameters);
  ~CTracker(void);

  std::vector<std::unique_ptr<CTrack>> tracks;
  void Update(const std::vector<Point_t>& detectedCentroid, const std::vector<std::vector<cv::Point> > &contour);

  void updateParameters(const Params &parameters);

private:
  // Aggregated parameters for the tracker object
  Params params;
  // ID for the upcoming CTrack object
  size_t NextTrackID;
};
