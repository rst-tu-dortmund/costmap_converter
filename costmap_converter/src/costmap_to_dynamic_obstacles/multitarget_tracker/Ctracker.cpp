// Based on https://github.com/Smorodov/Multitarget-tracker/tree/master/Tracker, GPLv3
// Refer to README.md in this directory.

#include <costmap_converter/costmap_to_dynamic_obstacles/multitarget_tracker/Ctracker.h>

// ---------------------------------------------------------------------------
// Tracker. Manage tracks. Create, remove, update.
// ---------------------------------------------------------------------------
CTracker::CTracker(const Params &parameters)
    : params(parameters),
      NextTrackID(0)
{
}
// ---------------------------------------------------------------------------
//
// ---------------------------------------------------------------------------
void CTracker::Update(const std::vector<Point_t>& detectedCentroid, const std::vector< std::vector<cv::Point> >& contours)
{
  // Each contour has a centroid
  assert(detectedCentroid.size() == contours.size());

  // -----------------------------------
  // If there is no tracks yet, then every cv::Point begins its own track.
  // -----------------------------------
  if (tracks.size() == 0)
  {
    // If no tracks yet
    for (size_t i = 0; i < detectedCentroid.size(); ++i)
    {
      tracks.push_back(
          std::unique_ptr<CTrack>(new CTrack(detectedCentroid[i], contours[i], params.dt, NextTrackID++)));
    }
  }

  size_t N = tracks.size();
  size_t M = detectedCentroid.size();

  assignments_t assignment;

  if (!tracks.empty())
  {
    // Distance matrix of N-th Track to the M-th detectedCentroid
    distMatrix_t Cost(N * M);

    // calculate distance between the blobs centroids
    for (size_t i = 0; i < tracks.size(); i++)
    {
      for (size_t j = 0; j < detectedCentroid.size(); j++)
      {
        Cost[i + j * N] = tracks[i]->CalcDist(detectedCentroid[j]);
      }
    }

    // -----------------------------------
    // Solving assignment problem (tracks and predictions of Kalman filter)
    // -----------------------------------
    AssignmentProblemSolver APS;
    APS.Solve(Cost, N, M, assignment, AssignmentProblemSolver::optimal);

    // -----------------------------------
    // clean assignment from pairs with large distance
    // -----------------------------------
    for (size_t i = 0; i < assignment.size(); i++)
    {
      if (assignment[i] != -1)
      {
        if (Cost[i + assignment[i] * N] > params.dist_thresh)
        {
          assignment[i] = -1;
          tracks[i]->skipped_frames = 1;
        }
      }
      else
      {
        // If track have no assigned detect, then increment skipped frames counter.
        tracks[i]->skipped_frames++;
      }
    }

    // -----------------------------------
    // If track didn't get detects long time, remove it.
    // -----------------------------------
    for (int i = 0; i < static_cast<int>(tracks.size()); i++)
    {
      if ((int)tracks[i]->skipped_frames > params.max_allowed_skipped_frames)
      {
        tracks.erase(tracks.begin() + i);
        assignment.erase(assignment.begin() + i);
        i--;
      }
    }
  }

  // -----------------------------------
  // Search for unassigned detects and start new tracks for them.
  // -----------------------------------
  for (size_t i = 0; i < detectedCentroid.size(); ++i)
  {
    if (find(assignment.begin(), assignment.end(), i) == assignment.end())
    {
      tracks.push_back(std::unique_ptr<CTrack>(new CTrack(detectedCentroid[i], contours[i], params.dt, NextTrackID++)));
    }
  }

  // Update Kalman Filters state

  for (size_t i = 0; i < assignment.size(); i++)
  {
    // If track updated less than one time, than filter state is not correct.

    if (assignment[i] != -1) // If we have assigned detect, then update using its coordinates,
    {
      tracks[i]->skipped_frames = 0;
      tracks[i]->Update(detectedCentroid[assignment[i]], contours[assignment[i]], true, params.max_trace_length);
    }
    else // if not continue using predictions
    {
      tracks[i]->Update(Point_t(), std::vector<cv::Point>(), false, params.max_trace_length);
    }
  }
}

void CTracker::updateParameters(const Params &parameters)
{
  params = parameters;
}
// ---------------------------------------------------------------------------
//
// ---------------------------------------------------------------------------
CTracker::~CTracker(void) {}
