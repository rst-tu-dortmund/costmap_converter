#include <opencv2/highgui.hpp>
#include <opencv2/cvv.hpp>
#include "costmap_converter/background_subtractor.h"


BackgroundSubtractor::BackgroundSubtractor()
{
  minSepBetweenSlowAndFastFilter_ = 80;
  minOccupancyProbability_ = 180;
}

void BackgroundSubtractor::apply(cv::Mat image, cv::Mat& fgMask, int shiftX, int shiftY, double alpha_slow, double alpha_fast, double beta)
{
  currentFrame_= image;

  // occupancyGrids are empty only once in the beginning -> initialize variables
  if(occupancyGrid_fast.empty() && occupancyGrid_slow.empty())
  {
    occupancyGrid_fast = currentFrame_;
    occupancyGrid_slow = currentFrame_;
    previousShiftX_ = shiftX;
    previousShiftY_ = shiftY;

    currentFrames_vec.push_back(currentFrame_);
    occupancyGrid_fast_vec.push_back(occupancyGrid_fast);
    occupancyGrid_slow_vec.push_back(occupancyGrid_slow);
    fgMask_vec.push_back(cv::Mat(currentFrame_.size(), CV_8UC1)); // Erste Maske erst im nächsten Schritt
    return;
  }

  if(currentFrames_vec.size() == 10)
  {
    WriteMatToYAML("currentFrames", currentFrames_vec);
    WriteMatToYAML("OccupancyGrid_fast", occupancyGrid_fast_vec);
    WriteMatToYAML("OccupancyGrid_slow", occupancyGrid_slow_vec);
    WriteMatToYAML("fgMask", fgMask_vec);
  }

  // Shift previous occupancyGrid to new location (match currentFrame)
  int shiftRelativeToPreviousPosX_ = shiftX - previousShiftX_;
  int shiftRelativeToPreviousPosY_ = shiftY - previousShiftY_;
  previousShiftX_ = shiftX;
  previousShiftY_ = shiftY;

  //if(shiftRelativeToPreviousPosX_ != 0 || shiftRelativeToPreviousPosY_ != 0)
  transformToCurrentFrame(shiftRelativeToPreviousPosX_, shiftRelativeToPreviousPosY_);

  //cvv::debugFilter(occupancyGrid_fast, currentFrame_, CVVISUAL_LOCATION);

  // Calculate normalized sum (mean) of nearest neighbors
  cv::Mat nearestNeighborMean_fast(occupancyGrid_fast.size(), CV_8UC1);
  cv::Mat nearestNeighborMean_slow(occupancyGrid_slow.size(), CV_8UC1);
//  cv::boxFilter(occupancyGrid_fast, nearestNeighborMean_fast, -1, cv::Size(3,3),cv::Point(-1,-1), true, cv::BORDER_REPLICATE);
//  cv::boxFilter(occupancyGrid_slow, nearestNeighborMean_slow, -1, cv::Size(3,3),cv::Point(-1,-1), true, cv::BORDER_REPLICATE);
  cv::GaussianBlur(occupancyGrid_fast, nearestNeighborMean_fast, cv::Size(3,3), 1, 1, cv::BORDER_REPLICATE);
  cv::GaussianBlur(occupancyGrid_fast, nearestNeighborMean_fast, cv::Size(3,3), 1, 1, cv::BORDER_REPLICATE);

  // compute time mean value for each pixel according to learningrate alpha
  occupancyGrid_fast = beta*(alpha_fast*currentFrame_ + (1.0-alpha_fast)*occupancyGrid_fast) + (1-beta)*nearestNeighborMean_fast;
  occupancyGrid_slow = beta*(alpha_slow*currentFrame_ + (1.0-alpha_slow)*occupancyGrid_slow) + (1-beta)*nearestNeighborMean_slow;

  // 1) occupancyGrid_fast > minOccupancyProbability
  cv::threshold(occupancyGrid_fast, occupancyGrid_fast, minOccupancyProbability_, 0/*unused*/, cv::THRESH_TOZERO);
  // 2) occupancyGrif_fast-occupancyGrid_slow > minSepBetweenSlowAndFastFilter
  cv::threshold(occupancyGrid_fast-occupancyGrid_slow, fgMask, minSepBetweenSlowAndFastFilter_, 255, cv::THRESH_BINARY);

  visualize("Current frame", currentFrame_);
  visualize("Foreground mask", fgMask);

  currentFrames_vec.push_back(currentFrame_);
  occupancyGrid_fast_vec.push_back(occupancyGrid_fast);
  occupancyGrid_slow_vec.push_back(occupancyGrid_slow);
  fgMask_vec.push_back(fgMask.clone()); // Merkwürdigerweise wird sonst ständig das zweite Bild angehängt
}

void BackgroundSubtractor::transformToCurrentFrame(int shiftX, int shiftY)
{
  // TODO: Statt mit Nullen mit erster Wahrnehmung (also currentFrame) auffüllen

  // Verschieben um shiftX nach links und shiftY nach unten (in costmap-Koordinaten!)
  // in cv::Mat Pixelkoordinaten wird um shift X nach links und um shiftY nach oben geschoben
  cv::Mat temp_fast, temp_slow;
  cv::Mat translationMat = (cv::Mat_<double>(2,3,CV_64F) << 1, 0, -shiftX, 0, 1, -shiftY);
  cv::warpAffine(occupancyGrid_fast, temp_fast, translationMat, occupancyGrid_fast.size()); // can't operate in-place
  cv::warpAffine(occupancyGrid_slow, temp_slow, translationMat, occupancyGrid_slow.size()); // can't operate in-place

  //cvv::debugFilter(occupancyGrid_fast, temp_fast);

  occupancyGrid_fast = temp_fast;
  occupancyGrid_slow = temp_slow;
}


void BackgroundSubtractor::visualize(std::string name, cv::Mat image)
{
  if(!image.empty())
  {
    cv::Mat im = image.clone();
    cv::flip(im, im, 0);
    cv::imshow(name, im);
    cv::waitKey(1);
  }
}

void BackgroundSubtractor::WriteMatToYAML(std::string filename, std::vector<cv::Mat> matVec)
{
  cv::FileStorage fs("./MasterThesis/Matlab/"+filename+".yml", cv::FileStorage::WRITE);
  fs << "frames" << matVec;
  fs.release();
}
