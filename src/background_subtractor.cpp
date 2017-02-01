#include <opencv2/highgui.hpp>
#include <opencv2/cvv.hpp>

#include "costmap_converter/background_subtractor.h"

BackgroundSubtractor::BackgroundSubtractor()
{
  minSepBetweenSlowAndFastFilter = 100;
  minOccupancyProbability_ = 200;
}

void BackgroundSubtractor::apply(cv::Mat image, cv::Mat& fgMask, int shiftX, int shiftY, double alpha_slow, double alpha_fast)
{
  currentFrame_= image;

  // occupancyGrids are empty only once in the beginning -> initialize variables
  if(occupancyGrid_fast.empty() && occupancyGrid_slow.empty())
  {
    occupancyGrid_fast = currentFrame_;
    occupancyGrid_slow = currentFrame_;
    previousShiftX_ = shiftX;
    previousShiftY_ = shiftY;
    return;
  }

  // Shift previous occupancyGrid to new location (match currentFrame)
  int shiftRelativeToPreviousPosX_ = shiftX - previousShiftX_;
  int shiftRelativeToPreviousPosY_ = shiftY - previousShiftY_;
  previousShiftX_ = shiftX;
  previousShiftY_ = shiftY;
  //if(shiftRelativeToPreviousPosX_ != 0 || shiftRelativeToPreviousPosY_ != 0)
  transformToCurrentFrame(shiftRelativeToPreviousPosX_, shiftRelativeToPreviousPosY_);

  // compute time mean value for each pixel according to learningrate alpha
  occupancyGrid_fast = alpha_fast * currentFrame_ + (1.0-alpha_fast) * occupancyGrid_fast;
  occupancyGrid_slow = alpha_slow * currentFrame_ + (1.0-alpha_slow) * occupancyGrid_slow;

  // determine moving pixels by thresholding
  cv::threshold(occupancyGrid_fast-occupancyGrid_slow, fgMask, minSepBetweenSlowAndFastFilter, 255, cv::THRESH_BINARY);

  visualize("Current frame", currentFrame_);
  visualize("Foreground mask", fgMask);
}

void BackgroundSubtractor::transformToCurrentFrame(int shiftX, int shiftY)
{
  // TODO: Statt mit Nullen mit erster Wahrnehmung (also currentFrame) auffüllen

  // Verschieben um shiftX nach links und shiftY nach unten (in costmap-Koordinaten!)
  // in cv::Mat Pixelkoordinaten wird um shift X nach links und um shiftY nach oben geschoben
  cv::Mat temp_fast, temp_slow;
  cv::Mat translationMat = (cv::Mat_<double>(2,3,CV_64F) << 1, 0, -shiftX, 0, 1, -shiftY);
  cv::warpAffine(occupancyGrid_fast, temp_fast, translationMat, occupancyGrid_fast.size());
  cv::warpAffine(occupancyGrid_slow, temp_slow, translationMat, occupancyGrid_slow.size());
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
