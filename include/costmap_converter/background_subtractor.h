#ifndef BACKGROUNDSUBTRACTOR_H
#define BACKGROUNDSUBTRACTOR_H

#include <cv_bridge/cv_bridge.h>


class BackgroundSubtractor
{
public:
  BackgroundSubtractor();

  void apply(cv::Mat image, cv::Mat &fgMask, int shiftX = 0, int shiftY = 0, double alpha_slow = 0.55, double alpha_fast = 0.95);

  void visualize(std::string name, cv::Mat image);

private:
  void transformToCurrentFrame(int shiftX, int shiftY);

  cv::Mat occupancyGrid_fast;
  cv::Mat occupancyGrid_slow;
  cv::Mat currentFrame_;

  int previousShiftX_;
  int previousShiftY_;

  double minOccupancyProbability_;
  double minSepBetweenSlowAndFastFilter_;
};

#endif // BACKGROUNDSUBTRACTOR_H
