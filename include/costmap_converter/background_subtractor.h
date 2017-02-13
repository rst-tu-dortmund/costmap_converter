#ifndef BACKGROUNDSUBTRACTOR_H
#define BACKGROUNDSUBTRACTOR_H

#include <cv_bridge/cv_bridge.h>

class BackgroundSubtractor
{
public:
  BackgroundSubtractor();

  void apply(cv::Mat image, cv::Mat& fgMask, int shiftX = 0, int shiftY = 0, double alpha_slow = 0.55,
             double alpha_fast = 0.95, double beta = 0.8);

  void visualize(std::string name, cv::Mat image);

  void WriteMatToYAML(std::string filename, std::vector<cv::Mat> matVec);

private:
  void transformToCurrentFrame(int shiftX, int shiftY);

  cv::Mat occupancyGrid_fast;
  cv::Mat occupancyGrid_slow;
  cv::Mat currentFrame_;

  int previousShiftX_;
  int previousShiftY_;

  double minOccupancyProbability_;
  double minSepBetweenSlowAndFastFilter_;
  double maxOccupancyNeighbors_;

  std::vector<cv::Mat> currentFrames_vec;
  std::vector<cv::Mat> occupancyGrid_fast_vec;
  std::vector<cv::Mat> occupancyGrid_slow_vec;
  std::vector<cv::Mat> fgMask_vec;
};

#endif // BACKGROUNDSUBTRACTOR_H
