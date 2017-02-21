#ifndef BACKGROUNDSUBTRACTOR_H
#define BACKGROUNDSUBTRACTOR_H

#include <cv_bridge/cv_bridge.h>

class BackgroundSubtractor
{
public:
  struct Params{
    double alpha_slow = 0.55;
    double alpha_fast = 0.95;
    double beta = 0.8;
    double minOccupancyProbability = 180;
    double minSepBetweenFastAndSlowFilter = 80;
    double maxOccupancyNeighbors = 80;
    int morph_size = 1;
  };

  BackgroundSubtractor(const Params& parameters);

  void apply(cv::Mat image, cv::Mat& fgMask, int shiftX = 0, int shiftY = 0);

  void visualize(std::string name, cv::Mat image);

  void WriteMatToYAML(std::string filename, std::vector<cv::Mat> matVec);

  void updateParameters(const Params& parameters);

private:
  void transformToCurrentFrame(int shiftX, int shiftY);

  cv::Mat occupancyGrid_fast;
  cv::Mat occupancyGrid_slow;
  cv::Mat currentFrame_;

  int previousShiftX_;
  int previousShiftY_;

  std::vector<cv::Mat> currentFrames_vec;
  std::vector<cv::Mat> occupancyGrid_fast_vec;
  std::vector<cv::Mat> occupancyGrid_slow_vec;
  std::vector<cv::Mat> fgMask_vec;

  Params params;
};

#endif // BACKGROUNDSUBTRACTOR_H
