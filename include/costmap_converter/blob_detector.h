#ifndef BLOBDETECTOR_H
#define BLOBDETECTOR_H

// Basically the OpenCV SimpleBlobDetector, extended with getContours()

#include <opencv2/features2d.hpp>

class BlobDetector : public cv::SimpleBlobDetector
{
public:
  BlobDetector(const cv::SimpleBlobDetector::Params& parameters = cv::SimpleBlobDetector::Params());

  static cv::Ptr<BlobDetector> create(const BlobDetector::Params& params);

  const std::vector<std::vector<cv::Point>> getContours();

  virtual void detect(const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints,
                      const cv::Mat& mask = cv::Mat()) const;

protected:
  struct Center
  {
    cv::Point2d location;
    double radius;
    double confidence;
  };

  virtual void findBlobs(const cv::Mat& image, const cv::Mat& binaryImage, std::vector<Center>& centers,
                         std::vector<std::vector<cv::Point>>& contours) const;

  Params params;
};

#endif // BLOBDETECTOR_H
