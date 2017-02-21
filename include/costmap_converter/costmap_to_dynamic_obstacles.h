#ifndef COSTMAP_TO_DYNAMIC_OBSTACLES_H_
#define COSTMAP_TO_DYNAMIC_OBSTACLES_H_

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <costmap_converter/costmap_converter_interface.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/features2d.hpp>
#include <opencv2/video/tracking.hpp>

#include "background_subtractor.h"
#include "blob_detector.h"
#include "Ctracker.h"

namespace costmap_converter
{

/**
 * @class CostmapToDynamicObstacles
 * @brief This class converts the costmap_2d into dynamic obstacles.
 */

class CostmapToDynamicObstacles : public BaseCostmapToPolygons
{
public:
  /*
   * @brief Constructor
   */
  CostmapToDynamicObstacles();

  /*
   * @brief Destructor
   */
  virtual ~CostmapToDynamicObstacles();

  /**
   * @brief Initialize the plugin
   * @param nh Nodehandle that defines the namespace for parameters
   */
  virtual void initialize(ros::NodeHandle nh);

  /**
   * @brief This method performs the actual work (conversion of the costmap to obstacles)
   */
  virtual void compute();

  /**
   * @brief Pass a pointer to the costmap to the plugin.
   * @sa updateCostmap2D
   * @param costmap Pointer to the costmap2d source
   */
  virtual void setCostmap2D(costmap_2d::Costmap2D* costmap);

  /**
   * @brief Get updated data from the previously set Costmap2D
   * @sa setCostmap2D
   */
  virtual void updateCostmap2D();

  /**
   * @brief Get a shared instance of the current obstacle container
   * @remarks If compute() or startWorker() has not been called before, this method returns an empty instance!
   * @return Shared instance of the current obstacle container
   */
  ObstacleContainerConstPtr getObstacles();

  // OpenCV Visualization
  void visualize(std::string name, cv::Mat image);

protected:
  /**
   * @brief Converts the estimated velocity of a tracked object to m/s and returns it
   * @param idx Index of the tracked object in the tracker
   * @return Point_t, which represents velocity in [m/s] of object(idx) in x,y,z coordinates
   */
  Point_t getEstimatedVelocityOfObject(unsigned int idx);

  /**
   * @brief Gets the last observed contour of a object and converts it from [px] to [m]
   * @param idx Index of the tracked object in the tracker
   * @return vector of Point_t, which represents the last observed contour in [m] in x,y,z coordinates
   */
  std::vector<Point_t> getContour(unsigned int idx);

  /**
   * @brief Thread-safe update of the internal obstacle container (that is shared using getObstacles() from outside this
   * class)
   * @param obstacles Updated obstacle container
   */
  void updateObstacleContainer(ObstacleContainerPtr obstacles);

  /*
   * @brief Convert a generic point type to a geometry_msgs::Polygon
   * @param point generic 2D point type
   * @param[out] polygon already instantiated polygon that will be filled with a single point
   * @tparam Point generic point type that should provide (writable) x and y member fiels.
   */
  template <typename Point> static void convertPointToPolygon(const Point& point, geometry_msgs::Polygon& polygon)
  {
    polygon.points.resize(1);
    polygon.points.front().x = point.x;
    polygon.points.front().y = point.y;
    polygon.points.front().z = 0;
  }

private:
  boost::mutex mutex_;
  costmap_2d::Costmap2D* costmap_;
  cv::Mat costmapMat_;
  ObstacleContainerPtr obstacles_;
  cv::Mat fgMask_;
  BackgroundSubtractor* bgSub_;
  cv::Ptr<BlobDetector> blobDet_;
  std::vector<cv::KeyPoint> keypoints_;
  CTracker* tracker_;
  ros::Subscriber odomSub_;
  Point_t egoVel_;

  void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
};

} // end namespace costmap_converter

#endif /* COSTMAP_TO_DYNAMIC_OBSTACLES_H_ */
