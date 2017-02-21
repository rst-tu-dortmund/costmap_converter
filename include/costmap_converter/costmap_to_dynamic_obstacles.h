#ifndef COSTMAP_TO_DYNAMIC_OBSTACLES_H_
#define COSTMAP_TO_DYNAMIC_OBSTACLES_H_

// ROS
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <costmap_converter/costmap_converter_interface.h>

// OpenCV
#include <cv_bridge/cv_bridge.h>
#include <opencv2/features2d.hpp>
#include <opencv2/video/tracking.hpp>

// dynamic reconfigure
#include <dynamic_reconfigure/server.h>
#include <costmap_converter/CostmapToDynamicObstaclesConfig.h>

// Own includes
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

  dynamic_reconfigure::Server<CostmapToDynamicObstaclesConfig>* dynamic_recfg_; //!< Dynamic reconfigure server to allow config modifications at runtime

  /**
   * @brief Callback for the odometry messages of the observing robot.
   * Used to convert the velocity of obstacles to the /map frame.
   * @param msg The Pointer to the nav_msgs::Odometry of the observing robot
   */
  void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);

  /**
   * @brief Callback for the dynamic_reconfigure node.
   *
   * This callback allows to modify parameters dynamically at runtime without restarting the node
   * @param config Reference to the dynamic reconfigure config
   * @param level Dynamic reconfigure level
   */
  void reconfigureCB(CostmapToDynamicObstaclesConfig& config, uint32_t level);
};

} // end namespace costmap_converter

#endif /* COSTMAP_TO_DYNAMIC_OBSTACLES_H_ */
