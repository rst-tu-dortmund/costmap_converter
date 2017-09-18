/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017
 *  TU Dortmund - Institute of Control Theory and Systems Engineering.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the institute nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Notes:
 * The following code makes use of the OpenCV library.
 * OpenCV is licensed under the terms of the 3-clause BSD License.
 *
 * Authors: Franz Albers, Christoph Rösmann
 *********************************************************************/

#ifndef COSTMAP_TO_DYNAMIC_OBSTACLES_H_
#define COSTMAP_TO_DYNAMIC_OBSTACLES_H_

// ROS
#include <costmap_converter/costmap_converter_interface.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

// OpenCV
#include <cv_bridge/cv_bridge.h>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/video/tracking.hpp>

// dynamic reconfigure
#include <costmap_converter/CostmapToDynamicObstaclesConfig.h>
#include <dynamic_reconfigure/server.h>

// Own includes
#include <costmap_converter/costmap_to_dynamic_obstacles/multitarget_tracker/Ctracker.h>
#include <costmap_converter/costmap_to_dynamic_obstacles/background_subtractor.h>
#include <costmap_converter/costmap_to_dynamic_obstacles/blob_detector.h>

// STL
#include <memory>

namespace costmap_converter {

/**
 * @class CostmapToDynamicObstacles
 * @brief This class converts the costmap_2d into dynamic obstacles.
 *
 * Static obstacles are treated as point obstacles.
 * @todo allow different plugins for both static and dynamic obstacles (arbitrary combinations)
 */
class CostmapToDynamicObstacles : public BaseCostmapToPolygons
{
public:
  /**
   * @brief Constructor
   */
  CostmapToDynamicObstacles();

  /**
   * @brief Destructor
   */
  virtual ~CostmapToDynamicObstacles();

  /**
   * @brief Initialize the plugin
   * @param nh Nodehandle that defines the namespace for parameters
   */
  virtual void initialize(ros::NodeHandle nh);

  /**
   * @brief This method performs the actual work (conversion of the costmap to
   * obstacles)
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
   * @remarks If compute() or startWorker() has not been called before, this
   * method returns an empty instance!
   * @return Shared instance of the current obstacle container
   */
  ObstacleArrayConstPtr getObstacles();

  /**
   * @brief Set name of robot's odometry topic
   *
   * @warning The method must be called before initialize()
   *
   * Some plugins might require odometry information
   * to compensate the robot's ego motion
   * @param odom_topic topic name
   */
  virtual void setOdomTopic(const std::string& odom_topic)
  {
    odom_topic_ = odom_topic;
  }

  /**
   * @brief OpenCV Visualization
   * @param name  Id/name of the opencv window
   * @param image Image to be visualized
   */
  void visualize(const std::string& name, const cv::Mat& image);

protected:
  /**
   * @brief Converts the estimated velocity of a tracked object to m/s and
   * returns it
   * @param idx Index of the tracked object in the tracker
   * @return Point_t, which represents velocity in [m/s] of object(idx) in x,y,z
   * coordinates
   */
  Point_t getEstimatedVelocityOfObject(unsigned int idx);

  /**
   * @brief Gets the last observed contour of a object and converts it from [px]
   * to [m]
   * @param[in]  idx Index of the tracked object in the tracker
   * @param[out] contour vector of Point_t, which represents the last observed contour in [m]
   *             in x,y,z coordinates
   */
  void getContour(unsigned int idx, std::vector<Point_t>& contour);

  /**
   * @brief Thread-safe update of the internal obstacle container (that is
   * shared using getObstacles() from outside this
   * class)
   * @param obstacles Updated obstacle container
   */
  void updateObstacleContainer(ObstacleArrayPtr obstacles);

private:
  boost::mutex mutex_;
  costmap_2d::Costmap2D* costmap_;
  cv::Mat costmap_mat_;
  ObstacleArrayPtr obstacles_;
  cv::Mat fg_mask_;
  std::unique_ptr<BackgroundSubtractor> bg_sub_;
  cv::Ptr<BlobDetector> blob_det_;
  std::vector<cv::KeyPoint> keypoints_;
  std::unique_ptr<CTracker> tracker_;
  ros::Subscriber odom_sub_;
  Point_t ego_vel_;

  std::string odom_topic_ = "/odom";
  bool publish_static_obstacles_ = true;

  dynamic_reconfigure::Server<CostmapToDynamicObstaclesConfig>*
      dynamic_recfg_; //!< Dynamic reconfigure server to allow config
                       //! modifications at runtime

  /**
   * @brief Callback for the odometry messages of the observing robot.
   *
   * Used to convert the velocity of obstacles to the /map frame.
   * @param msg The Pointer to the nav_msgs::Odometry of the observing robot
   */
  void odomCallback(const nav_msgs::Odometry::ConstPtr &msg);

  /**
   * @brief Callback for the dynamic_reconfigure node.
   *
   * This callback allows to modify parameters dynamically at runtime without
   * restarting the node
   * @param config Reference to the dynamic reconfigure config
   * @param level Dynamic reconfigure level
   */
  void reconfigureCB(CostmapToDynamicObstaclesConfig &config, uint32_t level);
};

} // end namespace costmap_converter

#endif /* COSTMAP_TO_DYNAMIC_OBSTACLES_H_ */
