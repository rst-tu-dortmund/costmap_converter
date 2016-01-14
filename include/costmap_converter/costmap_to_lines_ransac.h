/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016,
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
 * Author: Christoph Rösmann
 *********************************************************************/

#ifndef COSTMAP_TO_LINES_RANSAC_H_
#define COSTMAP_TO_LINES_RANSAC_H_

#include <costmap_converter/costmap_converter_interface.h>
#include <costmap_converter/costmap_to_polygons.h>
#include <boost/random.hpp>

#include <costmap_converter/CostmapToLinesDBSRANSACConfig.h>

namespace costmap_converter
{
  
/**
 * @class CostmapToLinesDBSRANSAC
 * @brief This class converts the costmap_2d into a set of lines (and points)
 * 
 * The conversion is performed in two stages:
 * 1. Clusters in the costmap are collected using the DBSCAN Algorithm (https://en.wikipedia.org/wiki/DBSCAN)
 *    Reference: Ester, Martin; Kriegel, Hans-Peter; Sander, Jörg; Xu, Xiaowei,
 *               A density-based algorithm for discovering clusters in large spatial databases with noise. 
 *               Proceedings of the Second International Conference on Knowledge Discovery and Data Mining. AAAI Press. 1996. pp. 226–231. ISBN 1-57735-004-9. 
 *    
 * 2. The RANSAC algorithm is used to find straight line segment models (https://en.wikipedia.org/wiki/RANSAC)
 *    RANSAC is called repeatedly to find multiple lines per cluster until the number of inliners is below a specific threshold.
 *    In that case the remaining outliers are used as points or keypoints of their convex hull are used as points (depending on a paramter).
 *    The latter one applies as a filter. The convex assumption is not strict in practice, since the remaining regions/cluster (line inliers are removed)
 *    should be dense and small. For details about the convex hull algorithm, refer to costmap_converter::CostmapToPolygonsDBSMCCH.
 *    Resulting lines of RANSAC are currently not refined by linear regression.
 * 
 * The output is a container of polygons (but each polygon does only contain a single vertex (point) or two vertices (line)
 */
  class CostmapToLinesDBSRANSAC : public CostmapToPolygonsDBSMCCH
  {
  public:
   
    /**
     * @brief Constructor
     */
    CostmapToLinesDBSRANSAC();
       
    /**
     * @brief Destructor
     */
    virtual ~CostmapToLinesDBSRANSAC();
    
    /**
     * @brief Initialize the plugin
     * @param nh Nodehandle that defines the namespace for parameters
     */
    virtual void initialize(ros::NodeHandle nh);
    
    /**
     * @brief This method performs the actual work (conversion of the costmap to polygons)
     */
    virtual void compute();   
    
    
    /**
     * @brief Calculate the distance between a point and a straight line (with infinite length)
     * @param point generic 2D point type defining the reference point
     * @param line_pt1 generic 2D point as part of the line
     * @param line_pt2 generic 2D point as part of the line
     * @tparam Point generic point type that should provide (writable) x and y member fiels.
     * @tparam LinePoint generic point type that should provide (writable) x and y member fiels.
     * @return (minimum) eucldian distance to the line segment
     */
    template <typename Point, typename LinePoint>
    static double computeDistanceToLine(const Point& point, const LinePoint& line_pt1, const LinePoint& line_pt2);
    
    /**
     * @brief Calculate the distance between a point and a straight line segment
     * @param point generic 2D point type defining the reference point
     * @param line_start generic 2D point type defining the start of the line
     * @param line_end generic 2D point type defining the end of the line
     * @param[out] is_inbetween write \c true, if the point is placed inbetween start and end [optional]
     * @tparam Point generic point type that should provide (writable) x and y member fiels.
     * @tparam LinePoint generic point type that should provide (writable) x and y member fiels.
     * @return (minimum) eucldian distance to the line segment
     */
    template <typename Point, typename LinePoint>
    static double computeDistanceToLineSegment(const Point& point, const LinePoint& line_start, const LinePoint& line_end, bool* is_inbetween=NULL);
    
     
    /**
     * @brief Check if the candidate point is an inlier.
     * @param point generic 2D point type defining the reference point
     * @param line_start generic 2D point type defining the start of the line
     * @param line_end generic 2D point type defining the end of the line
     * @param min_distance minimum distance allowed
     * @tparam Point generic point type that should provide (writable) x and y member fiels.
     * @tparam LinePoint generic point type that should provide (writable) x and y member fiels.
     * @return \c true if inlier, \c false otherwise
     */
    template <typename Point, typename LinePoint>
    static bool isInlier(const Point& point, const LinePoint& line_start, const LinePoint& line_end, double min_distance);
    
  protected:
    
    double ransac_inlier_distance_; //!< Maximum distance to the line segment for inliers
    int ransac_min_inliers_; //!< Minimum numer of inliers required to form a line
    int ransac_no_iterations_; //!< Number of ransac iterations
    int ransac_remainig_outliers_; //!< Repeat ransac until the number of outliers is as specified here
    bool ransac_convert_outlier_pts_; //!< If \c true, convert remaining outliers to single points.
    bool ransac_filter_remaining_outlier_pts_; //!< If \c true, filter the interior of remaining outliers and keep only keypoints of their convex hull
   
   
   boost::random::mt19937 rnd_generator_; //!< Random number generator for ransac with default seed
    
    
    /**
     * @brief Find a straight line segment in a point cloud with RANSAC (without linear regression).
     * @param data set of 2D data points
     * @param inlier_distance maximum distance that inliers must satisfy
     * @param no_iterations number of RANSAC iterations
     * @param min_inliers minimum number of inliers to return true
     * @param[out] best_model start and end of the best straight line segment 
     * @param[out] inliers inlier keypoints are written to this container [optional]
     * @param[out] outliers outlier keypoints are written to this container [optional]
     * @return \c false, if \c min_inliers is not satisfied, \c true otherwise
     */
    bool lineRansac(const std::vector<KeyPoint>& data, double inlier_distance, int no_iterations, int min_inliers,
                    std::pair<KeyPoint, KeyPoint>& best_model, std::vector<KeyPoint>* inliers = NULL,
                     std::vector<KeyPoint>* outliers = NULL);
    
    /**
     * @brief Perform a simple linear regression in order to fit a straight line 'y = slope*x + intercept'
     * @param data set of 2D data points
     * @param[out] slope The slope of the fitted line 
     * @param[out] intercept The intercept / offset of the line
     * @param[out] mean_x_out mean of the x-values of the data [optional]
     * @param[out] mean_y_out mean of the y-values of the data [optional]
     * @return \c true, if a valid line has been fitted, \c false otherwise.
     */
    bool linearRegression(const std::vector<KeyPoint>& data, double& slope, double& intercept,
                          double* mean_x_out = NULL, double* mean_y_out = NULL);
    
    
    
//     void adjustLineLength(const std::vector<KeyPoint>& data, const KeyPoint& linept1, const KeyPoint& linept2, 
//                           KeyPoint& line_start, KeyPoint& line_end);
    
    private:
    
    /**
     * @brief Callback for the dynamic_reconfigure node.
     * 
     * This callback allows to modify parameters dynamically at runtime without restarting the node
     * @param config Reference to the dynamic reconfigure config
     * @param level Dynamic reconfigure level
     */
    void reconfigureCB(CostmapToLinesDBSRANSACConfig& config, uint32_t level);
    
    dynamic_reconfigure::Server<CostmapToLinesDBSRANSACConfig>* dynamic_recfg_; //!< Dynamic reconfigure server to allow config modifications at runtime   
    
  };  
  
  
  
  
  
  
  
template <typename Point, typename LinePoint> 
double CostmapToLinesDBSRANSAC::computeDistanceToLine(const Point& point, const LinePoint& line_pt1, const LinePoint& line_pt2)
{
    double dx = line_pt2.x - line_pt1.x;
    double dy = line_pt2.y - line_pt1.y;
    
    double length = std::sqrt(dx*dx + dy*dy);
    
    if (length>0)
      return std::abs(dy * point.x - dx * point.y + line_pt2.x * line_pt1.y - line_pt2.y * line_pt1.x) / length;
   
    return std::sqrt(std::pow(point.x - line_pt1.x, 2) + std::pow(point.y - line_pt1.y, 2));  
}

  
template <typename Point, typename LinePoint> 
double CostmapToLinesDBSRANSAC::computeDistanceToLineSegment(const Point& point, const LinePoint& line_start, const LinePoint& line_end, bool* is_inbetween)
{
    double dx = line_end.x - line_start.x;
    double dy = line_end.y - line_start.y;
    
    double length = std::sqrt(dx*dx + dy*dy);
    
    double u = 0;
    
    if (length>0)
     u = ((point.x - line_start.x) * dx + (point.y - line_start.y)*dy) / length;
    
    if (is_inbetween)
      *is_inbetween = (u>=0 && u<=1);
  
    if (u <= 0)
      return std::sqrt(std::pow(point.x-line_start.x,2) + std::pow(point.y-line_start.y,2));
    
    if (u >= 1)
      return std::sqrt(std::pow(point.x-line_end.x,2) + std::pow(point.y-line_end.y,2));
    
    return std::sqrt(std::pow(point.x - (line_start.x+u*dx) ,2) + std::pow(point.y - (line_start.y+u*dy),2));
}  

template <typename Point, typename LinePoint>
bool CostmapToLinesDBSRANSAC::isInlier(const Point& point, const LinePoint& line_start, const LinePoint& line_end, double min_distance)
{
  bool is_inbetween = false;
  double distance = computeDistanceToLineSegment(point, line_start, line_end, &is_inbetween);
  if (!is_inbetween)
    return false;
  if (distance <= min_distance)
    return true;
  return false;
}

  
  
} //end namespace teb_local_planner

#endif /* COSTMAP_TO_LINES_RANSAC_H_ */
