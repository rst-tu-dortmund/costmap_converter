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
 * Author: Christoph Rösmann, Otniel Rinaldo
 *********************************************************************/

#ifndef COSTMAP_TO_LINES_H_
#define COSTMAP_TO_LINES_H_

#include <costmap_converter/costmap_converter_interface.h>
#include <costmap_converter/costmap_to_polygons.h>


namespace costmap_converter
{
  
/**
 * @class CostmapToLinesDBSMCCH
 * @brief This class converts the costmap_2d into a set of lines (and points)
 * 
 * The conversion is performed in three stages:
 * 1. Clusters in the costmap are collected using the DBSCAN Algorithm (https://en.wikipedia.org/wiki/DBSCAN)
 *    Reference: Ester, Martin; Kriegel, Hans-Peter; Sander, Jörg; Xu, Xiaowei,
 *               A density-based algorithm for discovering clusters in large spatial databases with noise. 
 *               Proceedings of the Second International Conference on Knowledge Discovery and Data Mining. AAAI Press. 1996. pp. 226–231. ISBN 1-57735-004-9. 
 *    
 * 2. In the subsequent stage, the convex hull of each cluster is determined using the monotone chain algorithm aka Andrew's algorithm:
 *    C++ implementation example: http://www.algorithmist.com/index.php/Monotone_Chain_Convex_Hull ( GNU Free Documentation License 1.2 )
 *    Reference:  A. M. Andrew, "Another Efficient Algorithm for Convex Hulls in Two Dimensions", Info. Proc. Letters 9, 216-219 (1979).
 * 
 * 3. In the third step extract lines from each polygon (convex hull) if there exist at least a predefined number of support points.
 * 
 * The output is a container of polygons (but each polygon does only contain a single vertex (point) or two vertices (line)
 *  
 * @todo Implement a line extractor plugin using RANSAC
 */
  class CostmapToLinesDBSMCCH : public CostmapToPolygonsDBSMCCH
  {
  public:
   
    /**
     * @brief Constructor
     */
    CostmapToLinesDBSMCCH();
       
    /**
     * @brief Empty destructor
     */
    virtual ~CostmapToLinesDBSMCCH(){}
    
    /**
     * @brief Initialize the plugin
     */
    virtual void initialize();
    
    /**
     * @brief This method performs the actual work (conversion of the costmap to polygons)
     */
    virtual void compute();   
    
    /**
     * @brief Calculate the distance between a point and a line segment
     * @param point generic 2D point type defining the reference point
     * @param line_start generic 2D point type defining the start of the line
     * @param line_end generic 2D point type defining the end of the line
     * @tparam Point generic point type that should provide (writable) x and y member fiels.
     * @tparam LinePoint generic point type that should provide (writable) x and y member fiels.
     * @return (minimum) eucldian distance to the line segment
     */
    template <typename Point, typename LinePoint>
    static double computeDistanceToLineSegment(const Point& point, const LinePoint& line_start, const LinePoint& line_end);
    
  protected:
    
    /**
     * @brief Extract points and lines from a given cluster by checking all keypoint pairs of the convex hull for a minimum number of support points
     * @param cluster list of points in the cluster
     * @param polygon convex hull of the cluster \c cluster
     * @param[out] lines back_inserter object to a sequence of polygon msgs (new lines will be pushed back)
     */
    void extractPointsAndLines(const std::vector<KeyPoint>& cluster, const geometry_msgs::Polygon& polygon, std::back_insert_iterator< std::vector<geometry_msgs::Polygon> > lines);
    
    
  protected:
        
    double support_pts_min_dist_;
    int min_support_pts_;
 
  };  
  
  
  
  
  
  
  
  
  
template <typename Point, typename LinePoint> 
double CostmapToLinesDBSMCCH::computeDistanceToLineSegment(const Point& point, const LinePoint& line_start, const LinePoint& line_end)
{
    double dx = line_end.x - line_start.x;
    double dy = line_end.y - line_start.y;
    
    double length = std::sqrt(dx*dx + dy*dy);
    
    double u = 0;
    
    if (length>0)
     u = ((point.x - line_start.x) * dx + (point.y - line_start.y)*dy) / length;
  
    if (u <= 0)
      return std::sqrt(std::pow(point.x-line_start.x,2) + std::pow(point.y-line_start.y,2));
    
    if (u >= 1)
      return std::sqrt(std::pow(point.x-line_end.x,2) + std::pow(point.y-line_end.y,2));
    
    return std::sqrt(std::pow(point.x - (line_start.x+u*dx) ,2) + std::pow(point.y - (line_start.y+u*dy),2));
}

  
  
} //end namespace teb_local_planner

#endif /* COSTMAP_TO_LINES_H_ */
