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

#ifndef COSTMAP_TO_POLYGONS_CONCAVE_H_
#define COSTMAP_TO_POLYGONS_CONCAVE_H_

#include <costmap_converter/costmap_to_polygons.h>
#include <costmap_converter/misc.h>

// dynamic reconfigure
//#include <costmap_converter/CostmapToPolygonsDBSConcaveHullConfig.h>
//#include <dynamic_reconfigure/server.h>


namespace costmap_converter
{
  
/**
 * @class CostmapToPolygonsDBSConcaveHull
 * @brief This class converts the costmap_2d into a set of non-convex polygons (and points)
 *  
 * All single points that do not define a cluster (noise) are also treated as polygon (represented as a single vertex)
 * @todo change the class hierarchy to a clearer and more generic one
 */
class CostmapToPolygonsDBSConcaveHull : public CostmapToPolygonsDBSMCCH
{
  public:
    
    
    
    /**
     * @brief Constructor
     */
    CostmapToPolygonsDBSConcaveHull();
    
    /**
     * @brief Destructor
     */
    virtual ~CostmapToPolygonsDBSConcaveHull();
    
    /**
     * @brief Initialize the plugin
     * @param nh Nodehandle that defines the namespace for parameters
     */
    virtual void initialize(rclcpp::Node::SharedPtr nh);
   
    
    /**
     * @brief This method performs the actual work (conversion of the costmap to polygons)
     */
    virtual void compute();
    
  protected:
 
   
    /**
     * @brief Compute the concave hull for a single cluster
     * 
     * @remarks The last point is the same as the first one
     * @param cluster list of keypoints that should be converted into a polygon
     * @param depth Smaller depth: sharper surface, depth -> high value: convex hull
     * @param[out] polygon the resulting convex polygon
     */  
    void concaveHull(std::vector<KeyPoint>& cluster, double depth, geometry_msgs::msg::Polygon& polygon);
        
    void concaveHullClusterCut(std::vector<KeyPoint>& cluster, double depth, geometry_msgs::msg::Polygon& polygon);
    
    template <typename PointLine, typename PointCluster, typename PointHull>
    std::size_t findNearestInnerPoint(PointLine line_start, PointLine line_end, const std::vector<PointCluster>& cluster, const std::vector<PointHull>& hull, bool* found);
        
    
    template <typename Point1, typename Point2, typename Point3, typename Point4>
    bool checkLineIntersection(const Point1& line1_start, const Point2& line1_end, const Point3& line2_start, const Point4& line2_end);
    
    template <typename PointHull, typename Point1, typename Point2, typename Point3, typename Point4>
    bool checkLineIntersection(const std::vector<PointHull>& hull, const Point1& current_line_start, 
                               const Point2& current_line_end, const Point3& test_line_start, const Point4& test_line_end);
    
    double concave_hull_depth_;

  private:
       
    /**
     * @brief Callback for the dynamic_reconfigure node.
     * 
     * This callback allows to modify parameters dynamically at runtime without restarting the node
     * @param config Reference to the dynamic reconfigure config
     * @param level Dynamic reconfigure level
     */
//    void reconfigureCB(CostmapToPolygonsDBSConcaveHullConfig& config, uint32_t level);
    
//    dynamic_reconfigure::Server<CostmapToPolygonsDBSConcaveHullConfig>* dynamic_recfg_; //!< Dynamic reconfigure server to allow config modifications at runtime
   
   
}; 


template <typename PointLine, typename PointCluster, typename PointHull>
std::size_t CostmapToPolygonsDBSConcaveHull::findNearestInnerPoint(PointLine line_start, PointLine line_end, const std::vector<PointCluster>& cluster, const std::vector<PointHull>& hull, bool* found)
{
    std::size_t nearsest_idx = 0;
    double distance = 0;
    *found = false;

    for (std::size_t i = 0; i < cluster.size(); ++i)
    {
        // Skip points that are already in the hull
        if (std::find_if( hull.begin(), hull.end(), std::bind(isApprox2d<PointHull, PointCluster>, std::placeholders::_1, std::cref(cluster[i]), 1e-5) ) != hull.end() )
            continue;

        double dist = computeDistanceToLineSegment(cluster[i], line_start, line_end);
        bool skip = false;
        for (int j = 0; !skip && j < (int)hull.size() - 1; ++j) 
        {
            double dist_temp = computeDistanceToLineSegment(cluster[i], hull[j], hull[j + 1]);
            skip |= dist_temp < dist;
        }
        if (skip) 
            continue;

        if (!(*found) || dist < distance) 
        {
            nearsest_idx = i;
            distance = dist;
            *found = true;
        }
    }
    return nearsest_idx;
}


template <typename Point1, typename Point2, typename Point3, typename Point4>
bool CostmapToPolygonsDBSConcaveHull::checkLineIntersection(const Point1& line1_start, const Point2& line1_end, const Point3& line2_start, const Point4& line2_end)
{
    double dx1 = line1_end.x - line1_start.x;
    double dy1 = line1_end.y - line1_start.y;
    double dx2 = line2_end.x - line2_start.x;
    double dy2 = line2_end.y - line2_start.y;
    double s = (-dy1 * (line1_start.x - line2_start.x) + dx1 * (line1_start.y - line2_start.y)) / (-dx2 * dy1 + dx1 * dy2);
    double t = ( dx2 * (line1_start.y - line2_start.y) - dy2 * (line1_start.x - line2_start.x)) / (-dx2 * dy1 + dx1 * dy2);
    return (s > 0 && s < 1 && t > 0 && t < 1);
}


template <typename PointHull, typename Point1, typename Point2, typename Point3, typename Point4>
bool CostmapToPolygonsDBSConcaveHull::checkLineIntersection(const std::vector<PointHull>& hull, const Point1& current_line_start, 
                                                            const Point2& current_line_end, const Point3& test_line_start, const Point4& test_line_end)
{
    for (int i = 0; i < (int)hull.size() - 2; i++) 
    {
        if (isApprox2d(current_line_start, hull[i], 1e-5) && isApprox2d(current_line_end, hull[i+1], 1e-5))
        {
            continue;
        }

        if (checkLineIntersection(test_line_start, test_line_end, hull[i], hull[i+1])) 
        {
            return true;
        }
    }
    return false;
} 
  
  
} //end namespace teb_local_planner

#endif /* COSTMAP_TO_POLYGONS_CONCAVE_H_ */
