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

#include <costmap_converter/costmap_to_lines_convex_hull.h>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(costmap_converter::CostmapToLinesDBSMCCH, costmap_converter::BaseCostmapToPolygons)

namespace costmap_converter
{

CostmapToLinesDBSMCCH::CostmapToLinesDBSMCCH() : CostmapToPolygonsDBSMCCH() 
{

}
  
void CostmapToLinesDBSMCCH::initialize(ros::NodeHandle nh)
{ 
    // DB SCAN
    max_distance_ = 0.4; 
    nh.param("cluster_max_distance", max_distance_, max_distance_);
    
    min_pts_ = 2;
    nh.param("cluster_min_pts", min_pts_, min_pts_);
    
    // convex hull
    min_keypoint_separation_ = 0.3;
    nh.param("convex_hull_min_pt_separation", min_keypoint_separation_, min_keypoint_separation_);
    
    // Line extraction
    support_pts_min_dist_ = 0.06;
    nh.param("support_pts_min_dist_", support_pts_min_dist_, support_pts_min_dist_);
    
    min_support_pts_ = 3;
    nh.param("min_support_pts_", min_support_pts_, min_support_pts_);
}  
  
void CostmapToLinesDBSMCCH::compute()
{
    std::vector< std::vector<KeyPoint> > clusters;
    dbScan(occupied_cells_, clusters);
    
    // Create new polygon container
    PolygonContainerPtr polygons(new std::vector<geometry_msgs::Polygon>());  
    
    
    // add convex hulls to polygon container
    for (int i = 1; i <clusters.size(); ++i) // skip first cluster, since it is just noise
    {
      geometry_msgs::Polygon polygon;
      convexHull(clusters[i], polygon );
      
      // now extract lines of the polygon (by searching for support points in the cluster)  
      // and add them to the polygon container
      extractPointsAndLines(clusters[i], polygon, std::back_inserter(*polygons));
    }
    
    // add our non-cluster points to the polygon container (as single points)
    if (!clusters.empty())
    {
      for (int i=0; i < clusters.front().size(); ++i)
      {
        polygons->push_back( geometry_msgs::Polygon() );
        convertPointToPolygon(clusters.front()[i], polygons->back());
      }
    }
        
    // replace shared polygon container
    updatePolygonContainer(polygons);
}



void CostmapToLinesDBSMCCH::extractPointsAndLines(const std::vector<KeyPoint>& cluster, const geometry_msgs::Polygon& polygon,
                                                  std::back_insert_iterator< std::vector<geometry_msgs::Polygon> > lines)
{
   if (polygon.points.empty())
     return;
  
   if (polygon.points.size() == 1)
   {
     lines = polygon; // our polygon is just a point, push back onto the output sequence
     return;
   }
  
   for (int i=0; i<(int)polygon.points.size() - 1; ++i) // this implemenation requires a closed polygon (start vertex = end vertex)
   {
        const geometry_msgs::Point32& vertex1 = polygon.points[i];
        const geometry_msgs::Point32& vertex2 = polygon.points[i+1];
       
        bool vertex1_is_part_of_a_line = false;
        
        //Search support points
        int support_points = 0;
        
        for(int c = 0; c < cluster.size(); ++c)
        {
          if((cluster[c].x == vertex1.x && cluster[c].y == vertex1.y) || (cluster[c].x == vertex2.x && cluster[c].y == vertex2.y))
          {  
            continue;
          }
          else
          {
            double dist_line_to_point = computeDistanceToLineSegment( cluster[c], vertex1, vertex2 );

            if(dist_line_to_point <= support_pts_min_dist_)
            {
              support_points++;
              if (support_points >= min_support_pts_)
              {
                // line found:
                geometry_msgs::Polygon line;
                line.points.push_back(vertex1);
                line.points.push_back(vertex2);
                lines = line; // back insertion
                vertex1_is_part_of_a_line = true;
                break;
              }
            }
          }
        }
              
        if (!vertex1_is_part_of_a_line) // for i=polygon.points.size() -> already included since vertex_0 == vertex_n
        {
            // add vertex 1 as single point
            geometry_msgs::Polygon point;
            point.points.push_back(vertex1);
            lines = point; // back insertion
        }
    }
 

}



}//end namespace costmap_converter


