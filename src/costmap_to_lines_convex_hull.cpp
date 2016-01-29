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
#include <costmap_converter/misc.h>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(costmap_converter::CostmapToLinesDBSMCCH, costmap_converter::BaseCostmapToPolygons)

namespace costmap_converter
{

CostmapToLinesDBSMCCH::CostmapToLinesDBSMCCH() : CostmapToPolygonsDBSMCCH() 
{
    dynamic_recfg_ = NULL;
}
  
CostmapToLinesDBSMCCH::~CostmapToLinesDBSMCCH() 
{
  if (dynamic_recfg_ != NULL)
    delete dynamic_recfg_;
}
  
void CostmapToLinesDBSMCCH::initialize(ros::NodeHandle nh)
{ 
    // DB SCAN
    max_distance_ = 0.4; 
    nh.param("cluster_max_distance", max_distance_, max_distance_);
    
    min_pts_ = 2;
    nh.param("cluster_min_pts", min_pts_, min_pts_);
    
    max_pts_ = 30;
    nh.param("cluster_max_pts", max_pts_, max_pts_);
    
    // convex hull
    min_keypoint_separation_ = 0.1;
    nh.param("convex_hull_min_pt_separation", min_keypoint_separation_, min_keypoint_separation_);
    
    // Line extraction
    support_pts_max_dist_ = 0.3;
    nh.param("support_pts_max_dist", support_pts_max_dist_, support_pts_max_dist_);
    
    support_pts_max_dist_inbetween_ = 1.0;
    nh.param("support_pts_max_dist_inbetween", support_pts_max_dist_inbetween_, support_pts_max_dist_inbetween_);
    
    min_support_pts_ = 2;
    nh.param("min_support_pts", min_support_pts_, min_support_pts_);
    
    // setup dynamic reconfigure
    dynamic_recfg_ = new dynamic_reconfigure::Server<CostmapToLinesDBSMCCHConfig>(nh);
    dynamic_reconfigure::Server<CostmapToLinesDBSMCCHConfig>::CallbackType cb = boost::bind(&CostmapToLinesDBSMCCH::reconfigureCB, this, _1, _2);
    dynamic_recfg_->setCallback(cb);
    
    // deprecated
    if (nh.hasParam("support_pts_min_dist_") || nh.hasParam("support_pts_min_dist"))
      ROS_WARN("CostmapToLinesDBSMCCH: Parameter 'support_pts_min_dist' is deprecated and not included anymore.");
    if (nh.hasParam("min_support_pts_"))
      ROS_WARN("CostmapToLinesDBSMCCH: Parameter 'min_support_pts_' is not found. Remove the underscore.");
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
      convexHull2(clusters[i], polygon );
      
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

typedef CostmapToLinesDBSMCCH CL;
bool sort_keypoint_x(const std::size_t& i, const std::size_t& j, const std::vector<CL::KeyPoint>& cluster) 
{ return (cluster[i].x<cluster[j].x) || (cluster[i].x == cluster[j].x && cluster[i].y < cluster[j].y); }
bool sort_keypoint_y(const std::size_t& i, const std::size_t& j, const std::vector<CL::KeyPoint>& cluster) 
{ return (cluster[i].y<cluster[j].y) || (cluster[i].y == cluster[j].y && cluster[i].x < cluster[j].x); }

void CostmapToLinesDBSMCCH::extractPointsAndLines(std::vector<KeyPoint>& cluster, const geometry_msgs::Polygon& polygon,
                                                  std::back_insert_iterator< std::vector<geometry_msgs::Polygon> > lines)
{
   if (polygon.points.empty())
     return;
  
   if (polygon.points.size() < 2)
   {
     lines = polygon; // our polygon is just a point, push back onto the output sequence
     return;
   }
   int n = (int)polygon.points.size();
     
   for (int i=1; i<n; ++i) // this implemenation requires an explicitly closed polygon (start vertex = end vertex)
   {
        const geometry_msgs::Point32* vertex1 = &polygon.points[i-1];
        const geometry_msgs::Point32* vertex2 = &polygon.points[i];

        // check how many vertices belong to the line (sometimes the convex hull algorithm finds multiple vertices on a line,
        // in case of small coordinate deviations)
        double dx = vertex2->x - vertex1->x;
        double dy = vertex2->y - vertex1->y;
//         double norm = std::sqrt(dx*dx + dy*dy);
//         dx /= norm;
//         dy /= norm;
//         for (int j=i; j<(int)polygon.points.size() - 2; ++j)
//         {
//           const geometry_msgs::Point32* vertex_jp2 = &polygon.points[j+2];
//           double dx2 = vertex_jp2->x - vertex2->x;
//           double dy2 = vertex_jp2->y - vertex2->y;
//           double norm2 = std::sqrt(dx2*dx2 + dy2*dy2);
//           dx2 /= norm2;
//           dy2 /= norm2;
//           if (std::abs(dx*dx2 + dy*dy2) < 0.05) //~3 degree
//           {
//             vertex2 = &polygon.points[j+2];
//             i = j; // DO NOT USE "i" afterwards
//           }
//           else break;
//         }
       
        //Search support points
        std::vector<std::size_t> support_points; // store indices of cluster
        
        for (std::size_t k = 0; k < cluster.size(); ++k)
        {
            bool is_inbetween = false;
            double dist_line_to_point = computeDistanceToLineSegment( cluster[k], *vertex1, *vertex2, &is_inbetween );

            if(is_inbetween && dist_line_to_point <= support_pts_max_dist_)
            {
              support_points.push_back(k);
              continue;
            }
        }

        // now check if the inlier models a line by checking the minimum distance between all support points (avoid lines over gaps)
        // and by checking if the minium number of points is reached // deactivate by setting support_pts_max_dist_inbetween_==0
        bool is_line=true;
        if (support_pts_max_dist_inbetween_!=0)
        {
          if ((int)support_points.size() >= min_support_pts_ + 2) // +2 since start and goal are included
          {          
            // sort points
            if (std::abs(dx) >= std::abs(dy))
              std::sort(support_points.begin(), support_points.end(), boost::bind(sort_keypoint_x, _1, _2, boost::cref(cluster)));
            else 
              std::sort(support_points.begin(), support_points.end(), boost::bind(sort_keypoint_y, _1, _2, boost::cref(cluster)));
            
            // now calculate distances
            for (int k = 1; k < int(support_points.size()); ++k)
            {
              double dist_x = cluster[support_points[k]].x - cluster[support_points[k-1]].x;
              double dist_y = cluster[support_points[k]].y - cluster[support_points[k-1]].y;
              double dist = std::sqrt( dist_x*dist_x + dist_y*dist_y);
              if (dist > support_pts_max_dist_inbetween_)
              {
                is_line = false;
                break;
              }
            }
            
          }
          else
            is_line = false;
        }
        
        if (is_line)
        {
          // line found:
          geometry_msgs::Polygon line;
          line.points.push_back(*vertex1);
          line.points.push_back(*vertex2);
          lines = line; // back insertion
          
          // remove inlier from list
          // iterate in reverse order, otherwise indices are not valid after erasing elements
          std::vector<std::size_t>::reverse_iterator support_it = support_points.rbegin();
          for (; support_it != support_points.rend(); ++support_it)
          {
            cluster.erase(cluster.begin() + *support_it);
          }
        }
        else
        {
            // remove goal, since it will be added in the subsequent iteration
            //support_points.pop_back();
          // old:
//             // add vertex 1 as single point
//             geometry_msgs::Polygon point;
//             point.points.push_back(*vertex1);
//             lines = point; // back insertion

           // add complete inlier set as points 
//            for (int k = 0; k < int(support_points.size()); ++k)
//            {
//               geometry_msgs::Polygon polygon;
//               convertPointToPolygon(cluster[support_points[k]], polygon);
//               lines = polygon; // back insertion
//            }
           
          // remove inlier from list and add them as point obstacles
          // iterate in reverse order, otherwise indices are not valid after erasing elements
          std::vector<std::size_t>::reverse_iterator support_it = support_points.rbegin();
          for (; support_it != support_points.rend(); ++support_it)
          {
            geometry_msgs::Polygon polygon;
            convertPointToPolygon(cluster[*support_it], polygon);
            lines = polygon; // back insertion
            
            cluster.erase(cluster.begin() + *support_it);
          }
        }
    }
 
    // add all remaining cluster points, that do not belong to a line
    for (int i=0; i<(int)cluster.size();++i)
    {
        geometry_msgs::Polygon polygon;
        convertPointToPolygon(cluster[i], polygon);
        lines = polygon; // back insertion
    }

}

void CostmapToLinesDBSMCCH::reconfigureCB(CostmapToLinesDBSMCCHConfig& config, uint32_t level)
{
    max_distance_ = config.cluster_max_distance;
    min_pts_ = config.cluster_min_pts;
    max_pts_ = config.cluster_max_pts;
    min_keypoint_separation_ = config.cluster_min_pts;
    support_pts_max_dist_ = config.support_pts_max_dist;
    support_pts_max_dist_inbetween_ = config.support_pts_max_dist_inbetween;
    min_support_pts_ = config.min_support_pts;
}



}//end namespace costmap_converter


