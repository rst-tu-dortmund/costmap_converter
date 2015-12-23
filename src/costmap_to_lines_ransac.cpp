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

#include <costmap_converter/costmap_to_lines_ransac.h>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(costmap_converter::CostmapToLinesDBSRANSAC, costmap_converter::BaseCostmapToPolygons)

namespace costmap_converter
{

CostmapToLinesDBSRANSAC::CostmapToLinesDBSRANSAC() : CostmapToPolygonsDBSMCCH() 
{

}
  
void CostmapToLinesDBSRANSAC::initialize(ros::NodeHandle nh)
{ 
    // DB SCAN
    max_distance_ = 0.4; 
    nh.param("cluster_max_distance", max_distance_, max_distance_);
    
    min_pts_ = 2;
    nh.param("cluster_min_pts", min_pts_, min_pts_);

    // ransac
    ransac_inlier_distance_ = 0.2;
    nh.param("ransac_inlier_distance", ransac_inlier_distance_, ransac_inlier_distance_);
    
    ransac_min_inliers_ = 8;
    nh.param("ransac_min_inliers", ransac_min_inliers_, ransac_min_inliers_);
    
    ransac_no_iterations_ = 100;
    nh.param("ransac_no_iterations", ransac_no_iterations_, ransac_no_iterations_);
   
    ransac_remainig_outliers_ = 3;
    nh.param("ransac_remainig_outliers", ransac_remainig_outliers_, ransac_remainig_outliers_);
    
    ransac_convert_outlier_pts_ = true;
    nh.param("ransac_convert_outlier_pts", ransac_convert_outlier_pts_, ransac_convert_outlier_pts_);
    
    ransac_filter_remaining_outlier_pts_ = true;
    nh.param("ransac_filter_remaining_outlier_pts", ransac_filter_remaining_outlier_pts_, ransac_filter_remaining_outlier_pts_);
    
    // convex hull (only necessary if outlier filtering is enabled)
    min_keypoint_separation_ = 0.1;
    nh.param("convex_hull_min_pt_separation", min_keypoint_separation_, min_keypoint_separation_);
}  
  
void CostmapToLinesDBSRANSAC::compute()
{
    std::vector< std::vector<KeyPoint> > clusters;
    dbScan(occupied_cells_, clusters);
    
    // Create new polygon container
    PolygonContainerPtr polygons(new std::vector<geometry_msgs::Polygon>());  
    
    
    // fit lines using ransac for each cluster
    for (int i = 1; i <clusters.size(); ++i) // skip first cluster, since it is just noise
    { 
      
      while (clusters[i].size() > ransac_remainig_outliers_)
      {
//         std::vector<KeyPoint> inliers;
        std::vector<KeyPoint> outliers;
        std::pair<KeyPoint,KeyPoint> model;
        if (!lineRansac(clusters[i], ransac_inlier_distance_, ransac_no_iterations_, ransac_min_inliers_, model, NULL, &outliers ) )
          break;
        
        // add to polygon container
        geometry_msgs::Polygon line;
        line.points.resize(2);
        model.first.toPointMsg(line.points.front());
        model.second.toPointMsg(line.points.back());
        polygons->push_back(line);
        
        clusters[i] = outliers;
      }
      // create point polygons for remaining outliers
      if (ransac_convert_outlier_pts_)
      {
        if (ransac_filter_remaining_outlier_pts_)
        {
          // take edge points of a convex polygon
          // these points define a cluster and since all lines are extracted,
          // we remove points from the interior...
          geometry_msgs::Polygon polygon;
          convexHull(clusters[i], polygon);
          for (int j=0; j < (int)polygon.points.size(); ++j)
          {
            polygons->push_back(geometry_msgs::Polygon());
            convertPointToPolygon(polygon.points[j], polygons->back());
          }
        }
        else
        {
          for (int j = 0; j < (int)clusters[i].size(); ++j)
          {
            polygons->push_back(geometry_msgs::Polygon());
            convertPointToPolygon(clusters[i][j], polygons->back());           
          }
        }

      }
      
      
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


bool CostmapToLinesDBSRANSAC::lineRansac(const std::vector<KeyPoint>& data, double inlier_distance, int no_iterations,
                                         int min_inliers, std::pair<KeyPoint, KeyPoint>& best_model, 
                                         std::vector<KeyPoint>* inliers, std::vector<KeyPoint>* outliers)
{
  if (data.size() < 2 ||  data.size() < min_inliers)
  {
    return false;
  }
  
  boost::random::uniform_int_distribution<> distribution(0, data.size()-1);
  
  std::pair<int, int> best_model_idx;
  int best_no_inliers = -1;
  
  
  for (int i=0; i < no_iterations; ++i)
  {
    // choose random points to define a line candidate
    int start_idx = distribution(rnd_generator_);
    int end_idx = start_idx;
    while (end_idx == start_idx)
      end_idx = distribution(rnd_generator_);
    
    
    // compute inliers
    int no_inliers = 0;
    for (int j=0; j<(int)data.size(); ++j)
    {
      if ( computeDistanceToLineSegment(data[j], data[start_idx], data[end_idx] ) <= inlier_distance )
        ++no_inliers;
    }
    
    if (no_inliers > best_no_inliers)
    {
      best_no_inliers = no_inliers;
      best_model_idx.first = start_idx;
      best_model_idx.second = end_idx;
    }
  }
  
  best_model.first = data[best_model_idx.first];
  best_model.second = data[best_model_idx.second];
  
  if (best_no_inliers < min_inliers)
    return false;
  
  // Now repeat the calculation for the best model in order to obtain the inliers and outliers set
  // This might be faster if no_iterations is large, but TEST
  if (inliers || outliers)
  {
    if (inliers)
      inliers->clear();
    if (outliers)
      outliers->clear();
    
    int no_inliers = 0;
    for (int i=0; i<(int)data.size(); ++i)
    {
        if ( computeDistanceToLineSegment( data[i], best_model.first, best_model.second ) <= inlier_distance )
        {
          if (inliers)
            inliers->push_back( data[i] );
        }
        else
        {
          if (outliers)
            outliers->push_back( data[i] );
        }
    }
  }
  
  return true;
}

bool CostmapToLinesDBSRANSAC::linearRegression(const std::vector<KeyPoint>& data, double& slope, double& intercept, 
                                               double* mean_x_out, double* mean_y_out)
{
  if (data.size() < 2)
  {
    ROS_ERROR("CostmapToLinesDBSRANSAC: at least 2 data points required for linear regression");
    return false;
  }
  
  double mean_x = 0;
  double mean_y = 0;
  for (int i=0; i<(int)data.size(); ++i)
  {
    mean_x += data[i].x;
    mean_y += data[i].y;
  }
  mean_x /= double(data.size());
  mean_y /= double(data.size());
  
  if (mean_x_out)
    *mean_x_out = mean_x;
 
  if (mean_y_out)
    *mean_y_out = mean_y;
  
  double numerator = 0.0;
  double denominator = 0.0;

  for(int i=0; i<(int)data.size(); ++i)
  {
      double dx = data[i].x - mean_x;
      numerator += dx * (data[i].y - mean_y);
      denominator += dx*dx;
  }
  
  if (denominator == 0)
  {
    ROS_ERROR("CostmapToLinesDBSRANSAC: linear regression failed, denominator 0");
    return false;
  }
  else
    slope = numerator / denominator;
  
  intercept = mean_y - slope * mean_x;
  return true;
}


/*
void CostmapToLinesDBSRANSAC::adjustLineLength(const std::vector<KeyPoint>& data, const KeyPoint& linept1, const KeyPoint& linept2, 
                                               KeyPoint& line_start, KeyPoint& line_end)
{
  line_start = linept1; 
  line_end = linept2;

  // infinite line is defined by linept1 and linept2
  double dir_x = line_end.x - line_start.x;
  double dir_y = line_end.y - line_start.y;
  double norm = std::sqrt(dir_x*dir_x + dir_y*dir_y);
  dir_x /= norm;
  dir_y /= norm;
  
  // project data onto the line and check if the distance is increased in both directions
  for (int i=0; i < (int) data.size(); ++i)
  {
    double dx = data[i].x - line_start.x;
    double dy = data[i].y - line_start.y;
    // check scalar product at start
    double extension = dx*dir_x + dy*dir_y;
    if (extension<0)
    {
      line_start.x -=  dir_x*extension;
      line_start.y -=  dir_y*extension;
    }
    else
    {
      dx = data[i].x - line_end.x;
      dy = data[i].y - line_end.y;
      // check scalar product at end
      double extension = dx*dir_x + dy*dir_y;
      if (extension>0)
      {
        line_end.x +=  dir_x*extension;
        line_end.y +=  dir_y*extension;
      }
    }
  }
  
}*/


}//end namespace costmap_converter


