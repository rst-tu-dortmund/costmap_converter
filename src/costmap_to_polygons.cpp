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

#include <costmap_converter/costmap_to_polygons.h>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <pluginlib/class_list_macros.h>


PLUGINLIB_EXPORT_CLASS(costmap_converter::CostmapToPolygonsDBSMCCH, costmap_converter::BaseCostmapToPolygons)

namespace costmap_converter
{
    
CostmapToPolygonsDBSMCCH::CostmapToPolygonsDBSMCCH() : BaseCostmapToPolygons()
{
  costmap_ = NULL;
}

void CostmapToPolygonsDBSMCCH::initialize()
{
    costmap_ = NULL;
    
    ros::NodeHandle nh("~costmap_to_polygons");

    max_distance_ = 0.4; 
    nh.param("cluster_max_distance", max_distance_, max_distance_);
    
    min_pts_ = 2;
    nh.param("cluster_min_pts", min_pts_, min_pts_);
}


void CostmapToPolygonsDBSMCCH::compute()
{
    std::vector< std::vector<KeyPoint> > clusters;
    dbScan(occupied_cells_, clusters);
    
    // Create new polygon container
    PolygonContainerPtr polygons(new std::vector<geometry_msgs::Polygon>());
    
    
    // add convex hulls to polygon container
    for (int i = 1; i <clusters.size(); ++i) // skip first cluster, since it is just noise
    {
      polygons->push_back( geometry_msgs::Polygon() );
      convexHull(clusters[i], polygons->back() );
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

void CostmapToPolygonsDBSMCCH::setCostmap2D(costmap_2d::Costmap2D *costmap)
{
    if (!costmap)
      return;

    costmap_ = costmap;     
    
    updateCostmap2D();
}

void CostmapToPolygonsDBSMCCH::updateCostmap2D()
{
      occupied_cells_.clear();
      
      if (!costmap_->getMutex())
      {
        ROS_ERROR("Cannot update costmap since the mutex pointer is null");
        return;
      }
      
      int idx = 0;
      
      costmap_2d::Costmap2D::mutex_t::scoped_lock lock(*costmap_->getMutex());
            
      // get indices of obstacle cells
      for(int i = 0; i < costmap_->getSizeInCellsX(); i++)
      {
        for(int j = 0; j < costmap_->getSizeInCellsY(); j++)
        {
          int value = costmap_->getCost(i,j);
          if(value >= 254)
          {
            double x = costmap_->getOriginX() + double(idx / costmap_->getSizeInCellsY()) * costmap_->getResolution();
            double y = costmap_->getOriginY() + double(idx % costmap_->getSizeInCellsX()) * costmap_->getResolution();
            occupied_cells_.push_back( KeyPoint( x, y ) );
          }
          ++idx;
        }
      }
}


void CostmapToPolygonsDBSMCCH::dbScan(const std::vector<KeyPoint>& occupied_cells, std::vector< std::vector<KeyPoint> >& clusters)
{
  std::vector<bool> visited(occupied_cells.size(), false);

  clusters.clear();  
  
  //DB Scan Algorithm
  int cluster_id = 0; // current cluster_id
  clusters.push_back(std::vector<KeyPoint>());
  for(int i = 0; i<occupied_cells.size(); i++)
  {
    if(!visited[i]) //keypoint has not been visited before
    {
      visited[i] = true; // mark as visited
      std::vector<int> neighbors;
      regionQuery(occupied_cells, i, neighbors); //Find neighbors around the keypoint
      if(neighbors.size() < min_pts_) //If not enough neighbors are found, mark as noise
      {		
        clusters[0].push_back(occupied_cells[i]);
      }
      else
      {
        ++cluster_id; // increment current cluster_id
        clusters.push_back(std::vector<KeyPoint>());
        
        // Expand the cluster
        clusters[cluster_id].push_back(occupied_cells[i]);
        for(int j = 0; j<neighbors.size(); j++)
        {
          if(!visited[neighbors[j]]) //keypoint has not been visited before
          {
            visited[neighbors[j]] = true;  // mark as visited
            std::vector<int> further_neighbors;
            regionQuery(occupied_cells, neighbors[j], further_neighbors); //Find more neighbors around the new keypoint
            if(further_neighbors.size() < min_pts_)
            {	  
              clusters[0].push_back(occupied_cells[neighbors[j]]);
            }
            else
            {
              // neighbors found
              neighbors.insert(neighbors.end(), further_neighbors.begin(), further_neighbors.end());  //Add these newfound P' neighbour to P neighbour vector "nb_indeces"
              clusters[cluster_id].push_back(occupied_cells[neighbors[j]]);
            }
          }
        }
      }	      
    }
  } 
}
  
void CostmapToPolygonsDBSMCCH::regionQuery(const std::vector<KeyPoint>& occupied_cells, int curr_index, std::vector<int>& neighbors)
{
    neighbors.clear();
    double curr_index_x = occupied_cells[curr_index].x;
    double curr_index_y = occupied_cells[curr_index].y;

    for(int i = 0; i < occupied_cells.size(); i++)
    {
      double neighbor_x = occupied_cells[i].x;
      double neighbor_y = occupied_cells[i].y;
      double dist = sqrt(pow((curr_index_x - neighbor_x),2)+pow((curr_index_y - neighbor_y),2));	//euclidean distance between two points // TODO map resolution
      if(dist <= max_distance_ && dist != 0.0f)
        neighbors.push_back(i);
    }
}

bool isXCoordinateSmaller(const CostmapToPolygonsDBSMCCH::KeyPoint& p1, const CostmapToPolygonsDBSMCCH::KeyPoint& p2)
{
  return p1.x < p2.x || (p1.x == p2.x && p1.y < p2.y);
}

void CostmapToPolygonsDBSMCCH::convexHull(std::vector<KeyPoint>& cluster, geometry_msgs::Polygon& polygon)
{
    //Monotone Chain ConvexHull Algorithm source from http://www.algorithmist.com/index.php/Monotone_Chain_Convex_Hull
  
    int k = 0;
    int n = cluster.size();
    
    // sort points according to x coordinate (TODO. is it already sorted due to the map representation?)
    std::sort(cluster.begin(), cluster.end(), isXCoordinateSmaller);
    
    polygon.points.resize(2*n);
      
    // lower hull
    for (int i = 0; i < n; ++i)
    {
      while (k >= 2 && cross(polygon.points[k-2], polygon.points[k-1], cluster[i]) <= 0) 
        k--;
      cluster[i].toPointMsg(polygon.points[k]);
      k++;
    }
      
    // upper hull  
    for (int i = n-2, t = k+1; i >= 0; i--) 
    {
      while (k >= t && cross(polygon.points[k-2], polygon.points[k-1], cluster[i]) <= 0)
        k--;
      cluster[i].toPointMsg(polygon.points[k]);
      k++;
    }
    polygon.points.resize(k);
      
//     for (int i = 0; i < (polygon.points.size() - 1); i++)
//     {
//       float dist = sqrt(pow((polygon.points[i].x - polygon.points[i+1].x),2)+pow((polygon.points[i].y - polygon.points[i+1].y),2));
//       if(dist < 3.0)
//       {
//         polygon.points.erase(polygon.points.begin()+i+1);
//       } 
//     }
}


void CostmapToPolygonsDBSMCCH::updatePolygonContainer(PolygonContainerPtr polygons)
{
  boost::mutex::scoped_lock lock(mutex_);
  polygons_ = polygons;
}


PolygonContainerConstPtr CostmapToPolygonsDBSMCCH::getPolygons()
{
  boost::mutex::scoped_lock lock(mutex_);
  PolygonContainerConstPtr polygons = polygons_;
  return polygons;
}


}//end namespace costmap_converter


