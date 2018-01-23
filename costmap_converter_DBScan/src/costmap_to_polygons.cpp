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

#include <costmap_converter_DBScan/costmap_to_polygons.h>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(costmap_converter::CostmapToPolygonsDBSMCCH, costmap_converter::BaseCostmapToPolygons)

namespace costmap_converter
{

CostmapToPolygonsDBSMCCH::CostmapToPolygonsDBSMCCH() : BaseCostmapToPolygons()
{
  costmap_ = NULL;
  dynamic_recfg_ = NULL;
}

CostmapToPolygonsDBSMCCH::~CostmapToPolygonsDBSMCCH()
{
  if (dynamic_recfg_ != NULL)
    delete dynamic_recfg_;
}

void CostmapToPolygonsDBSMCCH::initialize(ros::NodeHandle nh)
{
    costmap_ = NULL;

    max_distance_ = 0.4;
    nh.param("cluster_max_distance", max_distance_, max_distance_);

    min_pts_ = 2;
    nh.param("cluster_min_pts", min_pts_, min_pts_);

    max_pts_ = 30;
    nh.param("cluster_max_pts", max_pts_, max_pts_);

    min_keypoint_separation_ = 0.1;
    nh.param("convex_hull_min_pt_separation", min_keypoint_separation_, min_keypoint_separation_);

    // setup dynamic reconfigure
    dynamic_recfg_ = new dynamic_reconfigure::Server<CostmapToPolygonsDBSMCCHConfig>(nh);
    dynamic_reconfigure::Server<CostmapToPolygonsDBSMCCHConfig>::CallbackType cb = boost::bind(&CostmapToPolygonsDBSMCCH::reconfigureCB, this, _1, _2);
    dynamic_recfg_->setCallback(cb);
}


void CostmapToPolygonsDBSMCCH::compute()
{
    std::vector< std::vector<KeyPoint> > clusters;
    dbScan(occupied_cells_, clusters);

    // Create new polygon container
    PolygonContainerPtr polygons(new std::vector<geometry_msgs::Polygon>());


    // add convex hulls to polygon container
    for (std::size_t i = 1; i <clusters.size(); ++i) // skip first cluster, since it is just noise
    {
      polygons->push_back( geometry_msgs::Polygon() );
      convexHull2(clusters[i], polygons->back() );
    }

    // add our non-cluster points to the polygon container (as single points)
    if (!clusters.empty())
    {
      for (std::size_t i=0; i < clusters.front().size(); ++i)
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

#if ROS_VERSION_MINOR > 11 // 11 == Indigo
     if (!costmap_->getMutex())
#else
     if (!costmap_->getLock())
#endif
      {
        ROS_ERROR("Cannot update costmap since the mutex pointer is null");
        return;
      }

      int idx = 0;
#if ROS_VERSION_MINOR > 11 // 11 == Indigo
     costmap_2d::Costmap2D::mutex_t::scoped_lock lock(*costmap_->getMutex());
#else
     boost::shared_lock<boost::shared_mutex> lock(*costmap_->getLock());
#endif

      // get indices of obstacle cells
      for(std::size_t i = 0; i < costmap_->getSizeInCellsX(); i++)
      {
        for(std::size_t j = 0; j < costmap_->getSizeInCellsY(); j++)
        {
          int value = costmap_->getCost(i,j);
          if(value >= costmap_2d::LETHAL_OBSTACLE)
          {
            double x, y;
            costmap_->mapToWorld((unsigned int)i, (unsigned int)j, x, y);
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
  for(int i = 0; i< (int)occupied_cells.size(); i++)
  {
    if(!visited[i]) //keypoint has not been visited before
    {
      visited[i] = true; // mark as visited
      std::vector<int> neighbors;
      regionQuery(occupied_cells, i, neighbors); //Find neighbors around the keypoint
      if((int)neighbors.size() < min_pts_) //If not enough neighbors are found, mark as noise
      {
        clusters[0].push_back(occupied_cells[i]);
      }
      else
      {
        ++cluster_id; // increment current cluster_id
        clusters.push_back(std::vector<KeyPoint>());

        // Expand the cluster
        clusters[cluster_id].push_back(occupied_cells[i]);
        for(int j = 0; j<(int)neighbors.size(); j++)
        {
          if ((int)clusters[cluster_id].size() == max_pts_)
            break;

          if(!visited[neighbors[j]]) //keypoint has not been visited before
          {
            visited[neighbors[j]] = true;  // mark as visited
            std::vector<int> further_neighbors;
            regionQuery(occupied_cells, neighbors[j], further_neighbors); //Find more neighbors around the new keypoint
//             if(further_neighbors.size() < min_pts_)
//             {
//               clusters[0].push_back(occupied_cells[neighbors[j]]);
//             }
//             else
            if ((int)further_neighbors.size() >= min_pts_)
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

    for(int i = 0; i < (int)occupied_cells.size(); i++)
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
      {
        --k;
      }
      cluster[i].toPointMsg(polygon.points[k]);
      ++k;
    }

    // upper hull
    for (int i = n-2, t = k+1; i >= 0; --i)
    {
      while (k >= t && cross(polygon.points[k-2], polygon.points[k-1], cluster[i]) <= 0)
      {
        --k;
      }
      cluster[i].toPointMsg(polygon.points[k]);
      ++k;
    }


    polygon.points.resize(k); // original
    // TEST we skip the last point, since in our definition the polygon vertices do not contain the start/end vertex twice.
//     polygon.points.resize(k-1); // TODO remove last point from the algorithm above to reduce computational cost



    if (min_keypoint_separation_>0) // TODO maybe migrate to algorithm above to speed up computation
    {
      for (int i=0; i < (int) polygon.points.size() - 1; ++i)
      {
        if ( std::sqrt(std::pow((polygon.points[i].x - polygon.points[i+1].x),2) + std::pow((polygon.points[i].y - polygon.points[i+1].y),2)) < min_keypoint_separation_ )
          polygon.points.erase(polygon.points.begin()+i+1);
      }
    }
}



void CostmapToPolygonsDBSMCCH::convexHull2(std::vector<KeyPoint>& cluster, geometry_msgs::Polygon& polygon)
{
    std::vector<KeyPoint> P = cluster;
    std::vector<geometry_msgs::Point32>& points = polygon.points;

    // Sort P by x and y
    for (int i = 0; i < (int)P.size(); i++)
    {
        for (int j = i + 1; j < (int)P.size(); j++)
        {
            if (P[j].x < P[i].x || (P[j].x == P[i].x && P[j].y < P[i].y))
            {
                KeyPoint tmp = P[i];
                P[i] = P[j];
                P[j] = tmp;
            }
        }
    }

    // the output array H[] will be used as the stack
    int i;                 // array scan index

    // Get the indices of points with min x-coord and min|max y-coord
    int minmin = 0, minmax;
    double xmin = P[0].x;
    for (i = 1; i < (int)P.size(); i++)
        if (P[i].x != xmin) break;
    minmax = i - 1;
    if (minmax == (int)P.size() - 1)
    {   // degenerate case: all x-coords == xmin
        points.push_back(geometry_msgs::Point32());
        P[minmin].toPointMsg(points.back());
        if (P[minmax].y != P[minmin].y) // a  nontrivial segment
        {
            points.push_back(geometry_msgs::Point32());
            P[minmax].toPointMsg(points.back());
        }
        // add polygon endpoint
        points.push_back(geometry_msgs::Point32());
        P[minmin].toPointMsg(points.back());
        return;
    }

    // Get the indices of points with max x-coord and min|max y-coord
    int maxmin, maxmax = (int)P.size() - 1;
    double xmax = P.back().x;
    for (i = P.size() - 2; i >= 0; i--)
        if (P[i].x != xmax) break;
    maxmin = i+1;

    // Compute the lower hull on the stack H
    // push  minmin point onto stack
    points.push_back(geometry_msgs::Point32());
    P[minmin].toPointMsg(points.back());
    i = minmax;
    while (++i <= maxmin)
    {
        // the lower line joins P[minmin]  with P[maxmin]
        if (cross(P[minmin], P[maxmin], P[i]) >= 0 && i < maxmin)
            continue;           // ignore P[i] above or on the lower line

        while (points.size() > 1)         // there are at least 2 points on the stack
        {
            // test if  P[i] is left of the line at the stack top
            if (cross(points[points.size() - 2], points.back(), P[i]) > 0)
                break;         // P[i] is a new hull  vertex
            points.pop_back();         // pop top point off  stack
        }
        // push P[i] onto stack
        points.push_back(geometry_msgs::Point32());
        P[i].toPointMsg(points.back());
    }

    // Next, compute the upper hull on the stack H above  the bottom hull
    if (maxmax != maxmin)      // if  distinct xmax points
    {
         // push maxmax point onto stack
         points.push_back(geometry_msgs::Point32());
         P[maxmax].toPointMsg(points.back());
    }
    int bot = (int)points.size();                  // the bottom point of the upper hull stack
    i = maxmin;
    while (--i >= minmax)
    {
        // the upper line joins P[maxmax]  with P[minmax]
        if (cross( P[maxmax], P[minmax], P[i])  >= 0 && i > minmax)
            continue;           // ignore P[i] below or on the upper line

        while ((int)points.size() > bot)     // at least 2 points on the upper stack
        {
            // test if  P[i] is left of the line at the stack top
            if (cross(points[points.size() - 2], points.back(), P[i]) > 0)
                break;         // P[i] is a new hull  vertex
            points.pop_back();         // pop top point off stack
        }
        // push P[i] onto stack
        points.push_back(geometry_msgs::Point32());
        P[i].toPointMsg(points.back());
    }
    if (minmax != minmin)
    {
        // push  joining endpoint onto stack
        points.push_back(geometry_msgs::Point32());
        P[minmin].toPointMsg(points.back());
    }

    if (min_keypoint_separation_>0) // TODO maybe migrate to algorithm above to speed up computation
    {
      for (int i=0; i < (int) polygon.points.size() - 1; ++i)
      {
        if ( std::sqrt(std::pow((polygon.points[i].x - polygon.points[i+1].x),2) + std::pow((polygon.points[i].y - polygon.points[i+1].y),2)) < min_keypoint_separation_ )
          polygon.points.erase(polygon.points.begin()+i+1);
      }
    }
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

void CostmapToPolygonsDBSMCCH::reconfigureCB(CostmapToPolygonsDBSMCCHConfig& config, uint32_t level)
{
    max_distance_ = config.cluster_max_distance;
    min_pts_ = config.cluster_min_pts;
    max_pts_ = config.cluster_max_pts;
    min_keypoint_separation_ = config.cluster_min_pts;
}

}//end namespace costmap_converter
