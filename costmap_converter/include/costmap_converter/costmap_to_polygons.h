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

#ifndef COSTMAP_TO_POLYGONS_H_
#define COSTMAP_TO_POLYGONS_H_

#include <rclcpp/rclcpp.hpp>
#include <costmap_converter/costmap_converter_interface.h>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/polygon.hpp>
#include <vector>
#include <algorithm>
#include <Eigen/Core>
#include <Eigen/StdVector>

// dynamic reconfigure
//#include <costmap_converter/CostmapToPolygonsDBSMCCHConfig.h>
//#include <dynamic_reconfigure/server.h>


namespace costmap_converter
{
  
/**
 * @class CostmapToPolygonsDBSMCCH
 * @brief This class converts the costmap_2d into a set of convex polygons (and points)
 * 
 * The conversion is performed in two stages:
 * 1. Clusters in the costmap are collected using the DBSCAN Algorithm (https://en.wikipedia.org/wiki/DBSCAN)
 *    Reference: Ester, Martin; Kriegel, Hans-Peter; Sander, Jörg; Xu, Xiaowei,
 *               A density-based algorithm for discovering clusters in large spatial databases with noise. 
 *               Proceedings of the Second International Conference on Knowledge Discovery and Data Mining. AAAI Press. 1996. pp. 226–231. ISBN 1-57735-004-9. 
 *    
 * 2. In the subsequent stage, clusters are converted into convex polgons using the monotone chain algorithm aka Andrew's algorithm:
 *    C++ implementation example: http://www.algorithmist.com/index.php/Monotone_Chain_Convex_Hull ( GNU Free Documentation License 1.2 )
 *    Reference:  A. M. Andrew, "Another Efficient Algorithm for Convex Hulls in Two Dimensions", Info. Proc. Letters 9, 216-219 (1979).
 *  
 * All single points that do not define a cluster (noise) are also treated as polygon (represented as a single vertex)
 */
class CostmapToPolygonsDBSMCCH : public BaseCostmapToPolygons
{
  public:
    
    /**
     * @struct KeyPoint
     * @brief Defines a keypoint in metric coordinates of the map 
     */
    struct KeyPoint
    {
      //! Default constructor
      KeyPoint() {}
      //! Constructor with point initialization
      KeyPoint(double x_, double y_) : x(x_), y(y_) {}
      double x; //!< x coordinate [m]
      double y; //!< y coordinate [m]
      
      //! Convert keypoint to geometry_msgs::msg::Point message type
      void toPointMsg(geometry_msgs::msg::Point& point) const {point.x=x; point.y=y; point.z=0;}
      //! Convert keypoint to geometry_msgs::msg::Point32 message type
      void toPointMsg(geometry_msgs::msg::Point32& point) const {point.x=x; point.y=y; point.z=0;}
    };
    
    
    /**
     * @brief Constructor
     */
    CostmapToPolygonsDBSMCCH();
    
    /**
     * @brief Destructor
     */
    virtual ~CostmapToPolygonsDBSMCCH();
    
    /**
     * @brief Initialize the plugin
     * @param nh Nodehandle that defines the namespace for parameters
     */
    virtual void initialize(rclcpp::Node::SharedPtr nh) override;
    
    /**
     * @brief This method performs the actual work (conversion of the costmap to polygons)
     */
    virtual void compute();
        
    /**
     * @brief Pass a pointer to the costap to the plugin.
     * @sa updateCostmap2D
     * @param costmap Pointer to the costmap2d source
     */
    virtual void setCostmap2D(nav2_costmap_2d::Costmap2D* costmap);
    
    /**
     * @brief Get updated data from the previously set Costmap2D
     * @sa setCostmap2D
     */
    virtual void updateCostmap2D();
    
    
    /**
     * @brief Convert a generi point type to a geometry_msgs::msg::Polygon
     * @param point generic 2D point type
     * @param[out] polygon already instantiated polygon that will be filled with a single point
     * @tparam Point generic point type that should provide (writable) x and y member fiels.
     */
    template< typename Point>
    static void convertPointToPolygon(const Point& point, geometry_msgs::msg::Polygon& polygon)
    {
      polygon.points.resize(1);
      polygon.points.front().x = point.x;
      polygon.points.front().y = point.y;
      polygon.points.front().z = 0;
    }
    
    /**
     * @brief Get a shared instance of the current polygon container
     * @remarks If compute() or startWorker() has not been called before, this method returns an empty instance!
     * @return Shared instance of the current polygon container
     */
    PolygonContainerConstPtr getPolygons();

    
    
  protected:
    
    /**
     * @brief DBSCAN algorithm for clustering 
     * 
     * Clusters in the costmap are collected using the DBSCAN Algorithm (https://en.wikipedia.org/wiki/DBSCAN)
     *    Reference: Ester, Martin; Kriegel, Hans-Peter; Sander, Jörg; Xu, Xiaowei,
     *               A density-based algorithm for discovering clusters in large spatial databases with noise. 
     *               Proceedings of the Second International Conference on Knowledge Discovery and Data Mining. AAAI Press. 1996. pp. 226–231. ISBN 1-57735-004-9. 
     * 
     * @param occupied_cells a list of occupied cells of the costmap in metric coordinates
     * @param[out] clusters clusters will added to this output-argument (a sequence of keypoints for each cluster)
     *                      the first cluster (clusters[0]) will contain all noise points (that does not fulfil the min_pts condition 
     */
    void dbScan(const std::vector<KeyPoint>& occupied_cells, std::vector< std::vector<KeyPoint> >& clusters);
    
    /**
     * @brief Helper function for dbScan to search for neighboring points 
     * 
     * @param occupied_cells a list of occupied cells of the costmap in metric coordinates
     * @param curr_index index to the current item in \c occupied_cells
     * @param[out] neighbor_indices list of neighbors (indices of \c occupied cells)
     */
    void regionQuery(const std::vector<KeyPoint>& occupied_cells, int curr_index, std::vector<int>& neighbor_indices);

    
    /**
     * @brief Compute the convex hull for a single cluster (monotone chain algorithm)
     * 
     * Clusters are converted into convex polgons using the monotone chain algorithm aka Andrew's algorithm:
     *    C++ implementation example: http://www.algorithmist.com/index.php/Monotone_Chain_Convex_Hull ( GNU Free Documentation License 1.2 )
     *    Reference:  A. M. Andrew, "Another Efficient Algorithm for Convex Hulls in Two Dimensions", Info. Proc. Letters 9, 216-219 (1979).
     * @remarks The algorithm seems to cut edges, thus we give a try to convexHull2().
     * @todo Evaluate and figure out whether convexHull() or convexHull2() is better suited (performance, quality, ...)
     * @remarks The last point is the same as the first one
     * @param cluster list of keypoints that should be converted into a polygon
     * @param[out] polygon the resulting convex polygon
     */  
    void convexHull(std::vector<KeyPoint>& cluster, geometry_msgs::msg::Polygon& polygon);
    
    /**
     * @brief Compute the convex hull for a single cluster
     * 
     * Clusters are converted into convex polgons using an algorithm provided here:
     *  https://bitbucket.org/vostreltsov/concave-hull/overview
     * The convex hull algorithm is part of the concave hull algorithm.
     * The license is WTFPL 2.0 and permits any usage.
     * 
     * @remarks The last point is the same as the first one
     * @param cluster list of keypoints that should be converted into a polygon
     * @param[out] polygon the resulting convex polygon
     * @todo Evaluate and figure out whether convexHull() or convexHull2() is better suited (performance, quality, ...)
     */  
    void convexHull2(std::vector<KeyPoint>& cluster, geometry_msgs::msg::Polygon& polygon);
    
   /**
    * @brief 2D Cross product of two vectors defined by two points and a common origin
    * @param O Origin
    * @param A First point
    * @param B Second point
    * @tparam P1 2D Point type with x and y member fields
    * @tparam P2 2D Point type with x and y member fields
    * @tparam P3 2D Point type with x and y member fields
    */    
    template <typename P1, typename P2, typename P3>
    long double cross(const P1& O, const P2& A, const P3& B)
    { 
        return (long double)(A.x - O.x) * (long double)(B.y - O.y) - (long double)(A.y - O.y) * (long double)(B.x - O.x);
    }
    
      
   /**
    * @brief Thread-safe update of the internal polygon container (that is shared using getPolygons() from outside this class)
    * @param polygons Updated polygon container
    */
   void updatePolygonContainer(PolygonContainerPtr polygons);   
          
   
   std::vector<KeyPoint> occupied_cells_; //!< List of occupied cells in the current map (updated by updateCostmap2D())

   // DBSCAN parameters
   double max_distance_; //!< Parameter for DB_Scan, maximum distance to neighbors [m]
   int min_pts_; //!< Parameter for DB_Scan: minimum number of points that define a cluster
   int max_pts_; //!< Parameter for DB_Scan: maximum number of points that define a cluster (to avoid large L- and U-shapes)
   
   // convex hull parameters
   double min_keypoint_separation_; //!< Clear keypoints of the convex polygon that are close to each other [distance in meters] (0: keep all)
   
  private:
       
    /**
     * @brief Callback for the dynamic_reconfigure node.
     * 
     * This callback allows to modify parameters dynamically at runtime without restarting the node
     * @param config Reference to the dynamic reconfigure config
     * @param level Dynamic reconfigure level
     */
    //void reconfigureCB(CostmapToPolygonsDBSMCCHConfig& config, uint32_t level);
    
    
    PolygonContainerPtr polygons_; //!< Current shared container of polygons
    std::mutex mutex_; //!< Mutex that keeps track about the ownership of the shared polygon instance
    
    //dynamic_reconfigure::Server<CostmapToPolygonsDBSMCCHConfig>* dynamic_recfg_; //!< Dynamic reconfigure server to allow config modifications at runtime
   
    nav2_costmap_2d::Costmap2D *costmap_; //!< Pointer to the costmap2d
   
}; 

  
} //end namespace teb_local_planner

#endif /* COSTMAP_TO_POLYGONS_H_ */
