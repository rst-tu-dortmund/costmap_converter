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

#ifndef COSTMAP_TO_LINES_CONVEX_HULL_H_
#define COSTMAP_TO_LINES_CONVEX_HULL_H_

#include <costmap_converter_core/costmap_converter_interface.h>
#include <costmap_converter_DBScan/costmap_to_polygons.h>

// dynamic reconfigure
#include <costmap_converter_DBScan/CostmapToLinesDBSMCCHConfig.h>

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
 */
  class CostmapToLinesDBSMCCH : public CostmapToPolygonsDBSMCCH
  {
  public:

    /**
     * @brief Constructor
     */
    CostmapToLinesDBSMCCH();

    /**
     * @brief Destructor
     */
    virtual ~CostmapToLinesDBSMCCH();

    /**
     * @brief Initialize the plugin
     * @param nh Nodehandle that defines the namespace for parameters
     */
    virtual void initialize(ros::NodeHandle nh);

    /**
     * @brief This method performs the actual work (conversion of the costmap to polygons)
     */
    virtual void compute();


  protected:

    /**
     * @brief Extract points and lines from a given cluster by checking all keypoint pairs of the convex hull for a minimum number of support points
     * @param cluster list of points in the cluster
     * @param polygon convex hull of the cluster \c cluster
     * @param[out] lines back_inserter object to a sequence of polygon msgs (new lines will be pushed back)
     */
    void extractPointsAndLines(std::vector<KeyPoint>& cluster, const geometry_msgs::Polygon& polygon, std::back_insert_iterator< std::vector<geometry_msgs::Polygon> > lines);



  protected:

    double support_pts_max_dist_inbetween_;
    double support_pts_max_dist_;
    int min_support_pts_;

  private:

    /**
     * @brief Callback for the dynamic_reconfigure node.
     *
     * This callback allows to modify parameters dynamically at runtime without restarting the node
     * @param config Reference to the dynamic reconfigure config
     * @param level Dynamic reconfigure level
     */
    void reconfigureCB(CostmapToLinesDBSMCCHConfig& config, uint32_t level);

    dynamic_reconfigure::Server<CostmapToLinesDBSMCCHConfig>* dynamic_recfg_; //!< Dynamic reconfigure server to allow config modifications at runtime

  };








} //end namespace teb_local_planner

#endif /* COSTMAP_TO_LINES_CONVEX_HULL_H_ */
