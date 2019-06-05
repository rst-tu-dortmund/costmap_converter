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
#include <functional>

#include <rclcpp/rclcpp.hpp>

#include <nav2_costmap_2d/costmap_2d.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <costmap_converter/costmap_converter_interface.h>
#include <pluginlib/class_loader.hpp>

class CostmapStandaloneConversion : public rclcpp::Node
{
public:
  CostmapStandaloneConversion(const std::string node_name)
      : rclcpp::Node(node_name),
        converter_loader_("costmap_converter", "costmap_converter::BaseCostmapToPolygons")
  {
      n_ = std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node *) {});
      // load converter plugin from parameter server, otherwise set default
      std::string converter_plugin = "costmap_converter::CostmapToPolygonsDBSMCCH";
      get_parameter_or<std::string>("converter_plugin", converter_plugin, converter_plugin);

      try
      {
        converter_ = converter_loader_.createSharedInstance(converter_plugin);
      }
      catch(const pluginlib::PluginlibException& ex)
      {
        RCLCPP_ERROR(get_logger(), "The plugin failed to load for some reason. Error: %s", ex.what());
        rclcpp::shutdown();
        return;
      }

      RCLCPP_INFO(get_logger(), "Standalone costmap converter: %s loaded.", converter_plugin.c_str());

      std::string costmap_topic = "/map";
      get_parameter_or<std::string>("costmap_topic", costmap_topic, costmap_topic);

      std::string costmap_update_topic = "/map_update";
      get_parameter_or<std::string>("costmap_update_topic", costmap_update_topic, costmap_update_topic);

      std::string obstacles_topic = "costmap_obstacles";
      get_parameter_or<std::string>("obstacles_topic", obstacles_topic, obstacles_topic);

      std::string polygon_marker_topic = "costmap_polygon_markers";
      get_parameter_or<std::string>("polygon_marker_topic", polygon_marker_topic, polygon_marker_topic);

      costmap_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
                  costmap_topic,
                  std::bind(&CostmapStandaloneConversion::costmapCallback, this, std::placeholders::_1));
      costmap_update_sub_ = create_subscription<map_msgs::msg::OccupancyGridUpdate>(
                  costmap_update_topic,
                  std::bind(&CostmapStandaloneConversion::costmapUpdateCallback, this, std::placeholders::_1));

      obstacle_pub_ = create_publisher<costmap_converter_msgs::msg::ObstacleArrayMsg>(obstacles_topic, 1000);
      marker_pub_ = create_publisher<visualization_msgs::msg::Marker>(polygon_marker_topic, 10);

      occupied_min_value_ = 100;
      get_parameter_or<int>("occupied_min_value", occupied_min_value_, occupied_min_value_);

      std::string odom_topic = "/odom";
      get_parameter_or<std::string>("odom_topic", odom_topic, odom_topic);

      if (converter_)
      {
        converter_->setOdomTopic(odom_topic);
        converter_->initialize(n_);
        converter_->setCostmap2D(&map_);
        //converter_->startWorker(rclcpp::Rate(5), &map, true);
      }
   }


  void costmapCallback(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg)
  {
      RCLCPP_INFO_ONCE(get_logger(), "Got first costmap callback. This message will be printed once");

      if (msg->info.width != map_.getSizeInCellsX() || msg->info.height != map_.getSizeInCellsY() || msg->info.resolution != map_.getResolution())
      {
        RCLCPP_INFO(get_logger(), "New map format, resizing and resetting map...");
        map_.resizeMap(msg->info.width, msg->info.height, msg->info.resolution, msg->info.origin.position.x, msg->info.origin.position.y);
      }
      else
      {
        map_.updateOrigin(msg->info.origin.position.x, msg->info.origin.position.y);
      }


      for (std::size_t i=0; i < msg->data.size(); ++i)
      {
        unsigned int mx, my;
        map_.indexToCells((unsigned int)i, mx, my);
        map_.setCost(mx, my, msg->data[i] >= occupied_min_value_ ? 255 : 0 );
      }

      // convert
      converter_->updateCostmap2D();
      converter_->compute();
      costmap_converter::ObstacleArrayConstPtr obstacles = converter_->getObstacles();

      if (!obstacles)
        return;

      obstacle_pub_->publish(obstacles);

      frame_id_ = msg->header.frame_id;

      publishAsMarker(frame_id_, *obstacles);
  }

  void costmapUpdateCallback(const map_msgs::msg::OccupancyGridUpdate::ConstSharedPtr update)
  {
    unsigned int di = 0;
    for (unsigned int y = 0; y < update->height ; ++y)
    {
      for (unsigned int x = 0; x < update->width ; ++x)
      {
        map_.setCost(x, y, update->data[di++] >= occupied_min_value_ ? 255 : 0 );
      }
    }

    // convert
    // TODO(roesmann): currently, the converter updates the complete costmap and not the part which is updated in this callback
    converter_->updateCostmap2D();
    converter_->compute();
    costmap_converter::ObstacleArrayConstPtr obstacles = converter_->getObstacles();

    if (!obstacles)
      return;

    obstacle_pub_->publish(obstacles);

    publishAsMarker(frame_id_, *obstacles);
  }

  void publishAsMarker(const std::string& frame_id, const std::vector<geometry_msgs::msg::PolygonStamped>& polygonStamped)
  {
    visualization_msgs::msg::Marker line_list;
    line_list.header.frame_id = frame_id;
    line_list.header.stamp = now();
    line_list.ns = "Polygons";
    line_list.action = visualization_msgs::msg::Marker::ADD;
    line_list.pose.orientation.w = 1.0;

    line_list.id = 0;
    line_list.type = visualization_msgs::msg::Marker::LINE_LIST;

    line_list.scale.x = 0.1;
    line_list.color.g = 1.0;
    line_list.color.a = 1.0;

    for (std::size_t i=0; i<polygonStamped.size(); ++i)
    {
      for (int j=0; j< (int)polygonStamped[i].polygon.points.size()-1; ++j)
      {
        geometry_msgs::msg::Point line_start;
        line_start.x = polygonStamped[i].polygon.points[j].x;
        line_start.y = polygonStamped[i].polygon.points[j].y;
        line_list.points.push_back(line_start);
        geometry_msgs::msg::Point line_end;
        line_end.x = polygonStamped[i].polygon.points[j+1].x;
        line_end.y = polygonStamped[i].polygon.points[j+1].y;
        line_list.points.push_back(line_end);
      }
      // close loop for current polygon
      if (!polygonStamped[i].polygon.points.empty() && polygonStamped[i].polygon.points.size() != 2 )
      {
        geometry_msgs::msg::Point line_start;
        line_start.x = polygonStamped[i].polygon.points.back().x;
        line_start.y = polygonStamped[i].polygon.points.back().y;
        line_list.points.push_back(line_start);
        if (line_list.points.size() % 2 != 0)
        {
          geometry_msgs::msg::Point line_end;
          line_end.x = polygonStamped[i].polygon.points.front().x;
          line_end.y = polygonStamped[i].polygon.points.front().y;
          line_list.points.push_back(line_end);
        }
      }


    }
    marker_pub_->publish(line_list);
  }

  void publishAsMarker(const std::string& frame_id, const costmap_converter_msgs::msg::ObstacleArrayMsg& obstacles)
  {
    visualization_msgs::msg::Marker line_list;
    line_list.header.frame_id = frame_id;
    line_list.header.stamp = now();
    line_list.ns = "Polygons";
    line_list.action = visualization_msgs::msg::Marker::ADD;
    line_list.pose.orientation.w = 1.0;

    line_list.id = 0;
    line_list.type = visualization_msgs::msg::Marker::LINE_LIST;

    line_list.scale.x = 0.1;
    line_list.color.g = 1.0;
    line_list.color.a = 1.0;

    for (const auto& obstacle : obstacles.obstacles)
    {
      for (int j=0; j< (int)obstacle.polygon.points.size()-1; ++j)
      {
        geometry_msgs::msg::Point line_start;
        line_start.x = obstacle.polygon.points[j].x;
        line_start.y = obstacle.polygon.points[j].y;
        line_list.points.push_back(line_start);
        geometry_msgs::msg::Point line_end;
        line_end.x = obstacle.polygon.points[j+1].x;
        line_end.y = obstacle.polygon.points[j+1].y;
        line_list.points.push_back(line_end);
      }
      // close loop for current polygon
      if (!obstacle.polygon.points.empty() && obstacle.polygon.points.size() != 2 )
      {
        geometry_msgs::msg::Point line_start;
        line_start.x = obstacle.polygon.points.back().x;
        line_start.y = obstacle.polygon.points.back().y;
        line_list.points.push_back(line_start);
        if (line_list.points.size() % 2 != 0)
        {
          geometry_msgs::msg::Point line_end;
          line_end.x = obstacle.polygon.points.front().x;
          line_end.y = obstacle.polygon.points.front().y;
          line_list.points.push_back(line_end);
        }
      }


    }
    marker_pub_->publish(line_list);
  }

private:
  pluginlib::ClassLoader<costmap_converter::BaseCostmapToPolygons> converter_loader_;
  std::shared_ptr<costmap_converter::BaseCostmapToPolygons> converter_;

  rclcpp::Node::SharedPtr n_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
  rclcpp::Subscription<map_msgs::msg::OccupancyGridUpdate>::SharedPtr costmap_update_sub_;
  rclcpp::Publisher<costmap_converter_msgs::msg::ObstacleArrayMsg>::SharedPtr obstacle_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;

  std::string frame_id_;
  int occupied_min_value_;

  nav2_costmap_2d::Costmap2D map_;

};


int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto convert_process = std::make_shared<CostmapStandaloneConversion>("costmap_converter");

  rclcpp::spin(convert_process);

  return 0;
}


