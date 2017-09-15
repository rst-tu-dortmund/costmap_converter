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

#include <ros/ros.h>

#include <costmap_2d/costmap_2d.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PolygonStamped.h>
#include <visualization_msgs/Marker.h>

#include <costmap_converter/costmap_converter_interface.h>
#include <pluginlib/class_loader.h>



class CostmapStandaloneConversion
{
public:
  CostmapStandaloneConversion() : converter_loader_("costmap_converter", "costmap_converter::BaseCostmapToPolygons"), n_("~")
  {
      
      std::string converter_plugin = "costmap_converter::CostmapToDynamicObstacles";
      n_.param("converter_plugin", converter_plugin, converter_plugin);
      
      try
      {
        converter_ = converter_loader_.createInstance(converter_plugin);
      }
      catch(pluginlib::PluginlibException& ex)
      {
        ROS_ERROR("The plugin failed to load for some reason. Error: %s", ex.what());
        ros::shutdown();
      }
      
      ROS_INFO_STREAM(converter_plugin << " loaded.");
      
      std::string costmap_topic = "move_base/local_costmap/costmap";
      n_.param("costmap_topic", costmap_topic, costmap_topic);

      std::string obstacles_topic = "costmap_obstacles";
      n_.param("obstacles_topic", obstacles_topic, obstacles_topic);

      std::string polygon_marker_topic = "costmap_polygon_markers";
      n_.param("polygon_marker_topic", polygon_marker_topic, polygon_marker_topic);

      costmap_sub_ = n_.subscribe(costmap_topic, 1, &CostmapStandaloneConversion::costmapCallback, this);
      obstacle_pub_ = n_.advertise<costmap_converter::ObstacleArrayMsg>(obstacles_topic, 1000);
      marker_pub_ = n_.advertise<visualization_msgs::Marker>(polygon_marker_topic, 10);
      
      frame_id_ = "/map";
      n_.param("frame_id", frame_id_, frame_id_);

      occupied_min_value_ = 100;
      n_.param("occupied_min_value", occupied_min_value_, occupied_min_value_);
      
      if (converter_)
      {
        converter_->initialize(n_);
        converter_->setCostmap2D(&map); 
        //converter_->startWorker(ros::Rate(5), &map, true);
      }
   }
   
  
  void costmapCallback(const nav_msgs::OccupancyGridConstPtr& msg)
  {
      ROS_INFO_ONCE("Got first costmap callback. This message will be printed once");
      
      if (msg->info.width != map.getSizeInCellsX() || msg->info.height != map.getSizeInCellsY() || msg->info.resolution != map.getResolution())
      {
        ROS_INFO("New map format, resizing and resetting map...");
        map.resizeMap(msg->info.width, msg->info.height, msg->info.resolution, msg->info.origin.position.x, msg->info.origin.position.y);
      }
      else
      {
        map.updateOrigin(msg->info.origin.position.x, msg->info.origin.position.y);
      }
      
      
      for (std::size_t i=0; i < msg->data.size(); ++i)
      {
        unsigned int mx, my;
        map.indexToCells((unsigned int)i, mx, my);
        map.setCost(mx, my, msg->data[i] >= occupied_min_value_ ? 255 : 0 );
      }
      
      // convert
      converter_->updateCostmap2D();
      converter_->compute();
      costmap_converter::ObstacleArrayConstPtr obstacles = converter_->getObstacles();

      if (!obstacles)
        return;

      obstacle_pub_.publish(obstacles);
      
      publishAsMarker(msg->header.frame_id, *obstacles, marker_pub_);
      
  }
  
  void publishAsMarker(const std::string& frame_id, const std::vector<geometry_msgs::PolygonStamped>& polygonStamped, ros::Publisher& marker_pub)
  {
    visualization_msgs::Marker line_list;
    line_list.header.frame_id = frame_id;
    line_list.header.stamp = ros::Time::now();
    line_list.ns = "Polygons";
    line_list.action = visualization_msgs::Marker::ADD;
    line_list.pose.orientation.w = 1.0;
    
    line_list.id = 0;
    line_list.type = visualization_msgs::Marker::LINE_LIST;
    
    line_list.scale.x = 0.1;
    line_list.color.g = 1.0;
    line_list.color.a = 1.0;
    
    for (std::size_t i=0; i<polygonStamped.size(); ++i)
    {
      for (int j=0; j< (int)polygonStamped[i].polygon.points.size()-1; ++j)
      {
        geometry_msgs::Point line_start;
        line_start.x = polygonStamped[i].polygon.points[j].x;
        line_start.y = polygonStamped[i].polygon.points[j].y;
        line_list.points.push_back(line_start);
        geometry_msgs::Point line_end;
        line_end.x = polygonStamped[i].polygon.points[j+1].x;
        line_end.y = polygonStamped[i].polygon.points[j+1].y;
        line_list.points.push_back(line_end);
      }     
      // close loop for current polygon
      if (!polygonStamped[i].polygon.points.empty() && polygonStamped[i].polygon.points.size() != 2 )
      {
        geometry_msgs::Point line_start;
        line_start.x = polygonStamped[i].polygon.points.back().x;
        line_start.y = polygonStamped[i].polygon.points.back().y;
        line_list.points.push_back(line_start);
        if (line_list.points.size() % 2 != 0)
        {
          geometry_msgs::Point line_end;
          line_end.x = polygonStamped[i].polygon.points.front().x;
          line_end.y = polygonStamped[i].polygon.points.front().y;
          line_list.points.push_back(line_end);
        }
      }
        
      
    }
    marker_pub.publish(line_list);
  }

  void publishAsMarker(const std::string& frame_id, const costmap_converter::ObstacleArrayMsg& obstacles, ros::Publisher& marker_pub)
  {
    visualization_msgs::Marker line_list;
    line_list.header.frame_id = frame_id;
    line_list.header.stamp = ros::Time::now();
    line_list.ns = "Polygons";
    line_list.action = visualization_msgs::Marker::ADD;
    line_list.pose.orientation.w = 1.0;

    line_list.id = 0;
    line_list.type = visualization_msgs::Marker::LINE_LIST;

    line_list.scale.x = 0.1;
    line_list.color.g = 1.0;
    line_list.color.a = 1.0;

    for (const costmap_converter::ObstacleMsg& obstacle : obstacles.obstacles)
    {
      for (int j=0; j< (int)obstacle.polygon.points.size()-1; ++j)
      {
        geometry_msgs::Point line_start;
        line_start.x = obstacle.polygon.points[j].x;
        line_start.y = obstacle.polygon.points[j].y;
        line_list.points.push_back(line_start);
        geometry_msgs::Point line_end;
        line_end.x = obstacle.polygon.points[j+1].x;
        line_end.y = obstacle.polygon.points[j+1].y;
        line_list.points.push_back(line_end);
      }
      // close loop for current polygon
      if (!obstacle.polygon.points.empty() && obstacle.polygon.points.size() != 2 )
      {
        geometry_msgs::Point line_start;
        line_start.x = obstacle.polygon.points.back().x;
        line_start.y = obstacle.polygon.points.back().y;
        line_list.points.push_back(line_start);
        if (line_list.points.size() % 2 != 0)
        {
          geometry_msgs::Point line_end;
          line_end.x = obstacle.polygon.points.front().x;
          line_end.y = obstacle.polygon.points.front().y;
          line_list.points.push_back(line_end);
        }
      }


    }
    marker_pub.publish(line_list);
  }
  
private:
  pluginlib::ClassLoader<costmap_converter::BaseCostmapToPolygons> converter_loader_;
  boost::shared_ptr<costmap_converter::BaseCostmapToPolygons> converter_;
  
  ros::NodeHandle n_;
  ros::Subscriber costmap_sub_;
  ros::Publisher obstacle_pub_;
  ros::Publisher marker_pub_;
  
  std::string frame_id_;
  int occupied_min_value_;
  
  costmap_2d::Costmap2D map;
  
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "costmap_converter");
  
  CostmapStandaloneConversion convert_process;
  
  ros::spin();

  costmap_2d::Costmap2D costmap;
  
  return 0;
}


