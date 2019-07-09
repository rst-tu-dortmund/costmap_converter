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

#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <nav2_costmap_2d/costmap_2d.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <costmap_converter/costmap_converter_interface.h>
#include <pluginlib/class_loader.hpp>

class CostmapStandaloneConversion : public rclcpp::Node {
 public:
  CostmapStandaloneConversion(const std::string node_name)
      : rclcpp::Node(node_name),
        converter_loader_("costmap_converter",
                          "costmap_converter::BaseCostmapToPolygons") {
    costmap_ros_ =
        std::make_shared<nav2_costmap_2d::Costmap2DROS>("converter_costmap");
    costmap_thread_ = std::make_unique<std::thread>(
        [](rclcpp_lifecycle::LifecycleNode::SharedPtr node) {
          rclcpp::spin(node->get_node_base_interface());
        },
        costmap_ros_);
    rclcpp_lifecycle::State state;
    costmap_ros_->on_configure(state);
    costmap_ros_->on_activate(state);

    n_ = std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node *) {});
    // load converter plugin from parameter server, otherwise set default

    std::string converter_plugin =
        "costmap_converter::CostmapToPolygonsDBSMCCH";

    declare_parameter("converter_plugin",
                      rclcpp::ParameterValue(converter_plugin));

    get_parameter_or<std::string>("converter_plugin", converter_plugin,
                                  converter_plugin);

    try {
      converter_ = converter_loader_.createSharedInstance(converter_plugin);
    } catch (const pluginlib::PluginlibException &ex) {
      RCLCPP_ERROR(get_logger(),
                   "The plugin failed to load for some reason. Error: %s",
                   ex.what());
      rclcpp::shutdown();
      return;
    }

    RCLCPP_INFO(get_logger(), "Standalone costmap converter: %s loaded.",
                converter_plugin.c_str());

    std::string obstacles_topic = "costmap_obstacles";
    declare_parameter("obstacles_topic",
                      rclcpp::ParameterValue(obstacles_topic));
    get_parameter_or<std::string>("obstacles_topic", obstacles_topic,
                                  obstacles_topic);

    std::string polygon_marker_topic = "costmap_polygon_markers";
    declare_parameter("polygon_marker_topic",
                      rclcpp::ParameterValue(polygon_marker_topic));
    get_parameter_or<std::string>("polygon_marker_topic", polygon_marker_topic,
                                  polygon_marker_topic);

    obstacle_pub_ =
        create_publisher<costmap_converter_msgs::msg::ObstacleArrayMsg>(
            obstacles_topic, 1000);
    marker_pub_ = create_publisher<visualization_msgs::msg::Marker>(
        polygon_marker_topic, 10);

    occupied_min_value_ = 100;
    declare_parameter("occupied_min_value",
                      rclcpp::ParameterValue(occupied_min_value_));
    get_parameter_or<int>("occupied_min_value", occupied_min_value_,
                          occupied_min_value_);

    std::string odom_topic = "/odom";
    declare_parameter("odom_topic", rclcpp::ParameterValue(odom_topic));
    get_parameter_or<std::string>("odom_topic", odom_topic, odom_topic);

    if (converter_) {
      converter_->setOdomTopic(odom_topic);
      converter_->initialize(
          std::make_shared<rclcpp::Node>("intra_node", "costmap_converter"));
      converter_->startWorker(std::make_shared<rclcpp::Rate>(5),
                              costmap_ros_->getCostmap(), true);
    }

    pub_timer_ = n_->create_wall_timer(
        std::chrono::milliseconds(200),
        std::bind(&CostmapStandaloneConversion::publishCallback, this));
  }

  void publishCallback() {
    costmap_converter::ObstacleArrayConstPtr obstacles =
        converter_->getObstacles();

    if (!obstacles) return;

    obstacle_pub_->publish(*obstacles);

    frame_id_ = costmap_ros_->getGlobalFrameID();

    publishAsMarker(frame_id_, *obstacles);
  }

  void publishAsMarker(
      const std::string &frame_id,
      const std::vector<geometry_msgs::msg::PolygonStamped> &polygonStamped) {
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

    for (std::size_t i = 0; i < polygonStamped.size(); ++i) {
      for (int j = 0; j < (int)polygonStamped[i].polygon.points.size() - 1;
           ++j) {
        geometry_msgs::msg::Point line_start;
        line_start.x = polygonStamped[i].polygon.points[j].x;
        line_start.y = polygonStamped[i].polygon.points[j].y;
        line_list.points.push_back(line_start);
        geometry_msgs::msg::Point line_end;
        line_end.x = polygonStamped[i].polygon.points[j + 1].x;
        line_end.y = polygonStamped[i].polygon.points[j + 1].y;
        line_list.points.push_back(line_end);
      }
      // close loop for current polygon
      if (!polygonStamped[i].polygon.points.empty() &&
          polygonStamped[i].polygon.points.size() != 2) {
        geometry_msgs::msg::Point line_start;
        line_start.x = polygonStamped[i].polygon.points.back().x;
        line_start.y = polygonStamped[i].polygon.points.back().y;
        line_list.points.push_back(line_start);
        if (line_list.points.size() % 2 != 0) {
          geometry_msgs::msg::Point line_end;
          line_end.x = polygonStamped[i].polygon.points.front().x;
          line_end.y = polygonStamped[i].polygon.points.front().y;
          line_list.points.push_back(line_end);
        }
      }
    }
    marker_pub_->publish(line_list);
  }

  void publishAsMarker(
      const std::string &frame_id,
      const costmap_converter_msgs::msg::ObstacleArrayMsg &obstacles) {
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

    for (const auto &obstacle : obstacles.obstacles) {
      for (int j = 0; j < (int)obstacle.polygon.points.size() - 1; ++j) {
        geometry_msgs::msg::Point line_start;
        line_start.x = obstacle.polygon.points[j].x;
        line_start.y = obstacle.polygon.points[j].y;
        line_list.points.push_back(line_start);
        geometry_msgs::msg::Point line_end;
        line_end.x = obstacle.polygon.points[j + 1].x;
        line_end.y = obstacle.polygon.points[j + 1].y;
        line_list.points.push_back(line_end);
      }
      // close loop for current polygon
      if (!obstacle.polygon.points.empty() &&
          obstacle.polygon.points.size() != 2) {
        geometry_msgs::msg::Point line_start;
        line_start.x = obstacle.polygon.points.back().x;
        line_start.y = obstacle.polygon.points.back().y;
        line_list.points.push_back(line_start);
        if (line_list.points.size() % 2 != 0) {
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
  pluginlib::ClassLoader<costmap_converter::BaseCostmapToPolygons>
      converter_loader_;
  std::shared_ptr<costmap_converter::BaseCostmapToPolygons> converter_;

  rclcpp::Node::SharedPtr n_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  std::unique_ptr<std::thread> costmap_thread_;
  rclcpp::Publisher<costmap_converter_msgs::msg::ObstacleArrayMsg>::SharedPtr
      obstacle_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  rclcpp::TimerBase::SharedPtr pub_timer_;

  std::string frame_id_;
  int occupied_min_value_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto convert_process =
      std::make_shared<CostmapStandaloneConversion>("costmap_converter");

  rclcpp::spin(convert_process);

  return 0;
}
