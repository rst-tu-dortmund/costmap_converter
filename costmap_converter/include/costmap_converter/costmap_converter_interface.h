/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016
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

#ifndef COSTMAP_CONVERTER_INTERFACE_H_
#define COSTMAP_CONVERTER_INTERFACE_H_

//#include <costmap_2d/costmap_2d_ros.h>
#include <mutex>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <nav2_costmap_2d/costmap_2d.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <geometry_msgs/msg/polygon.hpp>
#include <costmap_converter_msgs/msg/obstacle_array_msg.hpp>

namespace costmap_converter
{
  
//! Typedef for a shared dynamic obstacle container
typedef costmap_converter_msgs::msg::ObstacleArrayMsg::SharedPtr ObstacleArrayPtr;
//! Typedef for a shared dynamic obstacle container (read-only access)
typedef costmap_converter_msgs::msg::ObstacleArrayMsg::ConstSharedPtr ObstacleArrayConstPtr;

//! Typedef for a shared polygon container 
typedef std::shared_ptr<std::vector<geometry_msgs::msg::Polygon>> PolygonContainerPtr;
//! Typedef for a shared polygon container (read-only access)
typedef std::shared_ptr<const std::vector<geometry_msgs::msg::Polygon>> PolygonContainerConstPtr;
  

/**
 * @class BaseCostmapToPolygons
 * @brief This abstract class defines the interface for plugins that convert the costmap into polygon types
 * 
 * Plugins must accept a nav2_costmap_2d::Costmap2D datatype as information source.
 * The interface provides two different use cases:
 * 1. Manual call to conversion routines: setCostmap2D(), compute() and getPolygons() 
 *    (in subsequent calls setCostmap2D() can be substituted by updateCostmap2D())
 * 2. Repeatedly process costmap with a specific rate (startWorker() and stopWorker()):
 *    Make sure that the costmap is valid while the worker is active (you can specify your own spinner or activate a threaded spinner).
 *    Costmaps can be obtained by invoking getPolygons().
 */
class BaseCostmapToPolygons
{
public: 
  
    /**
     * @brief Initialize the plugin
     * @param nh Nodehandle that defines the namespace for parameters
     */
    virtual void initialize(rclcpp::Node::SharedPtr nh) {
      nh_ = nh;
    }
    
    /**
     * @brief Destructor
     */
    virtual ~BaseCostmapToPolygons() 
    {
      stopWorker();
    }

    
    /**
     * @brief Pass a pointer to the costap to the plugin.
     * @warning The plugin should handle the costmap's mutex locking.
     * @sa updateCostmap2D
     * @param costmap Pointer to the costmap2d source
     */
    virtual void setCostmap2D(nav2_costmap_2d::Costmap2D* costmap) = 0;
    
    /**
     * @brief Get updated data from the previously set Costmap2D
     * @warning The plugin should handle the costmap's mutex locking.
     * @sa setCostmap2D
     */
    virtual void updateCostmap2D() = 0;
    
     /**
     * @brief This method performs the actual work (conversion of the costmap to polygons)
     */
    virtual void compute() = 0;
    
    /**
     * @brief Get a shared instance of the current polygon container
     *
     * If this method is not implemented by any subclass (plugin) the returned shared
     * pointer is empty.
     * @remarks If compute() or startWorker() has not been called before, this method returns an empty instance!
     * @warning The underlying plugin must ensure that this method is thread safe.
     * @return Shared instance of the current polygon container
     */
    virtual PolygonContainerConstPtr getPolygons(){return PolygonContainerConstPtr();}

  /**
   * @brief Get a shared instance of the current obstacle container
   * If this method is not overwritten by the underlying plugin, the obstacle container just imports getPolygons().
   * @remarks If compute() or startWorker() has not been called before, this method returns an empty instance!
   * @warning The underlying plugin must ensure that this method is thread safe.
   * @return Shared instance of the current obstacle container
   * @sa getPolygons
   */
    virtual ObstacleArrayConstPtr getObstacles()
    {
      ObstacleArrayPtr obstacles = std::make_shared<costmap_converter_msgs::msg::ObstacleArrayMsg>();
      PolygonContainerConstPtr polygons = getPolygons();
      if (polygons)
      {
        for (const geometry_msgs::msg::Polygon& polygon : *polygons)
        {
          obstacles->obstacles.emplace_back();
          obstacles->obstacles.back().polygon = polygon;
        }
      }
      return obstacles;
    }

    /**
     * @brief Set name of robot's odometry topic
     *
     * Some plugins might require odometry information
     * to compensate the robot's ego motion
     * @param odom_topic topic name
     */
    virtual void setOdomTopic(const std::string& odom_topic) { (void)odom_topic; }

    /**
     * @brief Determines whether an additional plugin for subsequent costmap conversion is specified
     *
     * @return false, since all plugins for static costmap conversion are independent
     */
    virtual bool stackedCostmapConversion() {return false;}

     /**
      * @brief Instantiate a worker that repeatedly coverts the most recent costmap to polygons.
      * The worker is implemented as a timer event that is invoked at a specific \c rate.
      * The passed \c costmap pointer must be valid at the complete time and must be lockable.
      * By specifying the argument \c spin_thread the timer event is invoked in a separate
      * thread and callback queue or otherwise it is called from the global callback queue (of the
      * node in which the plugin is used).
      * @param rate The rate that specifies how often the costmap should be updated
      * @param costmap Pointer to the underlying costmap (must be valid and lockable as long as the worker is active
      * @param spin_thread if \c true,the timer is invoked in a separate thread, otherwise in the default callback queue)
     */
    void startWorker(rclcpp::Rate::SharedPtr rate, nav2_costmap_2d::Costmap2D* costmap, bool spin_thread = false)
    {
      setCostmap2D(costmap);
      
      if (spin_thread_)
      {
        {
          std::lock_guard<std::mutex> terminate_lock(terminate_mutex_);
          need_to_terminate_ = true;
        }
        spin_thread_->join();
        delete spin_thread_;
      }
      
      if (spin_thread)
      {
        RCLCPP_DEBUG(nh_->get_logger(), "costmap_converter.\n Spinning up a thread for the CostmapToPolygons plugin");
        need_to_terminate_ = false;
        
        worker_timer_ = nh_->create_wall_timer(
                    rate->period(),
                    std::bind(&BaseCostmapToPolygons::workerCallback, this));
        spin_thread_ = new std::thread(std::bind(&BaseCostmapToPolygons::spinThread, this));
      }
      else
      {
        worker_timer_ = nh_->create_wall_timer(
                    rate->period(),
                    std::bind(&BaseCostmapToPolygons::workerCallback, this));
        spin_thread_ = nullptr;
      }
    }
    
    /**
     * @brief Stop the worker that repeatedly converts the costmap to polygons
     */
    void stopWorker()
    {
      if (worker_timer_) worker_timer_->cancel();
      if (spin_thread_)
      {
        {
          std::lock_guard<std::mutex> terminate_lock(terminate_mutex_);
          need_to_terminate_ = true;
        }
        spin_thread_->join();
        delete spin_thread_;
      }
    }

protected:
  
    /**
     * @brief Protected constructor that should be called by subclasses
     */
    BaseCostmapToPolygons() : //nh_("~costmap_to_polygons"),
        nh_(nullptr),
        spin_thread_(nullptr), need_to_terminate_(false) {}
    
    /**
     * @brief Blocking method that checks for new timer events (active if startWorker() is called with spin_thread enabled) 
     */
    void spinThread()
    {
      while (rclcpp::ok())
      {
        {
          std::lock_guard<std::mutex> terminate_lock(terminate_mutex_);
          if (need_to_terminate_)
            break;
          rclcpp::spin_some(nh_);
        }
      }
    }
    
    /**
     * @brief The callback of the worker that performs the actual work (updating the costmap and converting it to polygons)
     */
    void workerCallback()
    {
      updateCostmap2D();
      compute();
    }

    rclcpp::Logger getLogger() const
    {
        return nh_->get_logger();
    }

    rclcpp::Time now() const
    {
        return nh_->now();
    }
    
private:
  rclcpp::TimerBase::SharedPtr worker_timer_;
  rclcpp::Node::SharedPtr nh_;
  std::thread* spin_thread_;
  std::mutex terminate_mutex_;
  bool need_to_terminate_;
};    


/**
 * @class BaseCostmapToDynamicPolygons
 * @brief This class extends the BaseCostmapToPolygongs class for dynamic costmap conversion plugins in order to incorporate a subsequent costmap converter plugin for static obstacles
 *
 * This class should not be instantiated.
 */
class BaseCostmapToDynamicObstacles : public BaseCostmapToPolygons
{
public:

  /**
   * @brief Load underlying static costmap conversion plugin via plugin-loader
   * @param plugin_name Exact class name of the plugin to be loaded, e.g.
   *                    costmap_converter::CostmapToPolygonsDBSMCCH
   * @param nh_parent   NodeHandle which is extended by the namespace of the static conversion plugin
   */
  void loadStaticCostmapConverterPlugin(const std::string& plugin_name, rclcpp::Node::SharedPtr nh_parent)
  {
    try
    {
      static_costmap_converter_ = static_converter_loader_.createSharedInstance(plugin_name);

      if(std::dynamic_pointer_cast<BaseCostmapToDynamicObstacles>(static_costmap_converter_))
      {
        throw pluginlib::PluginlibException("The specified plugin for static costmap conversion is a dynamic plugin. Specify a static plugin.");
      }
//      std::string raw_plugin_name = static_converter_loader_.getName(plugin_name);
      static_costmap_converter_->initialize(nh_parent);
      setStaticCostmapConverterPlugin(static_costmap_converter_);
      RCLCPP_INFO(getLogger(), "CostmapToDynamicObstacles: underlying costmap conversion plugin for static obstacles %s loaded.", plugin_name.c_str());
    }
    catch(const pluginlib::PluginlibException& ex)
    {
      RCLCPP_WARN(getLogger(), "CostmapToDynamicObstacles: The specified costmap converter plugin cannot be loaded. "
                               "Continuing without subsequent conversion of static obstacles. Error message: %s", ex.what());
      static_costmap_converter_.reset();
    }
  }

  /**
   * @brief Set the underlying plugin for subsequent costmap conversion of the static background of the costmap
   * @param static_costmap_converter shared pointer to the static costmap conversion plugin
   */
  void setStaticCostmapConverterPlugin(std::shared_ptr<BaseCostmapToPolygons> static_costmap_converter)
  {
    static_costmap_converter_ = static_costmap_converter;
  }

  /**
   * @brief Set the costmap for the underlying plugin
   * @param static_costmap Costmap2D, which contains the static part of the original costmap
   */
  void setStaticCostmap(std::shared_ptr<nav2_costmap_2d::Costmap2D> static_costmap)
  {
    static_costmap_converter_->setCostmap2D(static_costmap.get());
  }

  /**
   * @brief Apply the underlying plugin for static costmap conversion
   */
  void convertStaticObstacles()
  {
    static_costmap_converter_->compute();
  }

  /**
   * @brief Get the converted polygons from the underlying plugin
   * @return Shared instance of the underlying plugins polygon container
   */
  PolygonContainerConstPtr getStaticPolygons()
  {
    return static_costmap_converter_->getPolygons();
  }

  /**
   * @brief Determines whether an additional plugin for subsequent costmap conversion is specified
   *
   * @return true, if a plugin for subsequent costmap conversion is specified
   */
  bool stackedCostmapConversion()
  {
    if(static_costmap_converter_)
      return true;
    else
      return false;
  }

protected:
  /**
   * @brief Protected constructor that should be called by subclasses
   */
  BaseCostmapToDynamicObstacles() : static_converter_loader_("costmap_converter", "costmap_converter::BaseCostmapToPolygons"), static_costmap_converter_() {}

private:
  pluginlib::ClassLoader<BaseCostmapToPolygons> static_converter_loader_;
  std::shared_ptr<BaseCostmapToPolygons> static_costmap_converter_;
};


}



#endif // end COSTMAP_CONVERTER_INTERFACE_H_
