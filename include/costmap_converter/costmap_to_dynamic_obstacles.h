#ifndef COSTMAP_TO_DYNAMIC_OBSTACLES_H_
#define COSTMAP_TO_DYNAMIC_OBSTACLES_H_

#include <ros/ros.h>
#include <costmap_converter/costmap_converter_interface.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/bgsegm.hpp>

namespace costmap_converter
{
  
/**
 * @class CostmapToDynamicObstacles
 * @brief This class converts the costmap_2d into dynamic obstacles.
 */

class CostmapToDynamicObstacles : public BaseCostmapToPolygons
{
public:
    /*
     * @brief Constructor
     */
    CostmapToDynamicObstacles();

    /*
     * @brief Destructor
     */
    virtual ~CostmapToDynamicObstacles();

    /**
     * @brief Initialize the plugin
     * @param nh Nodehandle that defines the namespace for parameters
     */
    virtual void initialize(ros::NodeHandle nh);

    /**
     * @brief This method performs the actual work (conversion of the costmap to obstacles)
     */
    virtual void compute();

    /**
     * @brief Pass a pointer to the costmap to the plugin.
     * @sa updateCostmap2D
     * @param costmap Pointer to the costmap2d source
     */
    virtual void setCostmap2D(costmap_2d::Costmap2D* costmap);

    /**
     * @brief Get updated data from the previously set Costmap2D
     * @sa setCostmap2D
     */
    virtual void updateCostmap2D();

    /**
     * @brief Get a shared instance of the current obstacle container
     * @remarks If compute() or startWorker() has not been called before, this method returns an empty instance!
     * @return Shared instance of the current obstacle container
     */
    ObstacleContainerConstPtr getObstacles();

    // OpenCV Visualization, workaround für freeze
    virtual void visualize();

protected:
    /**
     * @brief Thread-safe update of the internal obstacle container (that is shared using getObstacles() from outside this class)
     * @param obstacles Updated obstacle container
     */
    void updateObstacleContainer(ObstacleContainerPtr obstacles);

   /*
    * @brief Convert a generic point type to a geometry_msgs::Polygon
    * @param point generic 2D point type
    * @param[out] polygon already instantiated polygon that will be filled with a single point
    * @tparam Point generic point type that should provide (writable) x and y member fiels.
    */
   template< typename Point>
   static void convertPointToPolygon(const Point& point, geometry_msgs::Polygon& polygon)
   {
     polygon.points.resize(1);
     polygon.points.front().x = point.x;
     polygon.points.front().y = point.y;
     polygon.points.front().z = 0;
   }


private:
    boost::mutex mutex_;
    costmap_2d::Costmap2D *costmap_;
    cv::Mat costmapMat_;
    ObstacleContainerPtr obstacles_;
    // Vector von n blobs
    std::string background_subtraction_method_;
    cv::Mat fgMask_;
    cv::Ptr<cv::BackgroundSubtractor> bgSub_;      //MOG Background subtractor
}; 

  
} //end namespace costmap_converter

#endif /* COSTMAP_TO_DYNAMIC_OBSTACLES_H_ */

