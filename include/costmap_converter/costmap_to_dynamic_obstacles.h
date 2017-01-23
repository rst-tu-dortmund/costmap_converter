#ifndef COSTMAP_TO_DYNAMIC_OBSTACLES_H_
#define COSTMAP_TO_DYNAMIC_OBSTACLES_H_

#include <ros/ros.h>
#include <costmap_converter/costmap_converter_interface.h>

#include "cv_bridge/cv_bridge.h"


namespace costmap_converter
{
  
/**
 * @class CostmapToDynamicObstacles
 * @brief This class converts the costmap_2d into dynamic obstacles.
 */

class CostmapToDynamicObstacles : public BaseCostmapToPolygons
{
public:
    CostmapToDynamicObstacles();
    
    ~CostmapToDynamicObstacles();
    
    virtual void initialize(ros::NodeHandle nh);

    virtual void compute();

    virtual void setCostmap2D(costmap_2d::Costmap2D* costmap);

    virtual void updateCostmap2D();

    void updatePolygonContainer(PolygonContainerPtr polygons);

    PolygonContainerConstPtr getPolygons();

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
    PolygonContainerPtr polygons_;
    cv::Mat _frame;
    // ObstacleContainerPtr obstacles_;
    // Vector von n blobs
}; 

  
} //end namespace costmap_converter

#endif /* COSTMAP_TO_DYNAMIC_OBSTACLES_H_ */

