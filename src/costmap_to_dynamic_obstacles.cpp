#include "costmap_converter/costmap_to_dynamic_obstacles.h"

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(costmap_converter::CostmapToDynamicObstacles, costmap_converter::BaseCostmapToPolygons)


namespace  costmap_converter
{
    CostmapToDynamicObstacles::CostmapToDynamicObstacles() : BaseCostmapToPolygons()
    {
        costmap_ = NULL;
    }

    CostmapToDynamicObstacles::~CostmapToDynamicObstacles()
    {

    }

    void CostmapToDynamicObstacles::initialize(ros::NodeHandle nh)
    {
        costmap_ = NULL;
    }

    void CostmapToDynamicObstacles::compute()
    {
        // Blob Analyse erstmal ausgelassen.. In set_own_costmap ebenfalls auskommentiert
        // TODO: OpencvTracking _my_tracking;
        //       BackgroundSubtractionClass _bs;
        //       BlobClass _blob;
    }

    void CostmapToDynamicObstacles::setCostmap2D(costmap_2d::Costmap2D *costmap)
    {
        costmap_ = costmap;
        updateCostmap2D();
    }

    void CostmapToDynamicObstacles::updatePolygonContainer(PolygonContainerPtr polygons)
    {
        boost::mutex::scoped_lock lock(mutex_);
        polygons_ = polygons;
    }

    PolygonContainerConstPtr CostmapToDynamicObstacles::getPolygons()
    {
        boost::mutex::scoped_lock lock(mutex_);
        return polygons_;
    }
}
