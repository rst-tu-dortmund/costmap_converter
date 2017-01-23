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

        // Parameter setzen..
    }

    void CostmapToDynamicObstacles::compute()
    {
        // BackgroundSubtraction von Costmap (vorher in cv::Mat umwandeln, evtl Eigenbewegung abziehen)
        // Blob detection
        // Verknüpfung mit Hindernissen aus letztem Bild -> IDs verteilen
        // Geschwindigkeit berechnen, Kalman filter
        // ObstacleContainerPtr mit Werten füllen
    }

    void CostmapToDynamicObstacles::setCostmap2D(costmap_2d::Costmap2D *costmap)
    {
        costmap_ = costmap;
        updateCostmap2D();
    }

    void CostmapToDynamicObstacles::updateCostmap2D()
    {

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
