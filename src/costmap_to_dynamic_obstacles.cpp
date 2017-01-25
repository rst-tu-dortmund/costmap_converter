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
        background_subtraction_method_ = "MOG";
        nh.param("background_subtraction_method", background_subtraction_method_, background_subtraction_method_);

        if(background_subtraction_method_.compare("MOG"))
          bgSub_ = cv::bgsegm::createBackgroundSubtractorMOG();
        else if (background_subtraction_method_.compare("MOG2"))
          bgSub_ = cv::createBackgroundSubtractorMOG2();
        else
          ROS_ERROR("Unknown background subtraction method. Try \"MOG\" or \"MOG2\".");

        costmap_ = NULL;
        // Parameter setzen..
    }

    void CostmapToDynamicObstacles::compute()
    {
        // BackgroundSubtraction (TODO: Eigenbewegung abziehen)
        bgSub_->apply(costmapMat_, fgMask_);

        // Blob detection
        // Verknüpfung mit Hindernissen aus letztem Bild -> IDs verteilen
        // Geschwindigkeit berechnen, Kalman filter
        // ObstacleContainerPtr mit Werten füllen
    }

    void CostmapToDynamicObstacles::setCostmap2D(costmap_2d::Costmap2D *costmap)
    {
        if(!costmap)
          return;

        costmap_ = costmap;

        updateCostmap2D();
    }

    void CostmapToDynamicObstacles::updateCostmap2D()
    {
        if (!costmap_->getMutex())
        {
          ROS_ERROR("Cannot update costmap since the mutex pointer is null");
          return;
        }

        costmap_2d::Costmap2D::mutex_t::scoped_lock lock(*costmap_->getMutex());

        // Initialize costmapMat_ directly with the unsigned char array of costmap_
        costmapMat_=cv::Mat(costmap_->getSizeInCellsX(),
                            costmap_->getSizeInCellsY(),
                            CV_8UC1,
                            costmap_->getCharMap());
    }

    ObstacleContainerConstPtr CostmapToDynamicObstacles::getObstacles()
    {
        boost::mutex::scoped_lock lock(mutex_);
        return obstacles_;
    }

    void CostmapToDynamicObstacles::updateObstacleContainer(ObstacleContainerPtr obstacles)
    {
        boost::mutex::scoped_lock lock(mutex_);
        obstacles_ = obstacles;
    }
}
