#include "costmap_converter/costmap_to_dynamic_obstacles.h"

#include <pluginlib/class_list_macros.h>

#include <opencv2/highgui.hpp> //TODO: Wieder raus, nur zum debuggen..

PLUGINLIB_EXPORT_CLASS(costmap_converter::CostmapToDynamicObstacles, costmap_converter::BaseCostmapToPolygons)


namespace  costmap_converter
{
    CostmapToDynamicObstacles::CostmapToDynamicObstacles() : BaseCostmapToPolygons()
    {
        bgSub_ = new BackgroundSubtractor();
        costmap_ = NULL;
    }

    CostmapToDynamicObstacles::~CostmapToDynamicObstacles()
    {
        delete bgSub_;
    }

    void CostmapToDynamicObstacles::initialize(ros::NodeHandle nh)
    {
//        background_subtraction_method_ = "MOG";
//        nh.param("background_subtraction_method", background_subtraction_method_, background_subtraction_method_);

//        if(background_subtraction_method_.compare("MOG"))
//          bgSub_ = cv::bgsegm::createBackgroundSubtractorMOG();
//        else if (background_subtraction_method_.compare("MOG2"))
//          bgSub_ = cv::createBackgroundSubtractorMOG2();
//        else
//          ROS_ERROR("Unknown background subtraction method. Try \"MOG\" or \"MOG2\".");

        costmap_ = NULL;

        // Parameter setzen..
    }

    void CostmapToDynamicObstacles::compute()
    {
        if(costmapMat_.empty())
          return;

        int originX = round(costmap_->getOriginX() / costmap_->getResolution());
        int originY = round(costmap_->getOriginY() / costmap_->getResolution());
        ROS_INFO("Origin x  [m]: %f    Origin_y  [m]: %f", costmap_->getOriginX(), costmap_->getOriginY());
        ROS_INFO("Origin x [px]: %d \t Origin_y [px]: %d", originX, originY);

        // *** BackgroundSubtraction ***
//        bgSub_->apply(costmapMat_, fgMask_);
        bgSub_->apply(costmapMat_, fgMask_, originX, originY);

//        visualize();

//        if(!fgMask_.empty())
//        {
//          // *** Blob detection ***
//          // fgMask is modified, therefore clone fgMask_.. Wirklich notwendig?
//          cv::Mat fgMask = fgMask_.clone();

//          // Closing Operation
//          int morph_size = 2;
//          cv::Mat element = cv::getStructuringElement( cv::MORPH_ELLIPSE,
//                                                       cv::Size(2*morph_size+1, 2*morph_size+1),
//                                                       cv::Point(morph_size, morph_size));
//          cv::dilate( fgMask, fgMask, element ); // Eingangsbild = Ausgangsbild
//          cv::erode( fgMask, fgMask, element ); // Eingangsbild = Ausgangsbild

//          // Find Bounding Boxes
//          std::vector<std::vector<cv::Point> > contours;
//          cv::findContours(fgMask, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
//          boundingBoxes_.resize(contours.size());

//          for(int i = 0; i < contours.size(); i++)
//          {
//            boundingBoxes_[i] = cv::boundingRect(cv::Mat(contours[i]));
//          }
//        }

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
                            costmap_->getCharMap()).clone(); // Erstmal sicherer
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

    void CostmapToDynamicObstacles::visualize()
    {
        //bgSub_->visualize();
        // costmap_ and costmapMat_ share the same data
        if(!costmap_->getMutex())
          return;
        costmap_2d::Costmap2D::mutex_t::scoped_lock lock(*costmap_->getMutex());

        if(!costmapMat_.empty())
        {
          // Flip Mat to match rviz
          cv::Mat costmap;
          cv::flip(costmapMat_, costmap,0);
          cv::imshow("Costmap matrix", costmap);
        }

        if(!fgMask_.empty())
        {
          cv::Mat flippedMask;
          cv::flip(fgMask_, flippedMask,0);
          cv::imshow("Foreground mask", flippedMask);
        }

        cv::waitKey(1);
    }
}
