#include "costmap_converter/costmap_to_dynamic_obstacles.h"

#include <pluginlib/class_list_macros.h>

#include <opencv2/highgui.hpp> //TODO: Wieder raus, nur zum debuggen..

PLUGINLIB_EXPORT_CLASS(costmap_converter::CostmapToDynamicObstacles, costmap_converter::BaseCostmapToPolygons)


namespace  costmap_converter
{
    CostmapToDynamicObstacles::CostmapToDynamicObstacles() : BaseCostmapToPolygons()
    {
        costmap_ = NULL;
        bgSub_ = new BackgroundSubtractor();

        // Setup Blob detector
        // TODO: Make Parameters accessible from rqt tool
        cv::SimpleBlobDetector::Params blobDetParams;
        blobDetParams.minThreshold = 10;
        blobDetParams.maxThreshold = 200;

        blobDetParams.filterByColor = true; // actually filterByIntensity
        blobDetParams.blobColor = 255; // Extract light blobs

        blobDetParams.filterByArea = true;
        blobDetParams.minArea = 5; // Filter out blobs with less pixels
        blobDetParams.maxArea = 300;

        blobDetParams.filterByCircularity = true; // circularity = 4*pi*area/perimeter^2
        blobDetParams.minCircularity = 0.2;
        blobDetParams.maxCircularity = 1; // maximal 1 (in case of a circle)

        blobDetParams.filterByInertia = true;  // Filter blobs based on their elongation
        blobDetParams.minInertiaRatio = 0.2; // minimal 0 (in case of a line)
        blobDetParams.maxInertiaRatio = 1; // maximal 1 (in case of a circle)

        blobDetParams.filterByConvexity = false; // Area of the Blob / Area of its convex hull
        blobDetParams.minConvexity = 0; // minimal 0
        blobDetParams.maxConvexity = 1; // maximal 1

        blobDet_ = cv::SimpleBlobDetector::create(blobDetParams);
    }

    CostmapToDynamicObstacles::~CostmapToDynamicObstacles()
    {
        delete bgSub_;
    }

    void CostmapToDynamicObstacles::initialize(ros::NodeHandle nh)
    {
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
        bgSub_->apply(costmapMat_, fgMask_, originX, originY);


        // *** Blob detection ***
        if(!fgMask_.empty())
        {
          // fgMask is modified, therefore clone fgMask_.. Wirklich notwendig?
          cv::Mat fgMask = fgMask_.clone();

          // Closing Operation
          int morph_size = 1;
          cv::Mat element = cv::getStructuringElement( cv::MORPH_ELLIPSE,
                                                       cv::Size(2*morph_size+1, 2*morph_size+1),
                                                       cv::Point(morph_size, morph_size));
          cv::dilate( fgMask, fgMask, element ); // Eingangsbild = Ausgangsbild
          cv::dilate( fgMask, fgMask, element ); // Eingangsbild = Ausgangsbild
          cv::erode( fgMask, fgMask, element );  // Eingangsbild = Ausgangsbild

          blobDet_->detect(fgMask, keypoints_);

          cv::Mat fgMaskWithKeypoints = cv::Mat(fgMask.size(), CV_8UC3);
          cv::drawKeypoints(fgMask, keypoints_, fgMaskWithKeypoints, cv::Scalar(0,0,255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
          visualize("fgMaskWithKeyPoints", fgMaskWithKeypoints);
        }

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

    void CostmapToDynamicObstacles::visualize(std::string name, cv::Mat image)
    {
      if(!image.empty())
      {
        cv::Mat im = image.clone();
        cv::flip(im, im, 0);
        cv::imshow(name, im);
        cv::waitKey(1);
      }
    }
}
