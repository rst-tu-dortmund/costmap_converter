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
        BlobDetector::Params blobDetParams;
        blobDetParams.minThreshold = 10;
        blobDetParams.maxThreshold = 255;

        blobDetParams.filterByColor = true; // actually filterByIntensity
        blobDetParams.blobColor = 255; // Extract light blobs

        blobDetParams.filterByArea = true;
        blobDetParams.minArea = 3; // Filter out blobs with less pixels
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

        blobDet_ = BlobDetector::create(blobDetParams);

        float dt = 0.6;
        float accel_noise_mag = 0.0;
        float dist_thresh = 60.0;
        size_t max_allowed_skipped_frames = 1;
        size_t max_trace_length = 10;
        tracker_ = new CTracker(dt, accel_noise_mag, dist_thresh, max_allowed_skipped_frames, max_trace_length);
    }

    CostmapToDynamicObstacles::~CostmapToDynamicObstacles()
    {
        delete bgSub_;
        delete tracker_;
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

        /////////////////////////// Foreground detection /////////////////////////////////
        // Dynamic obstacles are separated from static obstacles
        int originX = round(costmap_->getOriginX() / costmap_->getResolution());
        int originY = round(costmap_->getOriginY() / costmap_->getResolution());
//        ROS_INFO("Origin x  [m]: %f    Origin_y  [m]: %f", costmap_->getOriginX(), costmap_->getOriginY());
//        ROS_INFO("Origin x [px]: %d \t Origin_y [px]: %d", originX, originY);

        bgSub_->apply(costmapMat_, fgMask_, originX, originY);

        /////////////////////////////// Blob detection //////////////////////////////////
        // Centers and contours of Blobs are detected
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
          std::vector < std::vector<cv::Point> > contour = blobDet_->getContours();

          cv::Mat fgMaskWithKeypoints = cv::Mat::zeros(fgMask.size(), CV_8UC3);
          cv::cvtColor(fgMask, fgMaskWithKeypoints, cv::COLOR_GRAY2BGR);
//          cv::drawKeypoints(fgMask, keypoints_, fgMaskWithKeypoints, cv::Scalar(0,0,255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
//          visualize("fgMaskWithKeyPoints", fgMaskWithKeypoints);

          ////////////////////////////// Tracking /////////////////////////////////////////
          // Objects are assigned to objects from previous frame based on Hungarian Algorithm
          // Object velocities are estimated using a Kalman Filter
          std::vector<Point_t> detectedCenters(keypoints_.size());
          for (int i = 0; i < keypoints_.size(); i++)
          {
            detectedCenters.at(i).x = keypoints_.at(i).pt.x;
            detectedCenters.at(i).y = keypoints_.at(i).pt.y;
            detectedCenters.at(i).z = 0;  // Currently unused!
          }

          std::vector<cv::Rect> detectedBoundingBoxes(contour.size());
          for (int i = 0; i < contour.size(); i++)
          {
            detectedBoundingBoxes.at(i) = cv::boundingRect(contour.at(i));
          }

          tracker_->Update(detectedCenters, detectedBoundingBoxes, CTracker::CentersDist);

          // Plotten..
          for (auto p : detectedCenters)
          {
            cv::circle(fgMaskWithKeypoints, cv::Point(round(p.x), round(p.y)), 3, cv::Scalar(0, 255, 0), 1);
          }

          for (int i = 0; i < tracker_->tracks.size(); i++)
          {
            cv::rectangle(fgMaskWithKeypoints, tracker_->tracks[i]->GetLastRect(), cv::Scalar(0, 0, 255), 1);
          }

          visualize("fgMaskWithKeyPoints", fgMaskWithKeypoints);
        }

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
