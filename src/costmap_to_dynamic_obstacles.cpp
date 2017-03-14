#include "costmap_converter/costmap_to_dynamic_obstacles.h"

#include <pluginlib/class_list_macros.h>
#include <tf/tf.h>

#include <opencv2/highgui.hpp> //TODO: Wieder raus, nur zum debuggen..

PLUGINLIB_EXPORT_CLASS(costmap_converter::CostmapToDynamicObstacles, costmap_converter::BaseCostmapToPolygons)

namespace costmap_converter
{
CostmapToDynamicObstacles::CostmapToDynamicObstacles() : BaseCostmapToPolygons()
{
  egoVel_.x = egoVel_.y = egoVel_.z = 0;
  costmap_ = NULL;
  dynamic_recfg_ = NULL;
}

CostmapToDynamicObstacles::~CostmapToDynamicObstacles()
{
  delete bgSub_;
  delete tracker_;

  if(dynamic_recfg_ != NULL)
    delete dynamic_recfg_;
}

void CostmapToDynamicObstacles::initialize(ros::NodeHandle nh)
{
  costmap_ = NULL;

  odomSub_ = nh.subscribe("/robot_0/odom", 1, &CostmapToDynamicObstacles::odomCallback, this);

  //////////////////////////////////
  // Foreground detection parameters
  BackgroundSubtractor::Params bgSubParams;

  bgSubParams.alpha_slow = 0.3;
  nh.param("alpha_slow", bgSubParams.alpha_slow, bgSubParams.alpha_slow);

  bgSubParams.alpha_fast = 0.85;
  nh.param("alpha_fast", bgSubParams.alpha_fast, bgSubParams.alpha_fast);

  bgSubParams.beta = 0.85;
  nh.param("beta", bgSubParams.beta, bgSubParams.beta);

  bgSubParams.minOccupancyProbability = 180;
  nh.param("min_occupancy_probability", bgSubParams.minOccupancyProbability, bgSubParams.minOccupancyProbability);

  bgSubParams.minSepBetweenFastAndSlowFilter = 80;
  nh.param("min_sep_between_slow_and_fast_filter", bgSubParams.minSepBetweenFastAndSlowFilter, bgSubParams.minSepBetweenFastAndSlowFilter);

  bgSubParams.maxOccupancyNeighbors = 100;
  nh.param("max_occupancy_neighbors", bgSubParams.maxOccupancyNeighbors, bgSubParams.maxOccupancyNeighbors);

  bgSubParams.morph_size = 1;
  nh.param("morph_size", bgSubParams.morph_size, bgSubParams.morph_size);

  bgSub_ = new BackgroundSubtractor(bgSubParams);

  ////////////////////////////
  // Blob detection parameters
  BlobDetector::Params blobDetParams;
  // constant parameters, changing these makes no sense
  blobDetParams.filterByColor = true; // actually filterByIntensity, always true
  blobDetParams.blobColor = 255;      // Extract light blobs
  blobDetParams.thresholdStep = 256;  // Input for blobDetection is already a binary image
  blobDetParams.minThreshold = 127;
  blobDetParams.maxThreshold = 255;
  blobDetParams.minRepeatability = 1;

  blobDetParams.minDistBetweenBlobs = 10;
  nh.param("min_distance_between_blobs", blobDetParams.minDistBetweenBlobs, blobDetParams.minDistBetweenBlobs);

  blobDetParams.filterByArea = true;
  nh.param("filter_by_area", blobDetParams.filterByArea, blobDetParams.filterByArea);

  blobDetParams.minArea = 3; // Filter out blobs with less pixels
  nh.param("min_area", blobDetParams.minArea, blobDetParams.minArea);

  blobDetParams.maxArea = 300;
  nh.param("max_area", blobDetParams.maxArea, blobDetParams.maxArea);

  blobDetParams.filterByCircularity = true; // circularity = 4*pi*area/perimeter^2
  nh.param("filter_by_circularity", blobDetParams.filterByCircularity, blobDetParams.filterByCircularity);

  blobDetParams.minCircularity = 0.2;
  nh.param("min_circularity", blobDetParams.minCircularity, blobDetParams.minCircularity);

  blobDetParams.maxCircularity = 1; // maximal 1 (in case of a circle)
  nh.param("max_circularity", blobDetParams.maxCircularity, blobDetParams.maxCircularity);

  blobDetParams.filterByInertia = true; // Filter blobs based on their elongation
  nh.param("filter_by_intertia", blobDetParams.filterByInertia, blobDetParams.filterByInertia);

  blobDetParams.minInertiaRatio = 0.2;  // minimal 0 (in case of a line)
  nh.param("min_inertia_ratio", blobDetParams.minInertiaRatio, blobDetParams.minInertiaRatio);

  blobDetParams.maxInertiaRatio = 1;    // maximal 1 (in case of a circle)
  nh.param("max_intertia_ratio", blobDetParams.maxInertiaRatio, blobDetParams.maxInertiaRatio);

  blobDetParams.filterByConvexity = false; // Area of the Blob / Area of its convex hull
  nh.param("filter_by_convexity", blobDetParams.filterByConvexity, blobDetParams.filterByConvexity);

  blobDetParams.minConvexity = 0;          // minimal 0
  nh.param("min_convexity", blobDetParams.minConvexity, blobDetParams.minConvexity);

  blobDetParams.maxConvexity = 1;          // maximal 1
  nh.param("max_convexity", blobDetParams.maxConvexity, blobDetParams.maxConvexity);

  blobDet_ = BlobDetector::create(blobDetParams);

  ////////////////////////////////////
  // Tracking parameters
  CTracker::Params trackerParams;
  trackerParams.dt = 0.2;
  nh.param("dt", trackerParams.dt, trackerParams.dt);

  trackerParams.dist_thresh = 60.0;
  nh.param("dist_thresh", trackerParams.dist_thresh, trackerParams.dist_thresh);

  trackerParams.max_allowed_skipped_frames = 3;
  nh.param("max_allowed_skipped_frames", trackerParams.max_allowed_skipped_frames, trackerParams.max_allowed_skipped_frames);

  trackerParams.max_trace_length = 10;
  nh.param("max_trace_length", trackerParams.max_trace_length, trackerParams.max_trace_length);

  tracker_ = new CTracker(trackerParams);

  // setup dynamic reconfigure
  dynamic_recfg_ = new dynamic_reconfigure::Server<CostmapToDynamicObstaclesConfig>(nh);
  dynamic_reconfigure::Server<CostmapToDynamicObstaclesConfig>::CallbackType cb = boost::bind(&CostmapToDynamicObstacles::reconfigureCB, this, _1, _2);
  dynamic_recfg_->setCallback(cb);
}

void CostmapToDynamicObstacles::compute()
{
  if (costmapMat_.empty())
    return;

  /////////////////////////// Foreground detection ////////////////////////////////////
  // Dynamic obstacles are separated from static obstacles
  int originX = round(costmap_->getOriginX() / costmap_->getResolution());
  int originY = round(costmap_->getOriginY() / costmap_->getResolution());
  // ROS_INFO("Origin x  [m]: %f    Origin_y  [m]: %f", costmap_->getOriginX(), costmap_->getOriginY());
  // ROS_INFO("Origin x [px]: %d \t Origin_y [px]: %d", originX, originY);

  bgSub_->apply(costmapMat_, fgMask_, originX, originY);

  // if no foreground object is detected, no ObstacleMsgs need to be published
  if (fgMask_.empty())
    return;


  /////////////////////////////// Blob detection /////////////////////////////////////
  // Centers and contours of Blobs are detected
  // fgMask is modified, therefore clone fgMask_.. Wirklich notwendig?
  cv::Mat fgMask = fgMask_.clone();

  blobDet_->detect(fgMask, keypoints_);
  std::vector<std::vector<cv::Point>> contours = blobDet_->getContours();


  ////////////////////////////// Tracking ////////////////////////////////////////////
  // Objects are assigned to objects from previous frame based on Hungarian Algorithm
  // Object velocities are estimated using a Kalman Filter
  std::vector<Point_t> detectedCenters(keypoints_.size());
  for (int i = 0; i < keypoints_.size(); i++)
  {
    detectedCenters.at(i).x = keypoints_.at(i).pt.x;
    detectedCenters.at(i).y = keypoints_.at(i).pt.y;
    detectedCenters.at(i).z = 0; // Currently unused!
  }

  tracker_->Update(detectedCenters, contours);


  ///////////////////////////////////// Output ///////////////////////////////////////
  cv::Mat fgMaskWithKeypoints = cv::Mat::zeros(fgMask.size(), CV_8UC3);
  cv::cvtColor(fgMask, fgMaskWithKeypoints, cv::COLOR_GRAY2BGR);

  for (auto p : detectedCenters)
    cv::circle(fgMaskWithKeypoints, cv::Point(round(p.x), round(p.y)), 3, cv::Scalar(0, 255, 0), 1);

  for (int i = 0; i < tracker_->tracks.size(); i++)
    cv::rectangle(fgMaskWithKeypoints, cv::boundingRect(tracker_->tracks[i]->getLastContour()),
                  cv::Scalar(0, 0, 255), 1);

  visualize("fgMaskWithKeyPoints", fgMaskWithKeypoints);

//  if(!tracker_->tracks.empty())
//  {
//    Point_t vel = getEstimatedVelocityOfObject(0);
//    ROS_INFO("Estimated: vel_x = %f, vel_y = %f, vel_z = %f", vel.x, vel.y, vel.z);
//  }


  //////////////////////////// ObstacleContainerPtr füllen /////////////////////////////
  ObstacleContainerPtr obstacles (new teb_local_planner::ObstacleMsg);
  // header.seq is automatically filled
  obstacles->header.stamp = ros::Time::now();
  obstacles->header.frame_id = "/map"; //Global frame /map

  // For all tracked objects
  for (unsigned int i = 0; i < tracker_->tracks.size(); i++)
  {
    geometry_msgs::PolygonStamped polygonStamped;
    // TODO: Header?
//    polygonStamped.header.stamp = ros::Time::now();
//    polygonStamped.header.frame_id = "/map"; // Global frame /map

    // Append each polygon point
    for (unsigned int j = 0; j < getContour(i).size(); j++)
    {
      polygonStamped.polygon.points.push_back(geometry_msgs::Point32());
      polygonStamped.polygon.points.at(j).x = getContour(i).at(j).x;
      polygonStamped.polygon.points.at(j).y = getContour(i).at(j).y;
      polygonStamped.polygon.points.at(j).z = 0;
    }

    obstacles->obstacles.push_back(polygonStamped);

    // Set obstacle ID
    obstacles->ids.push_back(tracker_->tracks.at(i)->track_id);

    // Set orientation
    geometry_msgs::QuaternionStamped orientationStamped;
//  TODO: Header?
//    orientationStamped.header.stamp = ros::Time::now();
//    orientationStamped.header.frame_id = "/map"; //Global frame /map
    Point_t vel = getEstimatedVelocityOfObject(i);
    double yaw = std::atan2(vel.y, vel.x);
    //ROS_INFO("yaw: %f", yaw);
    orientationStamped.quaternion = tf::createQuaternionMsgFromYaw(yaw);
    obstacles->orientations.push_back(orientationStamped);

    // Set velocity
    geometry_msgs::TwistWithCovariance velocities;
    velocities.twist.linear.x = std::sqrt(vel.x*vel.x + vel.y*vel.y);
    velocities.twist.linear.y = 0;
    velocities.twist.linear.z = 0;
    velocities.twist.angular.x = 0;
    velocities.twist.angular.y = 0;
    velocities.twist.angular.z = 0;

    // TODO: use correct covariance matrix
    velocities.covariance = {1, 0, 0, 0, 0, 0,
                             0, 1, 0, 0, 0, 0,
                             0, 0, 1, 0, 0, 0,
                             0, 0, 0, 1, 0, 0,
                             0, 0, 0, 0, 1, 0,
                             0, 0, 0, 0, 0, 1};

    obstacles->velocities.push_back(velocities);
  }

  updateObstacleContainer(obstacles);
}

void CostmapToDynamicObstacles::setCostmap2D(costmap_2d::Costmap2D* costmap)
{
  if (!costmap)
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
  costmapMat_ = cv::Mat(costmap_->getSizeInCellsX(), costmap_->getSizeInCellsY(), CV_8UC1,
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

Point_t CostmapToDynamicObstacles::getEstimatedVelocityOfObject(unsigned int idx)
{
  // vel [px/s] * costmapResolution [m/px] = vel [m/s]
  Point_t vel = tracker_->tracks.at(idx)->getEstimatedVelocity() * costmap_->getResolution() + egoVel_;

  //ROS_INFO("vel x: %f, vel y: %f, vel z: %f", vel.x, vel.y, vel.z);
  // velocity in /map frame
  return vel;
}

void CostmapToDynamicObstacles::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  tf::Quaternion pose;
  tf::quaternionMsgToTF(msg->pose.pose.orientation, pose);

  tf::Vector3 twistLinear;
  tf::vector3MsgToTF(msg->twist.twist.linear, twistLinear);

  // velocity of the robot in x, y and z coordinates
  tf::Vector3 vel = tf::quatRotate(pose, twistLinear);
  egoVel_.x = vel.x();
  egoVel_.y = vel.y();
  egoVel_.z = vel.z();
}

void CostmapToDynamicObstacles::reconfigureCB(CostmapToDynamicObstaclesConfig &config, uint32_t level)
{
  // BackgroundSubtraction Parameters
  BackgroundSubtractor::Params bgSubParams;
  bgSubParams.alpha_slow = config.alpha_slow;
  bgSubParams.alpha_fast = config.alpha_fast;
  bgSubParams.beta = config.beta;
  bgSubParams.minSepBetweenFastAndSlowFilter = config.min_sep_between_slow_and_fast_filter;
  bgSubParams.minOccupancyProbability = config.min_occupancy_probability;
  bgSubParams.maxOccupancyNeighbors = config.max_occupancy_neighbors;
  bgSubParams.morph_size = config.morph_size;
  bgSub_->updateParameters(bgSubParams);

  // BlobDetector Parameters
  BlobDetector::Params blobDetParams;
  // necessary, because blobDetParams are otherwise initialized with default values for dark blobs
  blobDetParams.filterByColor = true; // actually filterByIntensity, always true
  blobDetParams.blobColor = 255;      // Extract light blobs
  blobDetParams.thresholdStep = 256;  // Input for blobDetection is already a binary image
  blobDetParams.minThreshold = 127;
  blobDetParams.maxThreshold = 255;
  blobDetParams.minRepeatability = 1;
  blobDetParams.minDistBetweenBlobs = config.min_distance_between_blobs;
  blobDetParams.filterByArea = config.filter_by_area;
  blobDetParams.minArea = config.min_area;
  blobDetParams.maxArea = config.max_area;
  blobDetParams.filterByCircularity = config.filter_by_circularity;
  blobDetParams.minCircularity = config.min_circularity;
  blobDetParams.maxCircularity = config.max_circularity;
  blobDetParams.filterByInertia = config.filter_by_inertia;
  blobDetParams.minInertiaRatio = config.min_inertia_ratio;
  blobDetParams.maxInertiaRatio = config.max_inertia_ratio;
  blobDetParams.filterByConvexity = config.filter_by_convexity;
  blobDetParams.minConvexity = config.min_convexity;
  blobDetParams.maxConvexity = config.max_convexity;
  blobDet_->updateParameters(blobDetParams);

  // Tracking Parameters
  CTracker::Params trackingParams;
  trackingParams.dt = config.dt;
  trackingParams.dist_thresh = config.dist_thresh;
  trackingParams.max_allowed_skipped_frames = config.max_allowed_skipped_frames;
  trackingParams.max_trace_length = config.max_trace_length;
  tracker_->updateParameters(trackingParams);
}

std::vector<Point_t> CostmapToDynamicObstacles::getContour(unsigned int idx)
{
  assert(!tracker_->tracks.empty() && idx < tracker_->tracks.size());

  // contour [px] * costmapResolution [m/px] = contour [m]
  std::vector<cv::Point> contour2i = tracker_->tracks.at(idx)->getLastContour();
  std::vector<Point_t> contour3f;
  contour3f.reserve(contour2i.size());

  Point_t costmapOrigin(costmap_->getOriginX(), costmap_->getOriginY(), 0);

  for (int i = 0; i < contour2i.size(); i++)
  {
    contour3f.push_back((Point_t(contour2i.at(i).x, contour2i.at(i).y, 0.0)*costmap_->getResolution())
                        + costmapOrigin); // Shift to /map
  }

  return contour3f;
}

void CostmapToDynamicObstacles::visualize(std::string name, cv::Mat image)
{
  if (!image.empty())
  {
    cv::Mat im = image.clone();
    cv::flip(im, im, 0);
    cv::imshow(name, im);
    cv::waitKey(1);
  }
}
}
