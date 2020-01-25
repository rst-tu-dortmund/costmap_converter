^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package costmap_converter
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Fixed ament plugin export
* Revert release-mode for cmake build
* Contributors: Christoph Rösmann

0.1.0 (2020-01-23)
------------------
* Port to ROS2 (thanks to Vinnam Kim and stevemacenski)
* Messages moved to a separate package
* Contributors: Christoph Rösmann, Vinnam Kim, stevemacenski

0.0.12 (2019-12-02)
-------------------
* CostmapToPolygons: Simplification of the polygon by Douglas-Peucker algorithm (reduces the density of points in the polygon).
* Bugfixes
* Contributors: Rainer Kümmerle

0.0.11 (2019-10-26)
-------------------
* rostest integration to avoid running a roscore separately for unit testing
* Contributors: Christoph Rösmann

0.0.10 (2019-10-26)
-------------------
* Runtime improvements for CostmapToPolygonsDBSMCCH (`#12 <https://github.com/rst-tu-dortmund/costmap_converter/issues/12>`_)
  * Grid lookup for regionQuery
  * use a grid structure for looking up nearest neighbors
  * parameters in a struct
  * guard the parameters by drawing a copy from dynamic reconfigure
  * Adding some test cases for regionQuery and dbScan
  * Avoid computing sqrt at the end of convexHull2
  * Add doxygen comments for the neighbor lookup
  * Change the param read to one liners
  * Add test on empty map for dbScan
* Contributors: Rainer Kümmerle

0.0.9 (2018-05-28)
------------------
* Moved plugin loader for static costmap conversion to BaseCostmapToDynamicObstacles.
  The corresponding ROS parameter `static_converter_plugin` is now defined in the CostmapToDynamicObstacles namespace. 
* Contributors: Christoph Rösmann

0.0.8 (2018-05-17)
------------------
* Standalone converter subscribes now to costmap updates. Fixes `#1 <https://github.com/rst-tu-dortmund/costmap_converter/issues/1>`_
* Adds radius field for circular obstacles to ObstacleMsg
* Stacked costmap conversion (`#7 <https://github.com/rst-tu-dortmund/costmap_converter/issues/7>`_).
  E.g., it is now possible combine a dynamic obstacle and static obstacle converter plugin.
* Contributors: Christoph Rösmann, Franz Albers

0.0.7 (2017-09-20)
------------------
* Fixed some compilation issues (C++11 compiler flags and opencv2 on indigo/jade).
* Dynamic obstacle plugin: obstacle velocity is now published for both x and y coordinates rather than the absolute value

0.0.6 (2017-09-18)
------------------
* This pull request adds the costmap to dynamic obstacles plugin (written by Franz Albers).
  It detects the dynamic foreground of the costmap (based on the temporal evolution of the costmap)
  including blobs representing the obstacles. Furthermore, Kalman-based tracking is applied to estimate
  the current velocity for each obstacle.
  **Note, this plugin is still experimental.**
* New message types are introduced: costmap\_converter::ObstacleMsg and costmap\_converter::ObstacleArrayMsg.
  These types extend the previous polygon representation by additional velocity, orientation and id information.
* The API has been extended to provide obstacles via the new ObstacleArrayMsg type instead of vector of polygons.
* Contributors: Franz Albers, Christoph Rösmann

0.0.5 (2016-02-01)
------------------
* Major changes regarding the line detection based on the convex hull
  (it should be much more robust now).
* Concave hull plugin added.
* The cluster size can now be limited from above using a specific parameter.
  This implicitly avoids large clusters forming a 'L' or 'U'.
* All parameters can now be adjusted using dynamic_reconfigure (rqt_reconfigure).
* Some parameter names changed.
* Line plugin based on ransac: line inliers must now be placed inbetween the start and end of a line.

0.0.4 (2016-01-11)
------------------
* Fixed conversion from map to world coordinates if the costmap is not quadratic.

0.0.3 (2015-12-23)
------------------
* The argument list of the initialize method requires a nodehandle from now on. This facilitates the handling of parameter namespaces for multiple instantiations of the plugin.
* This change is pushed immediately as a single release to avoid API breaks (since version 0.0.2 is not on the official repos up to now).

0.0.2 (2015-12-22)
------------------
* Added a plugin for converting the costmap to lines using ransac

0.0.1 (2015-12-21)
------------------
* First release of the package including a pluginlib interface, two plugins (costmap to polygons and costmap to lines) and a standalone conversion node.

