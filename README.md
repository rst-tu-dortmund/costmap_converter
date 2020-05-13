costmap_converter ROS Package
=============================

A ros package that includes plugins and nodes to convert occupied costmap2d cells to primitive types

Build status of the *master* branch:
- ROS Buildfarm Noetic: [![Noetic Build Status](http://build.ros.org/buildStatus/icon?job=Ndev__costmap_converter__ubuntu_focal_amd64)](http://build.ros.org/job/Ndev__costmap_converter__ubuntu_focal_amd64/)
- ROS Buildfarm Melodic: [![Melodic Build Status](http://build.ros.org/buildStatus/icon?job=Mdev__costmap_converter__ubuntu_bionic_amd64)](http://build.ros.org/job/Mdev__costmap_converter__ubuntu_bionic_amd64/)


### Contributors

- Christoph Rösmann
- Franz Albers (*CostmapToDynamicObstacles* plugin)
- Otniel Rinaldo


### License

The *costmap_converter* package is licensed under the BSD license.
It depends on other ROS packages, which are listed in the package.xml. They are also BSD licensed.

Some third-party dependencies are included that are licensed under different terms:
 - *MultitargetTracker*, GNU GPLv3, https://github.com/Smorodov/Multitarget-tracker
   (partially required for the *CostmapToDynamicObstacles* plugin)

All packages included are distributed in the hope that they will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the licenses for more details.



