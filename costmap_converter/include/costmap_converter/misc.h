/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016,
 *  TU Dortmund - Institute of Control Theory and Systems Engineering.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the institute nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Christoph Rösmann
 *********************************************************************/

#ifndef MISC_H_
#define MISC_H_

#include <algorithm>
#include <cmath>

namespace costmap_converter
{

/**
  * @brief Calculate the distance between a point and a straight line (with infinite length)
  * @param point generic 2D point type defining the reference point
  * @param line_pt1 generic 2D point as part of the line
  * @param line_pt2 generic 2D point as part of the line
  * @tparam Point generic point type that should provide x and y member fiels.
  * @tparam LinePoint generic point type that should provide x and y member fiels.
  * @return (minimum) euclidean distance to the line segment
  */
template <typename Point, typename LinePoint>
inline double computeDistanceToLine(const Point& point, const LinePoint& line_pt1, const LinePoint& line_pt2)
{
    double dx = line_pt2.x - line_pt1.x;
    double dy = line_pt2.y - line_pt1.y;
    
    double length = std::sqrt(dx*dx + dy*dy);
    
    if (length>0)
      return std::abs(dy * point.x - dx * point.y + line_pt2.x * line_pt1.y - line_pt2.y * line_pt1.x) / length;
  
    return std::sqrt(std::pow(point.x - line_pt1.x, 2) + std::pow(point.y - line_pt1.y, 2));  
}

  
/**
  * @brief Calculate the distance between a point and a straight line segment
  * @param point generic 2D point type defining the reference point
  * @param line_start generic 2D point type defining the start of the line
  * @param line_end generic 2D point type defining the end of the line
  * @param[out] is_inbetween write \c true, if the point is placed inbetween start and end [optional]
  * @tparam Point generic point type that should provide x and y member fiels.
  * @tparam LinePoint generic point type that should provide x and y member fiels.
  * @return (minimum) euclidean distance to the line segment
  */
template <typename Point, typename LinePoint>
inline double computeDistanceToLineSegment(const Point& point, const LinePoint& line_start, const LinePoint& line_end, bool* is_inbetween=NULL)
{
    double dx = line_end.x - line_start.x;
    double dy = line_end.y - line_start.y;
    
    double length = std::sqrt(dx*dx + dy*dy);
    
    double u = 0;
    
    if (length>0)
     u = ((point.x - line_start.x) * dx + (point.y - line_start.y)*dy) / length;
    
    if (is_inbetween)
      *is_inbetween = (u>=0 && u<=1);
  
    if (u <= 0)
      return std::sqrt(std::pow(point.x-line_start.x,2) + std::pow(point.y-line_start.y,2));
    
    if (u >= 1)
      return std::sqrt(std::pow(point.x-line_end.x,2) + std::pow(point.y-line_end.y,2));
    
    return std::sqrt(std::pow(point.x - (line_start.x+u*dx) ,2) + std::pow(point.y - (line_start.y+u*dy),2));
}
  

/**
  * @brief Calculate the distance between two 2d points
  * @param pt1 generic 2D point
  * @param pt2 generic 2D point
  * @tparam Point1 generic point type that should provide x and y member fiels.
  * @tparam Point2 generic point type that should provide x and y member fiels.
  * @return euclidean distance to the line segment
  */
template <typename Point1, typename Point2>  
inline double norm2d(const Point1& pt1, const Point2& pt2)
{
  return std::sqrt( std::pow(pt2.x - pt1.x, 2) + std::pow(pt2.y - pt1.y, 2)  );
}

/**
  * @brief Check if two points are approximately defining the same one
  * @param pt1 generic 2D point
  * @param pt2 generic 2D point
  * @param threshold define the minimum threshold |pt1.x-pt2.y| < tresh && |pt1.y-pt2.y| < tresh
  * @tparam Point1 generic point type that should provide x and y member fiels.
  * @tparam Point2 generic point type that should provide x and y member fiels.
  * @return \c true, if similar, \c false otherwise
  */
template <typename Point1, typename Point2>  
inline bool isApprox2d(const Point1& pt1, const Point2& pt2, double threshold)
{
  return ( std::abs(pt2.x-pt1.x)<threshold && std::abs(pt2.y-pt1.y)<threshold );
}


  
} //end namespace teb_local_planner

#endif /* MISC_H_ */
