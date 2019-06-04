/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017
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
 * Notes:
 * The following code makes use of the OpenCV library.
 * OpenCV is licensed under the terms of the 3-clause BSD License.
 *
 * Authors: Franz Albers, Christoph Rösmann
 *********************************************************************/

#ifndef BACKGROUNDSUBTRACTOR_H_
#define BACKGROUNDSUBTRACTOR_H_

#include <cv_bridge/cv_bridge.h>

/**
 * @class BackgroundSubtractor
 * @brief Perform background subtraction to extract the "moving" foreground
 *
 * This class is based on OpenCV's background subtraction class cv::BackgroundSubtractor.
 * It has been modified in order to incorporate a specialized bandpass filter.
 *
 * See http://docs.opencv.org/3.2.0/d7/df6/classcv_1_1BackgroundSubtractor.html for the original class.
 */
class BackgroundSubtractor
{
public:
  struct Params
  {
    double alpha_slow; //!< Filter constant (learning rate) of the slow filter part
    double alpha_fast; //!< Filter constant (learning rate) of the fast filter part
    double beta;
    double min_sep_between_fast_and_slow_filter;
    double min_occupancy_probability;
    double max_occupancy_neighbors;
    int morph_size;
  };

  //! Constructor that accepts a specific parameter configuration
  BackgroundSubtractor(const Params& parameters);

  /**
   * @brief Computes a foreground mask
   * @param[in]  image    Next video frame
   * @param[out] fg_mask  Foreground mask as an 8-bit binary image
   * @param[in]  shift_x  Translation along the x axis between the current and previous image
   * @param[in]  shift_y  Translation along the y axis between the current and previous image
   */
  void apply(const cv::Mat& image, cv::Mat& fg_mask, int shift_x = 0, int shift_y = 0);

  /**
   * @brief OpenCV Visualization
   * @param name  Id/name of the opencv window
   * @param image Image to be visualized
   */
  void visualize(const std::string& name, const cv::Mat& image);

  /**
   * @brief Export vector of matrices to yaml file
   * @remarks This method is intended for debugging purposes
   * @param filename   Desired filename including path and excluding file suffix
   * @param mat_vec    Vector of cv::Mat to be exported
   */
  void writeMatToYAML(const std::string& filename, const std::vector<cv::Mat>& mat_vec);

  //! Update internal parameters
  void updateParameters(const Params& parameters);

private:
  //! Transform/shift all internal matrices/grids according to a given translation vector
  void transformToCurrentFrame(int shift_x, int shift_y);

  cv::Mat occupancy_grid_fast_;
  cv::Mat occupancy_grid_slow_;
  cv::Mat current_frame_;

  int previous_shift_x_;
  int previous_shift_y_;

  Params params_;
};

#endif // BACKGROUNDSUBTRACTOR_H_
