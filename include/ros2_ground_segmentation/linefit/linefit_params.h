/*
 *  Copyright (c) 2023, MAP IV, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef ROS2_LINEFIT_PARAMS_H
#define ROS2_LINEFIT_PARAMS_H

struct LinefitSegParams {
  LinefitSegParams()
    : r_min(0.1), r_max(20), bins_num(100), segs_num(360), max_dist_to_line(0.15), max_slope(1), threads_num(4),
      max_error(0.01), long_thres(2.0), max_long_height(0.1), max_start_height(0.2), sensor_height(0.5),
      line_search_angle(0.2) {
  }
/*
 *
 *  r_min: 2
    r_max: 100
    bins_num: 300
    segs_num: 360
    threads_num: 4
    max_slope: 0.3
    max_dist_to_line: 0.1
    long_thres: 5
    max_long_height: 0.1
    max_start_height: 0.5
    line_search_angle: 0.5
    max_fit_error: 0.02
 */
  // Minimum range of segmentation.
  double r_min;
  // Maximum range of segmentation.
  double r_max;
  // Number of radial bins.
  int bins_num;
  // Number of angular segments.
  int segs_num;
  // Maximum distance to a ground line to be classified as ground.
  double max_dist_to_line;
  // Max slope to be considered ground line.
  double max_slope;
  // Max error for line fit.
  double max_error;
  // Distance at which points are considered far from each other.
  double long_thres;
  // Maximum slope for
  double max_long_height;
  // Maximum heigh of starting line to be labelled ground.
  double max_start_height;
  // Height of sensor above ground.
  double sensor_height;
  // How far to search for a line in angular direction [rad].
  double line_search_angle;
  // Number of threads.
  int threads_num;
};

#endif //ROS2_LINEFIT_PARAMS_H
