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

#ifndef ROS2_RAYGROUND_PARAMS_H
#define ROS2_RAYGROUND_PARAMS_H

struct RayGroundParams {
  RayGroundParams()
  : general_max_slope(3.0), local_max_slope(5.0), sensor_height(0.0),
  radial_divider_angle(0.1), concentric_divider_distance(0.01), min_height_threshold(0.05),
  clipping_height(0.2), pointcloud_min_z(-0.1), min_point_distance(0.5),
  reclass_distance_threshold(0.2), outlier_filter(false), min_outlier_filter_neighbors(5),
  min_outlier_filter_radius(0.5), intensity_filter(false), min_intensity(4), max_intensity_distance(20.0) {
  }
  double general_max_slope;
  double local_max_slope;
  double sensor_height;
  double radial_divider_angle;
  double concentric_divider_distance;
  double min_height_threshold;
  double clipping_height;
  double pointcloud_min_z;
  double min_point_distance;
  double reclass_distance_threshold;
  bool outlier_filter;
  int min_outlier_filter_neighbors;
  double min_outlier_filter_radius;
  bool intensity_filter;
  int min_intensity;
  double max_intensity_distance;
};

#endif //ROS2_RAYGROUND_PARAMS_H
