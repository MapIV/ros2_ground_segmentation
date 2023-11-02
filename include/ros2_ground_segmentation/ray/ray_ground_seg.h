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

#ifndef ROS2_RAYGROUND_SEG_H
#define ROS2_RAYGROUND_SEG_H

#include <chrono>
#include <cmath>
#include <iostream>
#include <list>
#include <memory>
#include <mutex>
#include <thread>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/radius_outlier_removal.h>

#include "ros2_ground_segmentation/point_type.h"
#include "ros2_ground_segmentation/ray/ray_ground_params.h"

struct PointXYZIRTColor
{
  pcl::PointXYZI point;

  float radius;       //cylindric coords on XY Plane
  float theta;        //angle deg on XY plane

  size_t radial_div;  //index of the radial divsion to which this point belongs to
  size_t concentric_div;//index of the concentric division to which this points belongs to

//  std::uint8_t red;         //Red component  [0-255]
//  std::uint8_t green;       //Green Component[0-255]
//  std::uint8_t blue;        //Blue component [0-255]

  size_t original_index; //index of this point in the source pointcloud
};
typedef std::vector<PointXYZIRTColor> PointCloudXYZIRTColor;

class RayGroundSeg {
public:
  RayGroundSeg(const RayGroundParams& rayground_params );

  void setParameters(const RayGroundParams& rayground_params);

  void segmentGround(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr &in_cloud_msg,
                     pcl::PointCloud<pcl::PointXYZI>::Ptr out_groundless_points,
                     pcl::PointCloud<pcl::PointXYZI>::Ptr out_ground_points);

private:
  std::mutex params_mutex_;
  RayGroundParams params_;
  size_t radial_dividers_num_;

  /*!
 * Removes the points higher than a threshold
 * @param in_cloud_ptr PointCloud to perform Clipping
 * @param in_clip_height Maximum allowed height in the cloud
 * @param out_clipped_cloud_ptr Resultung PointCloud with the points removed
 */
  static void ClipCloud(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr in_cloud_ptr,
                 double in_max_height, double in_min_height,
                 pcl::PointCloud<pcl::PointXYZI>::Ptr out_clipped_cloud_ptr);
  /*!
 * Removes points up to a certain distance in the XY Plane
 * @param in_cloud_ptr Input PointCloud
 * @param in_min_distance Minimum valid distance, points closer than this will be removed.
 * @param out_filtered_cloud_ptr Resulting PointCloud with the invalid points removed.
 */
  static void RemovePointsUpTo(const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_ptr,
                        double in_min_distance,
                        pcl::PointCloud<pcl::PointXYZI>::Ptr out_filtered_cloud_ptr);

  /*!
 * Removes points up to a certain distance in the XY Plane
 * @param in_cloud_ptr Input PointCloud
 * @param in_min_distance Minimum valid distance, points closer than this will be removed.
 * @param out_filtered_cloud_ptr Resulting PointCloud with the invalid points removed.
 */
  static void FilterByIntensity(const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_ptr,
                               int min_intensity,
                               double max_distance,
                               pcl::PointCloud<pcl::PointXYZI>::Ptr out_filtered_cloud_ptr);

  /*!
 * Returns the resulting complementary PointCloud, one with the points kept and the other removed as indicated
 * in the indices
 * @param in_cloud_ptr Input PointCloud to which the extraction will be performed
 * @param in_indices Indices of the points to be both removed and kept
 * @param out_only_indices_cloud_ptr Resulting PointCloud with the indices kept
 * @param out_removed_indices_cloud_ptr Resulting PointCloud with the indices removed
 */
  static void ExtractPointsIndices(const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_ptr,
                            const pcl::PointIndices& in_indices,
                            pcl::PointCloud<pcl::PointXYZI>::Ptr out_only_indices_cloud_ptr,
                            pcl::PointCloud<pcl::PointXYZI>::Ptr out_removed_indices_cloud_ptr);

  /*!
 *
 * @param[in] in_cloud Input Point Cloud to be organized in radial segments
 * @param[out] out_organized_points Custom Point Cloud filled with XYZRTZColor data
 * @param[out] out_radial_divided_indices Indices of the points in the original cloud for each radial segment
 * @param[out] out_radial_ordered_clouds Vector of Points Clouds, each element will contain the points ordered
 */
  void ConvertXYZIToRTZColor(const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud,
                             PointCloudXYZIRTColor& out_organized_points,
                             std::vector<pcl::PointIndices>& out_radial_divided_indices,
                             std::vector<PointCloudXYZIRTColor>& out_radial_ordered_clouds);

  /*!
 * Classifies Points in the PointCoud as Ground and Not Ground
 * @param in_radial_ordered_clouds Vector of an Ordered PointsCloud ordered by radial distance from the origin
 * @param out_ground_indices Returns the indices of the points classified as ground in the original PointCloud
 * @param out_no_ground_indices Returns the indices of the points classified as not ground in the original PointCloud
 */
  void ClassifyPointCloud(std::vector<PointCloudXYZIRTColor>& in_radial_ordered_clouds,
                          pcl::PointIndices& out_ground_indices,
                          pcl::PointIndices& out_no_ground_indices);
};

#endif //ROS2_RAYGROUND_SEG_H
