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

#include <cmath>

#include "ros2_ground_segmentation/ray/ray_ground_seg.h"

RayGroundSeg::RayGroundSeg(const RayGroundParams& rayground_params ) {
  setParameters(rayground_params);
}

void RayGroundSeg::segmentGround(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr &in_cloud_points,
                                 pcl::PointCloud<pcl::PointXYZI>::Ptr out_groundless_points,
                                 pcl::PointCloud<pcl::PointXYZI>::Ptr out_ground_points) {
  const std::lock_guard<std::mutex> lock(params_mutex_);
  //remove points above and below certain point
  pcl::PointCloud<pcl::PointXYZI>::Ptr clipped_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
  ClipCloud(in_cloud_points,
            params_.clipping_height,
            params_.pointcloud_min_z,
            clipped_cloud_ptr);

  //remove closer points than a threshold
  pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
  RemovePointsUpTo(clipped_cloud_ptr,
                   params_.min_point_distance,
                   filtered_cloud_ptr);

    PointCloudXYZIRTColor organized_points;
  std::vector<pcl::PointIndices> radial_division_indices;
  std::vector<pcl::PointIndices> closest_indices;
  std::vector<PointCloudXYZIRTColor> radial_ordered_clouds;

  radial_dividers_num_ = ceil(360 / params_.radial_divider_angle);

  ConvertXYZIToRTZColor(filtered_cloud_ptr,
                        organized_points,
                        radial_division_indices,
                        radial_ordered_clouds);

  pcl::PointIndices ground_indices, no_ground_indices;

  ClassifyPointCloud(radial_ordered_clouds, ground_indices, no_ground_indices);

  ExtractPointsIndices(filtered_cloud_ptr,
                       ground_indices,
                       out_ground_points,
                       out_groundless_points);

  if (params_.outlier_filter)
  {
    //clean cloud
    pcl::PointCloud<pcl::PointXYZI>::Ptr no_ground_clean_cloud_ptr(new pcl::PointCloud <pcl::PointXYZI>);
    pcl::RadiusOutlierRemoval<pcl::PointXYZI> outlier_rem;

    outlier_rem.setInputCloud(out_groundless_points);
    outlier_rem.setRadiusSearch(0.5);
    outlier_rem.setMinNeighborsInRadius (1);
    outlier_rem.filter (*no_ground_clean_cloud_ptr);

    out_groundless_points = no_ground_clean_cloud_ptr;
  }
}

void RayGroundSeg::setParameters(const RayGroundParams& rayground_params ) {
  const std::lock_guard<std::mutex> lock(params_mutex_);
  params_ = rayground_params;
}

void RayGroundSeg::ClipCloud(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr in_cloud_ptr,
                                  double in_max_height, double in_min_height,
                                  pcl::PointCloud<pcl::PointXYZI>::Ptr out_clipped_cloud_ptr)
{
  pcl::ExtractIndices <pcl::PointXYZI> extractor;
  extractor.setInputCloud(in_cloud_ptr);
  pcl::PointIndices indices;

#pragma omp for
  for (size_t i = 0; i < in_cloud_ptr->points.size(); i++)
  {
    if (in_cloud_ptr->points[i].z > in_max_height || in_cloud_ptr->points[i].z < in_min_height)
    {
      indices.indices.push_back(i);
    }
  }
  extractor.setIndices(std::make_shared<pcl::PointIndices>(indices));
  extractor.setNegative(true);//true removes the indices, false leaves only the indices
  extractor.filter(*out_clipped_cloud_ptr);
}


void RayGroundSeg::ExtractPointsIndices(const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_ptr,
                                             const pcl::PointIndices& in_indices,
                                             pcl::PointCloud<pcl::PointXYZI>::Ptr out_only_indices_cloud_ptr,
                                             pcl::PointCloud<pcl::PointXYZI>::Ptr out_removed_indices_cloud_ptr)
{
  pcl::ExtractIndices<pcl::PointXYZI> extract_ground;
  extract_ground.setInputCloud (in_cloud_ptr);
  extract_ground.setIndices(std::make_shared<pcl::PointIndices>(in_indices));

  extract_ground.setNegative(false);//true removes the indices, false leaves only the indices
  extract_ground.filter(*out_only_indices_cloud_ptr);

  extract_ground.setNegative(true);//true removes the indices, false leaves only the indices
  extract_ground.filter(*out_removed_indices_cloud_ptr);
}

void RayGroundSeg::RemovePointsUpTo(const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_ptr,
                                         double in_min_distance,
                                         pcl::PointCloud<pcl::PointXYZI>::Ptr out_filtered_cloud_ptr)
{
  pcl::ExtractIndices <pcl::PointXYZI> extractor;
  extractor.setInputCloud(in_cloud_ptr);
  pcl::PointIndices indices;

#pragma omp for
  for (size_t i = 0; i < in_cloud_ptr->points.size(); i++)
  {
    if (std::sqrt(in_cloud_ptr->points[i].x * in_cloud_ptr->points[i].x +
             in_cloud_ptr->points[i].y * in_cloud_ptr->points[i].y)
        < in_min_distance)
    {
      indices.indices.push_back(i);
    }
  }
  extractor.setIndices(std::make_shared<pcl::PointIndices>(indices));
  extractor.setNegative(true);//true removes the indices, false leaves only the indices
  extractor.filter(*out_filtered_cloud_ptr);
}

/*!
 *
 * @param[in] in_cloud Input Point Cloud to be organized in radial segments
 * @param[out] out_organized_points Custom Point Cloud filled with XYZRTZColor data
 * @param[out] out_radial_divided_indices Indices of the points in the original cloud for each radial segment
 * @param[out] out_radial_ordered_clouds Vector of Points Clouds, each element will contain the points ordered
 */
void RayGroundSeg::ConvertXYZIToRTZColor(const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud,
                                              PointCloudXYZIRTColor& out_organized_points,
                                              std::vector<pcl::PointIndices>& out_radial_divided_indices,
                                              std::vector<PointCloudXYZIRTColor>& out_radial_ordered_clouds)
{
  out_organized_points.resize(in_cloud->points.size());
  out_radial_divided_indices.clear();
  out_radial_divided_indices.resize(radial_dividers_num_);
  out_radial_ordered_clouds.resize(radial_dividers_num_);

  for (size_t i = 0; i < in_cloud->points.size(); i++)
  {
    PointXYZIRTColor new_point{};
    auto radius = (float) sqrt(
      in_cloud->points[i].x * in_cloud->points[i].x
      + in_cloud->points[i].y * in_cloud->points[i].y
    );
    auto theta = (float) atan2(in_cloud->points[i].y, in_cloud->points[i].x) * 180 / M_PI;
    if (theta < 0)
    {
      theta += 360;
    }

    auto radial_div = (size_t) floor(theta / params_.radial_divider_angle);
    auto concentric_div = (size_t) floor(fabs(radius / params_.concentric_divider_distance));

    new_point.point.x = in_cloud->points[i].x;
    new_point.point.y = in_cloud->points[i].y;
    new_point.point.z = in_cloud->points[i].z;

    new_point.radius = radius;
    new_point.theta = theta;
    new_point.radial_div = radial_div;
    new_point.concentric_div = concentric_div;
    new_point.original_index = i;

    out_organized_points[i] = new_point;

    //radial divisions
    out_radial_divided_indices[radial_div].indices.push_back(i);

    out_radial_ordered_clouds[radial_div].push_back(new_point);

  }//end for

  //order radial points on each division
#pragma omp for
  for (size_t i = 0; i < radial_dividers_num_; i++)
  {
    std::sort(out_radial_ordered_clouds[i].begin(), out_radial_ordered_clouds[i].end(),
              [](const PointXYZIRTColor &a, const PointXYZIRTColor &b)
              {
                return a.radius < b.radius;
              });
  }
}

void RayGroundSeg::ClassifyPointCloud(std::vector<PointCloudXYZIRTColor>& in_radial_ordered_clouds,
                                           pcl::PointIndices& out_ground_indices,
                                           pcl::PointIndices& out_no_ground_indices)
{
  out_ground_indices.indices.clear();
  out_no_ground_indices.indices.clear();
#pragma omp for
  for (size_t i = 0; i < in_radial_ordered_clouds.size(); i++)//sweep through each radial division
  {
    float prev_radius = 0.f;
    float prev_height = -params_.sensor_height;
    bool prev_ground = false;
    bool current_ground = false;
    for (size_t j = 0; j < in_radial_ordered_clouds[i].size(); j++)//loop through each point in the radial div
    {
      float points_distance = in_radial_ordered_clouds[i][j].radius - prev_radius;
      float height_threshold = std::tan(DEG2RAD(params_.local_max_slope)) * points_distance;
      float current_height = in_radial_ordered_clouds[i][j].point.z;
      float general_height_threshold = std::tan(DEG2RAD(params_.general_max_slope)) * in_radial_ordered_clouds[i][j].radius;

      //for points which are very close causing the height threshold to be tiny, set a minimum value
      if (height_threshold < params_.min_height_threshold)
      {
        height_threshold = params_.min_height_threshold;
      }
      //only check points which radius is larger than the concentric_divider
      if (points_distance < params_.concentric_divider_distance)
      {
        current_ground = prev_ground;
      } else
      {
        //check current point height against the LOCAL threshold (previous point)
        if (current_height <= (prev_height + height_threshold)
            && current_height >= (prev_height - height_threshold)
          )
        {
          //Check again using general geometry (radius from origin) if previous points wasn't ground
          if (!prev_ground)
          {
            if (current_height <= (-params_.sensor_height + general_height_threshold)
                && current_height >= (-params_.sensor_height - general_height_threshold))
            {
              current_ground = true;
            } else
            {
              current_ground = false;
            }
          } else
          {
            current_ground = true;
          }
        } else
        {
          //check if previous point is too far from previous one, if so classify again
          if (points_distance > params_.reclass_distance_threshold &&
              (current_height <= (-params_.sensor_height + height_threshold)
               && current_height >= (-params_.sensor_height - height_threshold))
            )
          {
            current_ground = true;
          } else
          {
            current_ground = false;
          }
        }
      }//end larger than concentric_divider

      if (current_ground)
      {
        out_ground_indices.indices.push_back(in_radial_ordered_clouds[i][j].original_index);
        prev_ground = true;
      } else
      {
        out_no_ground_indices.indices.push_back(in_radial_ordered_clouds[i][j].original_index);
        prev_ground = false;
      }

      prev_radius = in_radial_ordered_clouds[i][j].radius;
      prev_height = in_radial_ordered_clouds[i][j].point.z;
    }
  }
}
