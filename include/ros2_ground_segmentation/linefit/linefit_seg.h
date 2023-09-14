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

#ifndef LINEFIT_SEG_H
#define LINEFIT_SEG_H

#include <chrono>
#include <cmath>
#include <iostream>
#include <list>
#include <memory>
#include <mutex>
#include <thread>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <yaml-cpp/yaml.h>

#include <ros2_ground_segmentation/point_type.h>
#include <ros2_ground_segmentation/linefit/segment.h>
#include <ros2_ground_segmentation/linefit/linefit_params.h>

class LinefitGroundSeg {
  enum class PointCategory {
    OBJECT = 0,
    GROUND = 1,
  };
public:
  LinefitGroundSeg(const LinefitSegParams& linefitseg_params );

  void setParameters(const LinefitSegParams& linefitseg_params);

  void segmentGround(const pcl::PointCloud<PointXYZIR>::ConstPtr &in_cloud_msg,
                     pcl::PointCloud<pcl::PointXYZI> &out_groundless_points,
                     pcl::PointCloud<pcl::PointXYZI> &out_ground_points);
  void displayParams() const;
private:
  std::mutex params_mutex_;
  LinefitSegParams params_;
  // Access with segments_[segment][bin].
  std::vector<Segment> segments_;
  // Bin index of every point.
  std::vector<std::pair<int, int>> bin_index_;
  // 2D coordinates (d, z) of every point in its respective segment.
  std::vector<Bin::MinZPoint> segment_coordinates_;

  void initializeSegments();

  void segment(const pcl::PointCloud<pcl::PointXYZ> &cloud, std::vector<PointCategory> *labels);

  void assignCluster(std::vector<PointCategory> *labels);

  void assignClusterThread(const unsigned int &start_index, const unsigned int &end_index,
                           std::vector<PointCategory> *labels);

  void insertPoints(const pcl::PointCloud<pcl::PointXYZ> &cloud);

  void insertionThread(const pcl::PointCloud<pcl::PointXYZ> &cloud, const size_t start_index, const size_t end_index);

  void lineFitting();

  void lineFitThread(const unsigned int start_index, const unsigned int end_index, std::mutex *lines_mutex);
};

#endif  // LINEFIT_SEG_H
