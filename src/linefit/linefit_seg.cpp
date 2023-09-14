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

#include <ros2_ground_segmentation/linefit/linefit_seg.h>

[[maybe_unused]] double selectMax(const double x, const double y, const double z) {
  double max = (x > y) ? x : y;
  max = (max > z) ? max : z;

  return max;
}

LinefitGroundSeg::LinefitGroundSeg(const LinefitSegParams& linefitseg_params ) {
  setParameters(linefitseg_params);
}

void LinefitGroundSeg::setParameters(const LinefitSegParams& linefitseg_params ) {
  const std::lock_guard<std::mutex> lock(params_mutex_);
  params_ = linefitseg_params;
}

void LinefitGroundSeg::segmentGround(const pcl::PointCloud<PointXYZIR>::ConstPtr &input_cloud,
                                     pcl::PointCloud<pcl::PointXYZI> &out_groundless_cloud,
                                     pcl::PointCloud<pcl::PointXYZI> &out_ground_cloud) {
  pcl::PointCloud<pcl::PointXYZ> xyz_cloud;
  for (const auto& p: input_cloud->points) {
    pcl::PointXYZ xyz;
    xyz.x = p.x;
    xyz.y = p.y;
    xyz.z = p.z;

    xyz_cloud.points.push_back(xyz);
  }
  const std::lock_guard<std::mutex> lock(params_mutex_);

  initializeSegments();

  std::vector<PointCategory> labels;
  segment(xyz_cloud, &labels);

  out_ground_cloud.reserve(input_cloud->size());
  out_groundless_cloud.reserve(input_cloud->size());

  for (size_t i = 0; i < input_cloud->size(); ++i) {
    pcl::PointXYZI p;
    p.x = input_cloud->points[i].x;
    p.y = input_cloud->points[i].y;
    p.z = input_cloud->points[i].z;
    p.intensity = input_cloud->points[i].intensity;

    if (labels[i] == PointCategory::GROUND) {
      out_ground_cloud.emplace_back(p);
    } else {
      out_groundless_cloud.emplace_back(p);
    }
  }
}

void LinefitGroundSeg::displayParams() const {
  std::cout << "Loading LinefitGroundSeg params" << std::endl;

  // Print the parameters
  std::cout << "Number of Bins: " << params_.bins_num << '\n';
  std::cout << "Number of Segments: " << params_.segs_num << '\n';
  std::cout << "Max distance to line: " << params_.max_dist_to_line << '\n';
  std::cout << "Max slope: " << params_.max_slope << '\n';
  std::cout << "Long threshold: " << params_.long_thres << '\n';
  std::cout << "Max long height: " << params_.max_long_height << '\n';
  std::cout << "Max start height: " << params_.max_start_height << '\n';
  std::cout << "Sensor height: " << params_.sensor_height << '\n';
  std::cout << "Line search angle: " << params_.line_search_angle << '\n';
  std::cout << "Number of threads: " << params_.threads_num << '\n';
  std::cout << "Root min: " << params_.r_min << '\n';
  std::cout << "Root max: " << params_.r_max << '\n';
  std::cout << "Max error: " << params_.max_error << '\n';

}

void LinefitGroundSeg::initializeSegments() {
  segments_.clear();

  // Fill all segments with empty
  for (size_t i = 0; i < params_.segs_num; i++) {
    segments_.emplace_back(params_);
  }
}

void LinefitGroundSeg::segment(const pcl::PointCloud<pcl::PointXYZ> &cloud, std::vector<PointCategory> *labels) {
  labels->clear();
  labels->resize(cloud.size(), PointCategory::OBJECT);
  bin_index_.resize(cloud.size());
  segment_coordinates_.resize(cloud.size());

  insertPoints(cloud);
  lineFitting();
  assignCluster(labels);
}

void LinefitGroundSeg::lineFitting() {
  std::mutex line_mutex;
  std::vector<std::thread> thread_vec(params_.threads_num);

  for (int i = 0; i < params_.threads_num; i++) {
    const unsigned int start_index = params_.segs_num / params_.threads_num * i;
    const unsigned int end_index = params_.segs_num / params_.threads_num * (i + 1);
    thread_vec[i] = std::thread(&LinefitGroundSeg::lineFitThread, this, start_index, end_index, &line_mutex);
  }

  // Wait for the child threads
  for (auto & it : thread_vec) {
    it.join();
  }
}

void LinefitGroundSeg::lineFitThread(const unsigned int start_index, const unsigned int end_index,
                                     std::mutex *lines_mutex) {
  for (unsigned int i = start_index; i < end_index; ++i) {
    segments_[i].fitSegmentLines();
  }
}

void LinefitGroundSeg::assignCluster(std::vector<PointCategory> *labels) {
  std::vector<std::thread> thread_vec(params_.threads_num);
  const size_t cloud_size = labels->size();

  for (unsigned int i = 0; i < params_.threads_num; ++i) {
    const unsigned int start_index = cloud_size / params_.threads_num * i;
    const unsigned int end_index = cloud_size / params_.threads_num * (i + 1);
    thread_vec[i] = std::thread(&LinefitGroundSeg::assignClusterThread, this, start_index, end_index, labels);
  }

  // Wait for the child threads
  for (auto & it : thread_vec) {
    it.join();
  }
}

void LinefitGroundSeg::assignClusterThread(const unsigned int &start_index, const unsigned int &end_index,
                                           std::vector<PointCategory> *labels) {
  const double segment_step = 2 * M_PI / params_.segs_num;

  for (unsigned int i = start_index; i < end_index; ++i) {
    Bin::MinZPoint point_2d = segment_coordinates_[i];
    const int segment_index = bin_index_[i].first;

    if (segment_index >= 0) {
      double distance = segments_[segment_index].verticalDistanceToLine(point_2d.d, point_2d.z);

      // Search neighboring segments.
      int steps = 1;
      while (distance == -1 && steps * segment_step < params_.line_search_angle) {
        int index_up = segment_index + steps;
        while (index_up >= params_.segs_num)
          index_up -= params_.segs_num;

        int index_down = segment_index - steps;
        while (index_down < 0)
          index_down += params_.segs_num;

        // Get distance to neighboring lines.
        const double distance_up = segments_[index_up].verticalDistanceToLine(point_2d.d, point_2d.z);
        const double distance_down = segments_[index_down].verticalDistanceToLine(point_2d.d, point_2d.z);

        // Select the max distance
        distance = (distance > distance_up) ? distance : distance_up;
        distance = (distance > distance_down) ? distance : distance_down;

        steps++;
      }

      if (distance < params_.max_dist_to_line && distance != -1) {
        labels->at(i) = PointCategory::GROUND;
      }
    }
  }
}

void LinefitGroundSeg::insertPoints(const pcl::PointCloud<pcl::PointXYZ> &cloud) {
  std::vector<std::thread> threads(params_.threads_num);
  const size_t points_per_thread = cloud.size() / params_.threads_num;

  // Launch threads.
  for (unsigned int i = 0; i < params_.threads_num - 1; ++i) {
    const size_t start_index = i * points_per_thread;
    const size_t end_index = (i + 1) * points_per_thread;
    threads[i] = std::thread(&LinefitGroundSeg::insertionThread, this, cloud, start_index, end_index);
  }

  // Launch last thread which might have more points than others.
  const size_t start_index = (params_.threads_num - 1) * points_per_thread;
  const size_t end_index = cloud.size();

  threads[params_.threads_num - 1] =
    std::thread(&LinefitGroundSeg::insertionThread, this, cloud, start_index, end_index);

  // Wait for threads to finish.
  for (auto & thread : threads) {
    thread.join();
  }
}

void LinefitGroundSeg::insertionThread(const pcl::PointCloud<pcl::PointXYZ> &cloud, const size_t start_index,
                                       const size_t end_index) {
  const double segment_step = 2 * M_PI / params_.segs_num;
  const double bin_step = (params_.r_max - params_.r_min) / params_.bins_num;
  for (unsigned int i = start_index; i < end_index; ++i) {
    pcl::PointXYZ point(cloud[i]);
    const double range_square = point.x * point.x + point.y * point.y;
    const double range = std::sqrt(range_square);

    if (params_.r_min < range && range < params_.r_max) {
      const double angle = std::atan2(point.y, point.x);
      const unsigned int bin_index = (range - params_.r_min) / bin_step;
      unsigned int segment_index = (angle + M_PI) / segment_step;

      // Guard for when angle is pi
      segment_index = (segment_index == segments_.size()) ? 0 : segment_index;

      segments_[segment_index][bin_index].addPoint(range, point.z);
      bin_index_[i] = std::make_pair(segment_index, bin_index);
    } else {
      bin_index_[i] = std::make_pair<int, int>(-1, -1);
    }

    segment_coordinates_[i] = Bin::MinZPoint(range, point.z);
  }
}
