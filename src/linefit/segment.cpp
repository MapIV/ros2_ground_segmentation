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

#include <ros2_ground_segmentation/linefit/segment.h>

Segment::Segment(const LinefitSegParams &param) {
  bins_.clear();
  bins_.resize(param.bins_num);
  max_slope_ = param.max_slope;
  max_error_ = param.max_error;
  long_thres_ = param.long_thres;
  max_long_height_ = param.max_long_height;
  max_start_height_ = param.max_start_height;
  sensor_height_ = param.sensor_height;
}

double Segment::verticalDistanceToLine(const double &d, const double &z) {
  static const double kMargin = 0.1;
  double distance = -1;
  for (auto it = lines_.begin(); it != lines_.end(); ++it) {
    if (it->first.d - kMargin < d && it->second.d + kMargin > d) {
      const double delta_z = it->second.z - it->first.z;
      const double delta_d = it->second.d - it->first.d;
      const double expected_z = (d - it->first.d) / delta_d * delta_z + it->first.z;
      distance = std::fabs(z - expected_z);
    }
  }
  return distance;
}

void Segment::fitSegmentLines() {
  // Find first point.
  auto line_start = bins_.begin();
  while (!line_start->hasPoint()) {
    ++line_start;
    // Stop if we reached last point.
    if (line_start == bins_.end())
      return;
  }
  // Fill lines.
  bool is_long_line = false;
  double cur_ground_height = -sensor_height_;
  std::list<Bin::MinZPoint> current_line_points(1, line_start->getMinZPoint());
  LineFormula cur_line = std::make_pair(0, 0);
  for (auto line_iter = line_start + 1; line_iter != bins_.end(); ++line_iter) {
    if (line_iter->hasPoint()) {
      Bin::MinZPoint cur_point = line_iter->getMinZPoint();
      if (cur_point.d - current_line_points.back().d > long_thres_) {
        is_long_line = true;
      }
      if (current_line_points.size() >= 2) {
        // Get expected z value to possibly reject far away points.
        double expected_z = std::numeric_limits<double>::max();
        if (is_long_line && current_line_points.size() > 2) {
          expected_z = cur_line.first * cur_point.d + cur_line.second;
        }
        current_line_points.push_back(cur_point);
        cur_line = fitLineFormula(current_line_points);
        const double error = getMaxError(current_line_points, cur_line);
        // Check if not a good line.
        if (error > max_error_ || std::fabs(cur_line.first) > max_slope_ ||
            is_long_line && std::fabs(expected_z - cur_point.z) > max_long_height_) {
          // Add line until previous point as ground.
          current_line_points.pop_back();
          // Don't let lines with 2 base points through.
          if (current_line_points.size() >= 3) {
            const LineFormula new_line = fitLineFormula(current_line_points);
            lines_.push_back(lineFormulaToLine(new_line, current_line_points));
            cur_ground_height = new_line.first * current_line_points.back().d + new_line.second;
          }
          // Start new line.
          is_long_line = false;
          current_line_points.erase(current_line_points.begin(), --current_line_points.end());
          --line_iter;
        }
          // Good line, continue.
        else {
        }
      } else {
        // Not enough points.
        if (cur_point.d - current_line_points.back().d < long_thres_ &&
            std::fabs(current_line_points.back().z - cur_ground_height) < max_start_height_) {
          // Add point if valid.
          current_line_points.push_back(cur_point);
        } else {
          // Start new line.
          current_line_points.clear();
          current_line_points.push_back(cur_point);
        }
      }
    }
  }
  // Add last line.
  if (current_line_points.size() > 2) {
    const LineFormula new_line = fitLineFormula(current_line_points);
    lines_.push_back(lineFormulaToLine(new_line, current_line_points));
  }
}

Segment::LineFormula Segment::fitLineFormula(const std::list<Bin::MinZPoint> &points) {
  const unsigned int n_points = points.size();
  Eigen::MatrixXd X(n_points, 2);
  Eigen::VectorXd Y(n_points);
  unsigned int counter = 0;
  for (auto iter = points.begin(); iter != points.end(); ++iter) {
    X(counter, 0) = iter->d;
    X(counter, 1) = 1;
    Y(counter) = iter->z;
    ++counter;
  }
  const Eigen::MatrixXd X_t = X.transpose();
  const Eigen::VectorXd result = (X_t * X).inverse() * X_t * Y;
  LineFormula line_result;
  line_result.first = result(0);
  line_result.second = result(1);
  return line_result;
}

double Segment::getMaxError(const std::list<Bin::MinZPoint> &points, const LineFormula &line) {
  double max_error = 0;
  for (auto it = points.begin(); it != points.end(); ++it) {
    const double residual = (line.first * it->d + line.second) - it->z;
    const double error = std::sqrt(residual * residual);
    if (error > max_error)
      max_error = error;
  }
  return max_error;
}

Segment::Line Segment::lineFormulaToLine(const LineFormula &local_line, const std::list<Bin::MinZPoint> &line_points) {
  Line line;
  const double first_d = line_points.front().d;
  const double second_d = line_points.back().d;
  const double first_z = local_line.first * first_d + local_line.second;
  const double second_z = local_line.first * second_d + local_line.second;
  line.first.z = first_z;
  line.first.d = first_d;
  line.second.z = second_z;
  line.second.d = second_d;
  return line;
}
