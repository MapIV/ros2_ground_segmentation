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

#ifndef LINE_FIT_GROUND_SEGMENTATION_SEGMENT_H_
#define LINE_FIT_GROUND_SEGMENTATION_SEGMENT_H_

#include <list>
#include <map>
#include <mutex>
#include <ros2_ground_segmentation/linefit/bin.h>
#include <ros2_ground_segmentation/linefit/linefit_params.h>

class Segment {
public:
  typedef std::pair<Bin::MinZPoint, Bin::MinZPoint> Line;
  typedef std::pair<double, double> LineFormula;

  Segment(const LinefitSegParams &param);

  double verticalDistanceToLine(const double &d, const double &z);

  void fitSegmentLines();

  inline Bin &operator[](const size_t &index) {
    return bins_[index];
  }

  inline std::vector<Bin>::iterator begin() {
    return bins_.begin();
  }

  inline std::vector<Bin>::iterator end() {
    return bins_.end();
  }

private:
  // Parameters. Description in GroundSegmentation.
  double max_slope_;
  double max_error_;
  double long_thres_;
  double max_long_height_;
  double max_start_height_;
  double sensor_height_;

  std::vector<Bin> bins_;
  std::list<Line> lines_;

  LineFormula fitLineFormula(const std::list<Bin::MinZPoint> &points);

  double getMaxError(const std::list<Bin::MinZPoint> &points, const LineFormula &line);

  Line lineFormulaToLine(const LineFormula &local_line, const std::list<Bin::MinZPoint> &line_points);
};

#endif  // LINE_FIT_GROUND_SEGMENTATION_SEGMENT_H_
