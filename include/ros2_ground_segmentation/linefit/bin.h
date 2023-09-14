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

#ifndef LINEFIT_BIN_H
#define LINEFIT_BIN_H

#include <atomic>
#include <limits>
#include <cmath>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class Bin {
public:
  struct MinZPoint {
    MinZPoint() : z(0), d(0) {
    }

    MinZPoint(const double &d, const double &z) : z(z), d(d) {
    }

    bool operator==(const MinZPoint &comp) const {
      return z == comp.z && d == comp.d;
    }

    double z;  // height
    double d;  // range
  };

  Bin();

  Bin(const Bin &segment);

  void addPoint(const pcl::PointXYZ &point);

  void addPoint(const double &d, const double &z);

  MinZPoint getMinZPoint();

  inline bool hasPoint() {
    return has_point_;
  }

private:
  std::atomic<bool> has_point_;
  std::atomic<double> min_z_;
  std::atomic<double> min_z_d_;
};

#endif  // LINEFIT_BIN_H
