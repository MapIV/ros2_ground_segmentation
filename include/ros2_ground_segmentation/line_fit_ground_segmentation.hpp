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

#ifndef LINEFIT_GROUND_SEGMENTATION_H
#define LINEFIT_GROUND_SEGMENTATION_H

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/node_interfaces/node_parameters_interface.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <ros2_ground_segmentation/point_type.h>
#include <ros2_ground_segmentation/linefit/linefit_params.h>
#include <ros2_ground_segmentation/linefit/linefit_seg.h>


template <typename T>
bool get_param(const std::vector<rclcpp::Parameter> & p, const std::string & name, T & value)
{
  auto it = std::find_if(p.cbegin(), p.cend(), [&name](const rclcpp::Parameter & parameter) {
    return parameter.get_name() == name;
  });
  if (it != p.cend()) {
    value = it->template get_value<T>();
    return true;
  }
  return false;
}

class LinefitGroundSegmentation : public rclcpp::Node
{
public:
  explicit LinefitGroundSegmentation(const rclcpp::NodeOptions & options);

private:
  void CloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr input_msg);
  rcl_interfaces::msg::SetParametersResult ReconfigureCallback(const std::vector<rclcpp::Parameter> & p);
  void transformPointCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & in,
                           sensor_msgs::msg::PointCloud2::SharedPtr & out);
  void transformPointCloud(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & in,
    sensor_msgs::msg::PointCloud2::SharedPtr & out,
    const std::string & target_frame);
  OnSetParametersCallbackHandle::SharedPtr set_param_res_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;
  std::shared_ptr<LinefitGroundSeg> linefit_segmentation_ptr_;

  LinefitSegParams linefit_params_;
  LinefitSegParams GetParameters();
  std::string output_frame_;

  std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;
};

#endif //LINEFIT_GROUND_SEGMENTATION_H
