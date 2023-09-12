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

#include "ros2_composable_template/composable_sample.hpp"

Ros2ComposableTemplate::Ros2ComposableTemplate(const rclcpp::NodeOptions & options) : Node("composable_sample", options) {

  pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "input/topic", rclcpp::SensorDataQoS().keep_last(1),
      std::bind(&Ros2ComposableTemplate::CloudCallback, this, std::placeholders::_1));

  pointcloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("output/topic", rclcpp::SensorDataQoS());
}

void Ros2ComposableTemplate::CloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr input_msg)
{
  if (
    pointcloud_pub_->get_subscription_count() > 0 ||
    pointcloud_pub_->get_intra_process_subscription_count() > 0) { //subscription guards

    sensor_msgs::msg::PointCloud2 cloud_result;
    RCLCPP_INFO_STREAM(get_logger(), "Point cloud received");

    // convert ros to pcl
    pcl::PointCloud<pcl::PointXYZ>::Ptr raw_pointcloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*input_msg, *raw_pointcloud_ptr);

    // do something

    // convert pcl to ros
    auto ros_pc_msg_ptr = std::make_unique<sensor_msgs::msg::PointCloud2>();
    pcl::toROSMsg(*raw_pointcloud_ptr, *ros_pc_msg_ptr);
  }

}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(Ros2ComposableTemplate)


