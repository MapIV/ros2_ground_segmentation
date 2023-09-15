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

#include "ros2_ground_segmentation/ray_ground_segmentation.hpp"

RayGroundSegmentation::RayGroundSegmentation(const rclcpp::NodeOptions &options) : Node("ray_ground_segmentation", options) {

  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "input/topic", rclcpp::SensorDataQoS().keep_last(1),
    std::bind(&RayGroundSegmentation::CloudCallback, this, std::placeholders::_1));

  pointcloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("output/topic", rclcpp::SensorDataQoS());

  rayground_params_ = GetParameters();

  rayground_ptr_ = std::make_shared<RayGroundSeg>(rayground_params_);

  set_param_res_ = this->add_on_set_parameters_callback(std::bind(&RayGroundSegmentation::ReconfigureCallback,
                                                                  this,
                                                                  std::placeholders::_1));

  tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);
}

rcl_interfaces::msg::SetParametersResult RayGroundSegmentation::ReconfigureCallback(const std::vector<rclcpp::Parameter> &p)
{
  bool update = false;
  if (get_param(p, "general_max_slope", rayground_params_.general_max_slope)) {
    RCLCPP_INFO_STREAM(get_logger(), "Setting general_max_slope to: " << rayground_params_.general_max_slope);
    update = true;
  }
  if (get_param(p, "local_max_slope", rayground_params_.local_max_slope)) {
    RCLCPP_INFO_STREAM(get_logger(), "Setting local_max_slope to: " << rayground_params_.local_max_slope);
    update = true;
  }
  if (get_param(p, "sensor_height", rayground_params_.sensor_height)) {
    RCLCPP_INFO_STREAM(get_logger(), "Setting bins_num to: " << rayground_params_.sensor_height);
    update = true;
  }
  if (get_param(p, "radial_divider_angle", rayground_params_.radial_divider_angle)) {
    RCLCPP_INFO_STREAM(get_logger(), "Setting radial_divider_angle to: " << rayground_params_.radial_divider_angle);
    update = true;
  }
  if (get_param(p, "concentric_divider_distance", rayground_params_.concentric_divider_distance)) {
    RCLCPP_INFO_STREAM(get_logger(), "Setting concentric_divider_distance to: " << rayground_params_.concentric_divider_distance);
    update = true;
  }
  if (get_param(p, "min_height_threshold", rayground_params_.min_height_threshold)) {
    RCLCPP_INFO_STREAM(get_logger(), "Setting min_height_threshold to: " << rayground_params_.min_height_threshold);
    update = true;
  }
  if (get_param(p, "clipping_height", rayground_params_.clipping_height)) {
    RCLCPP_INFO_STREAM(get_logger(), "Setting clipping_height to: " << rayground_params_.clipping_height);
    update = true;
  }
  if (get_param(p, "pointcloud_min_z", rayground_params_.pointcloud_min_z)) {
    RCLCPP_INFO_STREAM(get_logger(), "Setting pointcloud_min_z to: " << rayground_params_.pointcloud_min_z);
    update = true;
  }
  if (get_param(p, "min_point_distance", rayground_params_.min_point_distance)) {
    RCLCPP_INFO_STREAM(get_logger(), "Setting min_point_distance to: " << rayground_params_.min_point_distance);
    update = true;
  }
  if (get_param(p, "reclass_distance_threshold", rayground_params_.reclass_distance_threshold)) {
    RCLCPP_INFO_STREAM(get_logger(), "Setting reclass_distance_threshold to: " << rayground_params_.reclass_distance_threshold);
    update = true;
  }
  if (get_param(p, "outlier_filter", rayground_params_.outlier_filter)) {
    RCLCPP_INFO_STREAM(get_logger(), "Setting outlier_filter to: " << rayground_params_.outlier_filter);
    update = true;
  }

  if (update) {
    rayground_ptr_->setParameters(rayground_params_);
  }
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";

  return result;
}

RayGroundParams RayGroundSegmentation::GetParameters() {

  RayGroundParams rayground_params;
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.read_only = false;
  descriptor.dynamic_typing = false;
  descriptor.additional_constraints = "";
  this->declare_parameter<double>("general_max_slope", 3.0, descriptor);
  rayground_params.general_max_slope = this->get_parameter("general_max_slope").as_double();
  this->declare_parameter<double>("local_max_slope", 5.0, descriptor);
  rayground_params.local_max_slope = this->get_parameter("local_max_slope").as_double();
  this->declare_parameter<double>("sensor_height", 0.0, descriptor);
  rayground_params.sensor_height = this->get_parameter("sensor_height").as_double();
  this->declare_parameter<double>("radial_divider_angle", 0.1, descriptor);
  rayground_params.radial_divider_angle = this->get_parameter("radial_divider_angle").as_double();
  this->declare_parameter<double>("concentric_divider_distance", 0.01, descriptor);
  rayground_params.concentric_divider_distance = this->get_parameter("concentric_divider_distance").as_double();
  this->declare_parameter<double>("min_height_threshold", 0.05, descriptor);
  rayground_params.min_height_threshold = this->get_parameter("min_height_threshold").as_double();
  this->declare_parameter<double>("clipping_height", 2.0, descriptor);
  rayground_params.clipping_height = this->get_parameter("clipping_height").as_double();
  this->declare_parameter<double>("pointcloud_min_z", -0.2, descriptor);
  rayground_params.pointcloud_min_z = this->get_parameter("pointcloud_min_z").as_double();
  this->declare_parameter<double>("min_point_distance", 0.5, descriptor);
  rayground_params.min_point_distance = this->get_parameter("min_point_distance").as_double();
  this->declare_parameter<double>("reclass_distance_threshold", 0.2, descriptor);
  rayground_params.reclass_distance_threshold = this->get_parameter("reclass_distance_threshold").as_double();

  this->declare_parameter<bool>("outlier_filter", false, descriptor);
  rayground_params.outlier_filter = this->get_parameter("outlier_filter").as_bool();

  this->declare_parameter<std::string>("output_frame", "base_link");
  output_frame_ = this->get_parameter("output_frame").as_string();

  return rayground_params;
}

void RayGroundSegmentation::transformPointCloud(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & in,
  sensor_msgs::msg::PointCloud2::SharedPtr & out)
{
  transformPointCloud(in, out, output_frame_);
}

void RayGroundSegmentation::transformPointCloud(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & in,
  sensor_msgs::msg::PointCloud2::SharedPtr & out,
  const std::string & target_frame)
{
  // Transform the point clouds into the specified output frame
  if (target_frame != in->header.frame_id) {
    if (!pcl_ros::transformPointCloud(target_frame, *in, *out, *tf2_buffer_)) {
      RCLCPP_ERROR(
        this->get_logger(),
        "[transformPointCloud] Error converting first input dataset from %s to %s.",
        in->header.frame_id.c_str(), target_frame.c_str());
      return;
    }
  } else {
    out = std::make_shared<sensor_msgs::msg::PointCloud2>(*in);
  }
}


void RayGroundSegmentation::CloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr input_msg) {
//  if (
//    pointcloud_pub_->get_subscription_count() > 0 ||
//    pointcloud_pub_->get_intra_process_subscription_count() > 0)
  { //subscription guards

    //transform to desired frame
    sensor_msgs::msg::PointCloud2::SharedPtr transformed_cloud_ptr(
      new sensor_msgs::msg::PointCloud2());
    transformPointCloud(input_msg, transformed_cloud_ptr);

    sensor_msgs::msg::PointCloud2 cloud_result;

    // convert ros to pcl
    pcl::PointCloud<pcl::PointXYZI>::Ptr raw_pointcloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*transformed_cloud_ptr, *raw_pointcloud_ptr);

    pcl::PointCloud<pcl::PointXYZI>::Ptr ground_pointcloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr no_ground_pointcloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    rayground_ptr_->segmentGround(raw_pointcloud_ptr,
                                  no_ground_pointcloud_ptr,
                                  ground_pointcloud_ptr);
    RCLCPP_INFO_STREAM(get_logger(), "points:" << raw_pointcloud_ptr->points.size()
    <<" no_g: " << no_ground_pointcloud_ptr->points.size()
    <<" gr: " << ground_pointcloud_ptr->points.size());

    // convert pcl to ros
    auto ros_pc_msg_ptr = std::make_unique<sensor_msgs::msg::PointCloud2>();
    pcl::toROSMsg(*no_ground_pointcloud_ptr, *ros_pc_msg_ptr);
    ros_pc_msg_ptr->header = input_msg->header;
    ros_pc_msg_ptr->header.frame_id = output_frame_;
    pointcloud_pub_->publish(std::move(ros_pc_msg_ptr));
  }// end guards

}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(RayGroundSegmentation)


