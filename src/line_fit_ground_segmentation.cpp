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

#include "ros2_ground_segmentation/line_fit_ground_segmentation.hpp"

LinefitGroundSegmentation::LinefitGroundSegmentation(const rclcpp::NodeOptions &options) : Node("ground_segmentation", options) {

  pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "input/topic", rclcpp::SensorDataQoS().keep_last(1),
    std::bind(&LinefitGroundSegmentation::CloudCallback, this, std::placeholders::_1));

  pointcloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("output/topic", rclcpp::SensorDataQoS());

  linefit_params_ = GetParameters();

  linefit_segmentation_ptr_ = std::make_shared<LinefitGroundSeg>(linefit_params_);
  linefit_segmentation_ptr_->displayParams();

  set_param_res_ = this->add_on_set_parameters_callback(std::bind(&LinefitGroundSegmentation::ReconfigureCallback,
                                                                  this,
                                                                  std::placeholders::_1));

  tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);
}
rcl_interfaces::msg::SetParametersResult LinefitGroundSegmentation::ReconfigureCallback(const std::vector<rclcpp::Parameter> &p)
{
  bool update = false;
  if (get_param(p, "r_min", linefit_params_.r_min)) {
    RCLCPP_INFO_STREAM(get_logger(), "Setting r_min to: " << linefit_params_.r_min);
    update = true;
  }
  if (get_param(p, "r_max", linefit_params_.r_max)) {
    RCLCPP_INFO_STREAM(get_logger(), "Setting r_max to: " << linefit_params_.r_max);
    update = true;
  }
  if (get_param(p, "bins_num", linefit_params_.bins_num)) {
    RCLCPP_INFO_STREAM(get_logger(), "Setting bins_num to: " << linefit_params_.bins_num);
    update = true;
  }
  if (get_param(p, "segs_num", linefit_params_.segs_num)) {
    RCLCPP_INFO_STREAM(get_logger(), "Setting segs_num to: " << linefit_params_.segs_num);
    update = true;
  }
  if (get_param(p, "max_slope", linefit_params_.max_slope)) {
    RCLCPP_INFO_STREAM(get_logger(), "Setting max_slope to: " << linefit_params_.max_slope);
    update = true;
  }
  if (get_param(p, "max_dist_to_line", linefit_params_.max_dist_to_line)) {
    RCLCPP_INFO_STREAM(get_logger(), "Setting max_dist_to_line to: " << linefit_params_.max_dist_to_line);
    update = true;
  }
  if (get_param(p, "long_thres", linefit_params_.long_thres)) {
    RCLCPP_INFO_STREAM(get_logger(), "Setting long_thres to: " << linefit_params_.long_thres);
    update = true;
  }
  if (get_param(p, "max_long_height", linefit_params_.max_long_height)) {
    RCLCPP_INFO_STREAM(get_logger(), "Setting max_long_height to: " << linefit_params_.max_long_height);
    update = true;
  }
  if (get_param(p, "max_start_height", linefit_params_.max_start_height)) {
    RCLCPP_INFO_STREAM(get_logger(), "Setting max_start_height to: " << linefit_params_.max_start_height);
    update = true;
  }
  if (get_param(p, "line_search_angle", linefit_params_.line_search_angle)) {
    RCLCPP_INFO_STREAM(get_logger(), "Setting line_search_angle to: " << linefit_params_.line_search_angle);
    update = true;
  }
  if (get_param(p, "max_error", linefit_params_.max_error)) {
    RCLCPP_INFO_STREAM(get_logger(), "Setting max_error to: " << linefit_params_.max_error);
    update = true;
  }
  if (get_param(p, "sensor_height", linefit_params_.sensor_height)) {
    RCLCPP_INFO_STREAM(get_logger(), "Setting sensor_height to: " << linefit_params_.sensor_height);
    update = true;
  }

  if (update) {
    linefit_segmentation_ptr_->setParameters(linefit_params_);
  }
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";

  return result;
}

LinefitSegParams LinefitGroundSegmentation::GetParameters() {

  LinefitSegParams linefit_params;
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.read_only = false;
  descriptor.dynamic_typing = false;
  descriptor.additional_constraints = "";
  this->declare_parameter<double>("r_min", 1.5, descriptor);
  linefit_params.r_min = this->get_parameter("r_min").as_double();
  this->declare_parameter<double>("r_max", 100, descriptor);
  linefit_params.r_max = this->get_parameter("r_max").as_double();
  this->declare_parameter<int>("bins_num", 100, descriptor);
  linefit_params.bins_num = this->get_parameter("bins_num").as_int();
  this->declare_parameter<int>("segs_num", 16, descriptor);
  linefit_params.segs_num = this->get_parameter("segs_num").as_int();
  this->declare_parameter<int>("threads_num", 4, descriptor);
  linefit_params.threads_num = this->get_parameter("threads_num").as_int();
  this->declare_parameter<double>("max_slope", 0.1, descriptor);
  linefit_params.max_slope = this->get_parameter("max_slope").as_double();
  this->declare_parameter<double>("max_dist_to_line", 0.1, descriptor);
  linefit_params.max_dist_to_line = this->get_parameter("max_dist_to_line").as_double();
  this->declare_parameter<double>("long_thres", 2.0, descriptor);
  linefit_params.max_dist_to_line = this->get_parameter("long_thres").as_double();
  this->declare_parameter<double>("max_long_height", 0.1, descriptor);
  linefit_params.max_long_height = this->get_parameter("max_long_height").as_double();
  this->declare_parameter<double>("max_start_height", 0.2, descriptor);
  linefit_params.max_start_height = this->get_parameter("max_start_height").as_double();
  this->declare_parameter<double>("line_search_angle", 0.1, descriptor);
  linefit_params.line_search_angle = this->get_parameter("line_search_angle").as_double();
  this->declare_parameter<double>("max_error", 0.02, descriptor);
  linefit_params.max_error = this->get_parameter("max_error").as_double();
  this->declare_parameter<double>("sensor_height", 0.5, descriptor);
  linefit_params.sensor_height = this->get_parameter("sensor_height").as_double();

  this->declare_parameter<std::string>("output_frame", "base_link");
  output_frame_ = this->get_parameter("output_frame").as_string();

  return linefit_params;
}

void LinefitGroundSegmentation::transformPointCloud(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & in,
  sensor_msgs::msg::PointCloud2::SharedPtr & out)
{
  transformPointCloud(in, out, output_frame_);
}

void LinefitGroundSegmentation::transformPointCloud(
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


void LinefitGroundSegmentation::CloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr input_msg) {
  if (
    pointcloud_pub_->get_subscription_count() > 0 ||
    pointcloud_pub_->get_intra_process_subscription_count() > 0) { //subscription guards

    //transform to desired frame
    sensor_msgs::msg::PointCloud2::SharedPtr transformed_cloud_ptr(
      new sensor_msgs::msg::PointCloud2());
    transformPointCloud(input_msg, transformed_cloud_ptr);

    sensor_msgs::msg::PointCloud2 cloud_result;

    // convert ros to pcl
    pcl::PointCloud<PointXYZIR>::Ptr raw_pointcloud_ptr(new pcl::PointCloud<PointXYZIR>);
    pcl::fromROSMsg(*transformed_cloud_ptr, *raw_pointcloud_ptr);

    pcl::PointCloud<pcl::PointXYZI>::Ptr ground_pointcloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr no_ground_pointcloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    linefit_segmentation_ptr_->segmentGround(raw_pointcloud_ptr, *no_ground_pointcloud_ptr, *ground_pointcloud_ptr);

    // convert pcl to ros
    auto ros_pc_msg_ptr = std::make_unique<sensor_msgs::msg::PointCloud2>();
    pcl::toROSMsg(*no_ground_pointcloud_ptr, *ros_pc_msg_ptr);
    ros_pc_msg_ptr->header = input_msg->header;
    ros_pc_msg_ptr->header.frame_id = output_frame_;
    pointcloud_pub_->publish(std::move(ros_pc_msg_ptr));
  }// end guards

}

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(LinefitGroundSegmentation)


