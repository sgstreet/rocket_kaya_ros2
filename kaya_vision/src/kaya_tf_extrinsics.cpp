/*
 * kaya_tf_extrinsics.cpp
 *
 *  Created on: Jan 18, 2020
 *      Author: Stephen Street (stephen@redrocketcomputing.com)
 */

#include <kaya_vision/kaya_tf_extrinsics.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>


kaya_vision::KayaTFExtinsics::KayaTFExtinsics(const rclcpp::NodeOptions& /* options */) :
	Node("kaya_tf_extrinsics", rclcpp::NodeOptions())
{
	// Declare parameters
	declare_parameter("cameras", std::vector<std::string>());

	// Start up the static transform broadcaster
	transform_broadcaster = std::make_unique<tf2_ros::StaticTransformBroadcaster>(this);

	// Loop through the cameras subscibing the the extrinsics topics
	auto cameras = get_parameter("cameras").as_string_array();
	for (auto const& camera : cameras) {
		RCLCPP_INFO(get_logger(), "publishing transform for %s", camera.c_str());
		extrinsics_subs.push_back(create_subscription<geometry_msgs::msg::TransformStamped>(camera, 1, std::bind(&KayaTFExtinsics::extrinsics_callback, this, std::placeholders::_1)));
	}
}

kaya_vision::KayaTFExtinsics::~KayaTFExtinsics()
{

}

void kaya_vision::KayaTFExtinsics::extrinsics_callback(const geometry_msgs::msg::TransformStamped::SharedPtr msg)
{
//	RCLCPP_INFO(get_logger(), "sending transform form %s to %s", msg->header.frame_id.c_str(), msg->child_frame_id.c_str());
	transform_broadcaster->sendTransform(*msg);
}

RCLCPP_COMPONENTS_REGISTER_NODE(kaya_vision::KayaTFExtinsics)
