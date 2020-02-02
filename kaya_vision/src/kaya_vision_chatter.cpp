/*
 * kaya_vision_chatter.cpp
 *
 *  Created on: Jan 5, 2020
 *      Author: Stephen Street (stephen@redrocketcomputing.com)
 */

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <kaya_vision/kaya_vision_chatter.hpp>

kaya_vision::KayaVisionChatter::KayaVisionChatter(const rclcpp::NodeOptions& options) :
	rclcpp::Node("kaya_vision_chatter", options)
{
	// Get the message
	chatter_msg.data = declare_parameter("message", "not set");

	// Create the publisher
	chatter_publisher = create_publisher<std_msgs::msg::String>("vision_chatter", 10);

	// Create the timer
	chatter_timer = create_wall_timer(rclcpp::Rate(1).period(), std::bind(&KayaVisionChatter::chatter_timer_callback, this));
}

kaya_vision::KayaVisionChatter::~KayaVisionChatter()
{
}

void kaya_vision::KayaVisionChatter::chatter_timer_callback()
{
	chatter_publisher->publish(chatter_msg);
}

RCLCPP_COMPONENTS_REGISTER_NODE(kaya_vision::KayaVisionChatter)


