/*
 * kaya_vision_chatter.cpp
 *
 *  Created on: Jan 5, 2020
 *      Author: Stephen Street (stephen@redrocketcomputing.com)
 */

#include <kaya_vision/kaya_vision_chatter.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>


kaya_vision::KayaVisionChatter::KayaVisionChatter(const rclcpp::NodeOptions& options) :
	rclcpp::Node("kaya_vision_chatter", options)
{
	// Declare all parameters
	declare_parameter("cameras", std::vector<std::string>());

	// Get the camera
	cameras = get_parameter("cameras").as_string_array();

	// Resize frame counters to match
	frame_counters.resize(cameras.size(), 0);
	last_counters.resize(cameras.size(), 0);

	// Create the subscribers
	int i = 0;
	for (auto const & camera: cameras) {
		RCLCPP_INFO(get_logger(), "subscribing to %s", camera.c_str());
		std::function<void(const sensor_msgs::msg::Image::SharedPtr msg)> image_callback = std::bind(&KayaVisionChatter::image_callback, this, std::placeholders::_1, i++);
		camera_subs.push_back(create_subscription<sensor_msgs::msg::Image>(camera, rclcpp::SensorDataQoS(), image_callback));
	}

	// Create the publisher
	chatter_publisher = create_publisher<std_msgs::msg::String>("vision_chatter", 10);

	// Create the timer
	last_timestamp = now();
	chatter_timer = create_wall_timer(rclcpp::Rate(1).period(), std::bind(&KayaVisionChatter::chatter_timer_callback, this));
}

kaya_vision::KayaVisionChatter::~KayaVisionChatter()
{
}

void kaya_vision::KayaVisionChatter::chatter_timer_callback()
{
	std_msgs::msg::String msg;
	auto timestamp = now();

	for (std::size_t i = 0; i < cameras.size(); ++i) {

		// Calculate frame per second
		int frames = frame_counters[i];
		auto fps = (frames - last_counters[i]) / (timestamp - last_timestamp).seconds();

		// Append to the chatter msg
		msg.data = msg.data + cameras[i] + ": " + std::to_string(fps) + " ";

		// Track the current frames
		last_counters[i] = frames;
	}

	last_timestamp = timestamp;

	// Publish the message
	if (chatter_publisher->get_subscription_count() > 0)
		chatter_publisher->publish(msg);
}

void kaya_vision::KayaVisionChatter::image_callback(const sensor_msgs::msg::Image::SharedPtr __attribute__((unused)) msg, const int index)
{
	++frame_counters[index];
}

RCLCPP_COMPONENTS_REGISTER_NODE(kaya_vision::KayaVisionChatter)


