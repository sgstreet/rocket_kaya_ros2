/*
 * kaya_vision_chatter.hpp
 *
 *  Created on: Jan 5, 2020
 *      Author: Stephen Street (stephen@redrocketcomputing.com)
 */

#ifndef _KAYA_VISION_CHATTER_HPP_
#define _KAYA_VISION_CHATTER_HPP_


#include <rclcpp/rclcpp.hpp>
#include <rclcpp/node.hpp>

#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace kaya_vision {

class KayaVisionChatter final : public rclcpp::Node
{
	public:
		explicit KayaVisionChatter(const rclcpp::NodeOptions& options);
		~KayaVisionChatter();

	private:

		void chatter_timer_callback();
		void image_callback(const sensor_msgs::msg::Image::SharedPtr __attribute__((unused)) msg, const int index);

		std::vector<std::string> cameras;
		rclcpp::Time last_timestamp;
		std::vector<int> last_counters;
		std::vector<int> frame_counters;
		rclcpp::Publisher<std_msgs::msg::String>::SharedPtr chatter_publisher;
		std::vector<rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr> camera_subs;
		rclcpp::TimerBase::SharedPtr chatter_timer;
};

}

#endif
