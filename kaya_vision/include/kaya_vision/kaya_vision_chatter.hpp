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

namespace kaya_vision {

class KayaVisionChatter final : public rclcpp::Node
{
	public:
		explicit KayaVisionChatter(const rclcpp::NodeOptions& options);
		~KayaVisionChatter();

	private:

		void chatter_timer_callback();

		std_msgs::msg::String chatter_msg;
		rclcpp::Publisher<std_msgs::msg::String>::SharedPtr chatter_publisher;
		rclcpp::TimerBase::SharedPtr chatter_timer;
};

}

#endif
