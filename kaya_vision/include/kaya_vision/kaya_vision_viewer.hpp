#ifndef _KAYA_VISION_VIEWER_HPP_
#define _KAYA_VISION_VIEWER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace kaya_vision {

class KayaVisionViewer final : public rclcpp::Node
{
	public:
		explicit KayaVisionViewer(const rclcpp::NodeOptions& options);
		~KayaVisionViewer();

	private:

		void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);

		rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_sub;
};

}

#endif
