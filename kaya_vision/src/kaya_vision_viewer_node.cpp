/*
 * kaya_vision_viewer_node.cpp
 *
 *  Created on: Mar 8, 2020
 *      Author: Stephen Street (stephen@redrocketcomputing.com)
 */

#include <kaya_vision/kaya_vision_viewer.hpp>

#include <rclcpp/rclcpp.hpp>


int main(int argc, char ** argv)
{
	rclcpp::init(argc, argv);

	auto node = std::make_shared<kaya_vision::KayaVisionViewer>(rclcpp::NodeOptions());

	rclcpp::spin(node);

	rclcpp::shutdown();

	return 0;
}



