/*
 * kaya_vision_node.cpp
 *
 *  Created on: Jan 2, 2020
 *      Author:  ()
 */

#include <memory>

#include <rclcpp/rclcpp.hpp>

#include <kaya_vision/kaya_vision_manager.hpp>

int main(int argc, char * argv[])
{
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);

	rclcpp::init(argc, argv);

	auto kaya_vision_manager = std::make_shared<kaya_vision::KayaVisionManager>(rclcpp::NodeOptions());
	kaya_vision_manager->pipeline_spin();

    rclcpp::shutdown();

	return 0;
}



