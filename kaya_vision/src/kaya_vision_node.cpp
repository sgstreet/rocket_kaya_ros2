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
	rclcpp::init(argc, argv);

	rclcpp::executors::SingleThreadedExecutor executor;

    auto kaya_vision_manager = std::make_shared<kaya_vision::KayaVisionManager>(rclcpp::NodeOptions());

    executor.add_node(kaya_vision_manager->get_node_base_interface());
    executor.spin();

    rclcpp::shutdown();

	return 0;
}



