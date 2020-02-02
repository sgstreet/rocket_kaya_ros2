/*
 * kaya_vision_manager.hpp
 *
 *  Created on: Jan 5, 2020
 *      Author: Stephen Street (stephen@redrocketcomputing.com)
 */

#ifndef _KAYA_VISION_MANAGER_HPP_
#define _KAYA_VISION_MANAGER_HPP_

#include <memory>
#include <thread>
#include <set>

#include <rclcpp/rclcpp.hpp>

#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <rclcpp_components/node_factory.hpp>

#include <class_loader/class_loader.hpp>

#include <lifecycle_msgs/msg/transition.hpp>

namespace kaya_vision {

class KayaVisionManager : public rclcpp_lifecycle::LifecycleNode
{
	public:
		using rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

		explicit KayaVisionManager(const rclcpp::NodeOptions& options);
		virtual ~KayaVisionManager();

		virtual CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state);
		virtual CallbackReturn on_cleanup(const rclcpp_lifecycle::State& previous_state);
		virtual CallbackReturn on_shutdown(const rclcpp_lifecycle::State& previous_state);
		virtual CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state);
		virtual CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state);
		virtual CallbackReturn on_error(const rclcpp_lifecycle::State& previous_state);

	private:
		struct pipeline_entry
		{
			std::string name;
			std::shared_ptr<class_loader::ClassLoader> classloader;
			rclcpp_components::NodeInstanceWrapper node_wrapper;
		};

		std::vector<pipeline_entry> pipeline;

		rclcpp::executors::MultiThreadedExecutor pipeline_executor;
		std::unique_ptr<std::thread> pipeline_thread;

		void pipeline_spin();
};

}


#endif
