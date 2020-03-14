/*
 * kaya_vision_manager.cpp
 *
 *  Created on: Jan 5, 2020
 *      Author: Stephen Street (stephen@redrocketcomputing.com)
 */

#include <chrono>

#include <kaya_vision/kaya_vision_manager.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

using namespace std::chrono_literals;

static inline std::string to_classname(const std::string& component)
{
	auto pos = component.find("://");
	if (pos == std::string::npos)
		throw rclcpp::exceptions::InvalidParametersException("no resource separator found: " + component);
	return "rclcpp_components::NodeFactoryTemplate<" + component.substr(pos + 3) + ">";
}

static inline std::string to_libname(const std::string& component)
{
	auto pos = component.find("://");
	if (pos == std::string::npos)
		throw rclcpp::exceptions::InvalidParametersException("no resource separator found: " + component);
	return class_loader::systemLibraryFormat(component.substr(0, pos));
}

kaya_vision::KayaVisionManager::KayaVisionManager(const rclcpp::NodeOptions& options) :
rclcpp_lifecycle::LifecycleNode("kaya_vision_manager", options)
{
	// Declare all parameters
	declare_parameter("pipeline_components");

	// Add ourselves to the executor
	pipeline_executor.add_node(get_node_base_interface());
}

kaya_vision::KayaVisionManager::~KayaVisionManager()
{
	// Clean up
	pipeline_executor.remove_node(get_node_base_interface());
}

kaya_vision::KayaVisionManager::CallbackReturn kaya_vision::KayaVisionManager::on_configure(const rclcpp_lifecycle::State& /*previous_state*/)
{
	try {
		// Retrieve the pipeline components
		std::vector<std::string> components = get_parameter("pipeline_components").as_string_array();

		// Build up the class loaders and pipeline components
		std::map<std::string, std::shared_ptr<class_loader::ClassLoader>> classloaders;
		for (auto const& component : components) {

			// Extract the library name
			auto classloader = classloaders.find(to_libname(component));
			if (classloader == classloaders.end())
				classloader = classloaders.insert(std::make_pair(to_libname(component), std::make_shared<class_loader::ClassLoader>(to_libname(component)))).first;

			// Create the pipeline node wrapper
			pipeline_entry pipeline_entry = { to_classname(component), classloader->second, rclcpp_components::NodeInstanceWrapper() };
			pipeline.push_back(pipeline_entry);
		}

		// All good
		return CallbackReturn::SUCCESS;

	} catch (class_loader::LibraryLoadException& e) {
		RCLCPP_ERROR(get_logger(), "creating class loader: %s", e.what());
		return CallbackReturn::FAILURE;
	} catch (rclcpp::exceptions::ParameterNotDeclaredException& e) {
		RCLCPP_ERROR(get_logger(), "get parameter: %s", e.what());
		return CallbackReturn::FAILURE;
	} catch (rclcpp::exceptions::InvalidParametersException& e) {
		RCLCPP_ERROR(get_logger(), "get parameter: %s", e.what());
		return CallbackReturn::FAILURE;
	}
}

kaya_vision::KayaVisionManager::CallbackReturn kaya_vision::KayaVisionManager::on_cleanup(const rclcpp_lifecycle::State&  /*previous_state*/)
{
	return CallbackReturn::SUCCESS;
}

kaya_vision::KayaVisionManager::CallbackReturn kaya_vision::KayaVisionManager::on_shutdown(const rclcpp_lifecycle::State& /*previous_state*/)
{
	// Let the pipe executor know it is time to shutdown
	shutdown = true;

	return CallbackReturn::SUCCESS;
}

kaya_vision::KayaVisionManager::CallbackReturn kaya_vision::KayaVisionManager::on_activate(const rclcpp_lifecycle::State& /*previous_state*/)
{
	// Set the options
	rclcpp::NodeOptions options = get_node_options();

	// Create the nodes
	for (auto &pipeline_entry : pipeline) {
		auto factory = pipeline_entry.classloader->createInstance<rclcpp_components::NodeFactory>(pipeline_entry.name);
		pipeline_entry.node_wrapper = factory->create_node_instance(options.use_intra_process_comms(true));
		pipeline_executor.add_node(pipeline_entry.node_wrapper.get_node_base_interface());
	}

	return CallbackReturn::SUCCESS;
}

kaya_vision::KayaVisionManager::CallbackReturn kaya_vision::KayaVisionManager::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/)
{
	for (auto &pipeline_entry : pipeline) {
		pipeline_executor.remove_node(pipeline_entry.node_wrapper.get_node_base_interface());
		pipeline_entry.node_wrapper = rclcpp_components::NodeInstanceWrapper();
	}

	return CallbackReturn::SUCCESS;
}

kaya_vision::KayaVisionManager::CallbackReturn kaya_vision::KayaVisionManager::on_error(const rclcpp_lifecycle::State& /*previous_state*/)
{
	return CallbackReturn::FAILURE;
}

void kaya_vision::KayaVisionManager::pipeline_spin()
{
	// Run the executor
	while (!shutdown)
		pipeline_executor.spin_some(1s);
}

RCLCPP_COMPONENTS_REGISTER_NODE(kaya_vision::KayaVisionManager)


