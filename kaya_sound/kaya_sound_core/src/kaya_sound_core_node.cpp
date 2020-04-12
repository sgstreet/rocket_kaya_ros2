#include <memory>

#include <rclcpp/rclcpp.hpp>

#include <kaya_sound_core/kaya_sound_core.hpp>

int main(int argc, char **argv)
{
	// Initialize ros
	rclcpp::init(argc, argv);

	// Setup the node options
	rclcpp::NodeOptions options;

	auto node = std::make_shared<kaya_sound::KayaSoundCore>(options);

	rclcpp::spin(node);

	rclcpp::shutdown();

	return 0;
}



