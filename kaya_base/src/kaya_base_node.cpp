#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <kaya_base/kaya_base.hpp>

int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);

	rclcpp::NodeOptions options;

	auto kaya_base = std::make_shared<kaya_base::KayaBase>(options);

	rclcpp::spin(kaya_base->get_node_base_interface());

	RCLCPP_INFO(kaya_base->get_logger(), "shutdown");

	return 0;
}
