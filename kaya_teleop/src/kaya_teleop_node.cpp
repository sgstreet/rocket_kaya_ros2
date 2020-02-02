#include <rclcpp/rclcpp.hpp>
#include <kaya_teleop/kaya_teleop.hpp>

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);

	rclcpp::NodeOptions options;
	options.allow_undeclared_parameters(true);
    options.automatically_declare_parameters_from_overrides(true);

	auto kaya_teleop = std::make_shared<kaya_teleop::KayaTeleop>("kaya_teleop", options);

	rclcpp::spin(kaya_teleop);

	RCLCPP_INFO(kaya_teleop->get_logger(), "shutdown");

	return 0;
}
