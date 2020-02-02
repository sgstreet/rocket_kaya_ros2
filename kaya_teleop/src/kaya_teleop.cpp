#include "rclcpp/logger.hpp"

#include "kaya_teleop/kaya_teleop.hpp"

kaya_teleop::KayaTeleop::KayaTeleop(const std::string& node_name, const rclcpp::NodeOptions& options) :
	rclcpp::Node(node_name, options)

{
	// Build up the axes
	axes.push_back(get_parameter("linear_axes.x").as_int());
	axes.push_back(get_parameter("linear_axes.y").as_int());
	axes.push_back(get_parameter("angular_axes.yaw").as_int());

	// Build up the scales
	scale.push_back(get_parameter("linear_scale.x").as_double());
	scale.push_back(get_parameter("linear_scale.y").as_double());
	scale.push_back(get_parameter("angular_scale.yaw").as_double());

	// Build up the switch map
	auto switch_names = get_parameter("switches").as_string_array();
	for (auto name : switch_names) {

		// First look for the mapped button
		int button = get_parameter(name).as_int();

		// Now bind the button to the action
		if (name == "deadman") {
			switches[button] = std::bind(&KayaTeleop::deadman_action, this, std::placeholders::_1, std::placeholders::_2);
			deadman = button;
			button_state[button] = false;
		} else if (name == "teleop") {
			switches[button] = std::bind(&KayaTeleop::teleop_action, this, std::placeholders::_1, std::placeholders::_2);
			teleop = button;
			button_state[button] = false;
		} else if (name == "omni") {
			switches[button] = std::bind(&KayaTeleop::omni_action, this, std::placeholders::_1, std::placeholders::_2);
			button_state[button] = false;
		} else if (name == "activate") {
			switches[button] = std::bind(&KayaTeleop::activate_action, this, std::placeholders::_1, std::placeholders::_2);
			button_state[button] = false;
		} else if (name == "explore") {
			switches[button] = std::bind(&KayaTeleop::explore_action, this, std::placeholders::_1, std::placeholders::_2);
			button_state[button] = false;
		} else if (name == "patrol") {
			switches[button] = std::bind(&KayaTeleop::patrol_action, this, std::placeholders::_1, std::placeholders::_2);
			button_state[button] = false;
		} else if (name == "halt") {
			switches[button] = std::bind(&KayaTeleop::halt_action, this, std::placeholders::_1, std::placeholders::_2);
			button_state[button] = false;
		} else if (name == "spin") {
			switches[button] = std::bind(&KayaTeleop::spin_action, this, std::placeholders::_1, std::placeholders::_2);
			button_state[button] = false;
		} else
			RCLCPP_WARN(get_logger(), "unknown action: %s", name.c_str());
	}

	// Create publishers and subscribers
	cmd_vel_publisher = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
	joy_subscriber = create_subscription<sensor_msgs::msg::Joy>("joy", 10, std::bind(&KayaTeleop::joy_callback, this, std::placeholders::_1));
}

kaya_teleop::KayaTeleop::~KayaTeleop()
{
}

void kaya_teleop::KayaTeleop::joy_callback(sensor_msgs::msg::Joy::SharedPtr msg)
{
	// Dispatch buttons
	for (const auto& [button, handler] : switches)
		if (button_state[button] != msg->buttons[button])
			handler(button, msg->buttons[button]);

	// Generate twist
	if (is_teleop()) {

		// Build of the diff twist
		geometry_msgs::msg::Twist cmd_vel;
		if (omni) {
			cmd_vel.linear.x = msg->axes[axes[0]] * scale[0];
			cmd_vel.linear.y = msg->axes[axes[1]] * scale[1];
			cmd_vel.angular.z = msg->axes[axes[2]] * scale[2];
		} else {
			cmd_vel.linear.x = msg->axes[axes[0]] * scale[0];
			cmd_vel.linear.y = 0;
			cmd_vel.angular.z = msg->axes[axes[2]] * scale[2];
		}

		// Publish the twist
		cmd_vel_publisher->publish(cmd_vel);
	}
}

void kaya_teleop::KayaTeleop::deadman_action(int button, bool state)
{
	button_state[button] = state;
	RCLCPP_INFO(get_logger(), "deadman: %s", state ? "true" : "false");
}

void kaya_teleop::KayaTeleop::teleop_action(int button, bool state)
{
	button_state[button] = state;
	RCLCPP_INFO(get_logger(), "teleop: %s", state ? "true" : "false");
}

void kaya_teleop::KayaTeleop::activate_action(int button, bool state)
{
	button_state[button] = state;
	RCLCPP_INFO(get_logger(), "activate: %s", state ? "true" : "false");
}

void kaya_teleop::KayaTeleop::explore_action(int button, bool state)
{
	button_state[button] = state;
	RCLCPP_INFO(get_logger(), "explore: %s", state ? "true" : "false");
}

void kaya_teleop::KayaTeleop::patrol_action(int button, bool state)
{
	button_state[button] = state;
	RCLCPP_INFO(get_logger(), "patrol: %s", state ? "true" : "false");
}

void kaya_teleop::KayaTeleop::halt_action(int button, bool state)
{
	button_state[button] = state;
	RCLCPP_INFO(get_logger(), "halt: %s", state ? "true" : "false");
}

void kaya_teleop::KayaTeleop::spin_action(int button, bool state)
{
	button_state[button] = state;
	RCLCPP_INFO(get_logger(), "spin: %s", state ? "true" : "false");
}

void kaya_teleop::KayaTeleop::omni_action(int button, bool state)
{
	// Latch this
	if (state)
		omni = !omni;
	button_state[button] = state;
	RCLCPP_INFO(get_logger(), "omni: %s", omni ? "true" : "false");
}

