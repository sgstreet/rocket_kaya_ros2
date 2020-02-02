#include <memory>
#include <array>
#include <cmath>

#include <kaya_base/kaya_joints.hpp>

#include "port_handler_linux.h"
#include "protocol2_packet_handler.h"

constexpr uint16_t operating_mode_addr = 11;
constexpr uint16_t torque_enable_addr = 64;
constexpr uint16_t led_addr = 65;
constexpr uint16_t goal_velocity_addr = 104;
constexpr uint16_t realtime_tick_addr = 120;

constexpr uint8_t velocity_control_mode = 1;
constexpr uint8_t position_control_mode = 3;
constexpr uint8_t extended_position_control_mode = 4;
constexpr uint8_t pwm_control_mode = 16;

constexpr double velocity_scale = 0.104719755 * 0.229;
constexpr double position_scale = (2 * M_PI) / 4096;
constexpr double effort_scale = 1.5 *(0.1 / 100.0);

kaya_base::KayaJoints::KayaJoints(rclcpp_lifecycle::LifecycleNode* parent) :
parent(parent), protocol_handler(dynamixel::Protocol2PacketHandler::getInstance())
{
}

kaya_base::KayaJoints::~KayaJoints()
{
	// Do we need to clean up
	if (port_handler) {

		// Disable torque
		int status = set_torque(false);
		if (status != COMM_SUCCESS)
			RCLCPP_FATAL(parent->get_logger(), "could not turn torque on: %d", status);

		// Turn the leds off
		status = set_led(false);
		if (status != COMM_SUCCESS)
			RCLCPP_FATAL(parent->get_logger(), "could not turn leds off: %d", status);

		// Close the port handler
		port_handler->closePort();

		// Clear the port handler
		port_handler.reset();
	}
}

int kaya_base::KayaJoints::set_torque(bool on)
{
	// Build the cmd on the stack
	size_t i = 0;
	uint8_t *cmd = static_cast<uint8_t *>(alloca(joint_ids.size() * 2));
	for (auto id : joint_ids) {
		cmd[i] = id;
		cmd[i + 1] = on;
		i += 2;
	}

	// Send it off
	return protocol_handler->syncWriteTxOnly(port_handler.get(), torque_enable_addr, 1, cmd, joint_ids.size() * 2);
}

int kaya_base::KayaJoints::set_operating_mode(int mode)
{
	// Build the cmd on the stack
	size_t i = 0;
	uint8_t *cmd = static_cast<uint8_t *>(alloca(joint_ids.size() * 2));
	for (auto id : joint_ids) {
		cmd[i] = id;
		cmd[i + 1] = mode & 0xff;;
		i += 2;
	}

	// Send it off
	return protocol_handler->syncWriteTxOnly(port_handler.get(), operating_mode_addr, 1, cmd, joint_ids.size() * 2);
}

int kaya_base::KayaJoints::set_led(bool on)
{
	// Build the cmd on the stack
	size_t i = 0;
	uint8_t *cmd = static_cast<uint8_t *>(alloca(joint_ids.size() * 2));
	for (auto id : joint_ids) {
		cmd[i] = id;
		cmd[i + 1] = on;
		i += 2;
	}

	// Send it off
	return protocol_handler->syncWriteTxOnly(port_handler.get(), led_addr, 1, cmd, joint_ids.size() * 2);
}

int kaya_base::KayaJoints::init()
{
	int status;

	// Initialize the data blocks
	position_offset = { 0, 0, 0 };
	present_position = { 0.0, 0.0, 0.0 };
	present_velocity = { 0.0, 0.0, 0.0 };
	present_effort = { 0.0, 0.0, 0.0 };
	commanded_velocity = { 0.0, 0.0, 0.0 };

	// Get the U2D2 information
	std::string u2d2_port = parent->get_parameter("joints.u2d2.port").as_string();
	long int u2d2_baud_rate = parent->get_parameter("joints.u2d2.baud_rate").as_int();

	// Get the joint names and ids
	std::vector<std::string> names = parent->get_parameter("joints.names").as_string_array();
	std::vector<long int> ids = parent->get_parameter("joints.ids").as_integer_array();
	uint16_t joint_model = parent->get_parameter("joints.model").as_int() & 0xffff;

	// Range check the vectors, the kaya has three motors
	if (names.size() != 3 || ids.size() != 3) {
		RCLCPP_FATAL(parent->get_logger(), "wrong number of id or names: %lu %lu", ids.size(), ids.size());
		return -1;
	}

	// Open the port
	if (!port_handler) {
		port_handler = std::make_unique<dynamixel::PortHandlerLinux>(u2d2_port.c_str());
		if (!port_handler->openPort()) {
			RCLCPP_FATAL(parent->get_logger(), "problem opening U2D2 port: %s", u2d2_port.c_str());
			return -1;
		}
	}

	// Set the port handler
	if (!port_handler->setBaudRate(u2d2_baud_rate)) {
		RCLCPP_FATAL(parent->get_logger(), "invalid U2D2 baud rate: %ld", u2d2_baud_rate);
		return -1;
	}

	// Ping all the joint and verify the model number matches
	for (size_t i = 0; i < joint_ids.size(); ++i) {

		// Initial the names and ids
		joint_names[i] = names[i];
		joint_ids[i] = ids[i];

		// Launch ping
		uint16_t model;
		uint8_t error = 0;
		status = protocol_handler->ping(port_handler.get(), joint_ids[i], &model, &error);
		if (status != COMM_SUCCESS) {
			RCLCPP_FATAL(parent->get_logger(), "could not ping joint %ld: %d", joint_ids[i], status);
			return -1;
		}
		if (error != 0) {
			RCLCPP_FATAL(parent->get_logger(), "ping of joint %ld failed with error: %hhu", joint_ids[i], error);
			return -1;
		}

		// Match the model numbers
		if (joint_model != model) {
			RCLCPP_FATAL(parent->get_logger(), "joint %ld bad model: %hu", model);
			return -1;
		}

		RCLCPP_INFO(parent->get_logger(), "found joint %s with model 0x%02hx", joint_names[i].c_str(), model);
	}

	// Turn the leds off
	status = set_led(false);
	if (status != COMM_SUCCESS) {
		RCLCPP_FATAL(parent->get_logger(), "could not turn leds on: %d", status);
		return -1;
	}

	// Turn the torque off for all motors
	status = set_torque(false);
	if (status != COMM_SUCCESS) {
		RCLCPP_FATAL(parent->get_logger(), "could not turn torque off: %d", status);
		return -1;
	}

	// Set the operating mode to velocity for all motors
	status = set_operating_mode(velocity_control_mode);
	if (status != COMM_SUCCESS) {
		RCLCPP_FATAL(parent->get_logger(), "could not set velocity control mode: %d", status);
		return -1;
	}

	// Write the zero velocity
	status = write();
	if (status < 0) {
		RCLCPP_FATAL(parent->get_logger(), "could not send the zero velocity command: %d", status);
		return -1;
	}

	// Read the current state
	status = read();
	if (status < 0) {
		RCLCPP_FATAL(parent->get_logger(), "could not read the current state: %d", status);
		return -1;
	}

	// Update the position offset
	position_offset[0] = present_raw[0].present_position;
	position_offset[1] = present_raw[1].present_position;
	position_offset[2] = present_raw[2].present_position;

	RCLCPP_INFO(parent->get_logger(), "joints ready");

	return 0;
}

int kaya_base::KayaJoints::deinit()
{
	// Do we need to clean up
	if (port_handler) {

		// Disable torque
		int status = set_torque(false);
		if (status != COMM_SUCCESS) {
			RCLCPP_FATAL(parent->get_logger(), "could not turn torque on: %d", status);
			return -1;
		}

		// Turn the leds off
		status = set_led(false);
		if (status != COMM_SUCCESS) {
			RCLCPP_FATAL(parent->get_logger(), "could not turn leds off: %d", status);
			return -1;
		}

		// Close the port handler
		port_handler->closePort();

		// Clear the port handler
		port_handler.reset();
	}

	RCLCPP_INFO(parent->get_logger(), "joints not ready");

	// All good
	return 0;
}

int kaya_base::KayaJoints::enable()
{
	// Enable torque
	int status = set_torque(true);
	if (status != COMM_SUCCESS) {
		RCLCPP_FATAL(parent->get_logger(), "could not turn torque on: %d", status);
		return -1;
	}

	// Turn the leds on
	status = set_led(true);
	if (status != COMM_SUCCESS) {
		RCLCPP_FATAL(parent->get_logger(), "could not turn leds on: %d", status);
		return -1;
	}

	RCLCPP_INFO(parent->get_logger(), "joints powered");

	// All good
	return 0;
}

int kaya_base::KayaJoints::disable()
{
	// Disable torque
	int status = set_torque(false);
	if (status != COMM_SUCCESS) {
		RCLCPP_FATAL(parent->get_logger(), "could not turn torque off: %d", status);
		return -1;
	}

	// Turn the leds off
	status = set_led(false);
	if (status != COMM_SUCCESS) {
		RCLCPP_FATAL(parent->get_logger(), "could not turn leds off: %d", status);
		return -1;
	}

	RCLCPP_INFO(parent->get_logger(), "joints unpowered");

	// All good
	return 0;
}

int kaya_base::KayaJoints::read()
{
	int status;
	uint8_t error;

	// Send the sync read request
	status = protocol_handler->syncReadTx(port_handler.get(), realtime_tick_addr, sizeof(struct present_raw), joint_ids.data(), joint_ids.size());
	if (status != COMM_SUCCESS) {
		RCLCPP_ERROR(parent->get_logger(), "could not send the sync read: %d", status);
		return -1;
	}

	// Try to read all the responses
	size_t i = 0;
	for (auto id : joint_ids) {

		// Read the response
		error = 0;
		status = protocol_handler->readRx(port_handler.get(), id, sizeof(struct present_raw), reinterpret_cast<uint8_t *>(&present_raw[i]), &error);
		if (status != COMM_SUCCESS) {
			RCLCPP_ERROR(parent->get_logger(), "could not read response for %hhu: %d", id, status);
			return -1;
		}

		// Process the response
		present_position[i] = (present_raw[i].present_position - position_offset[i]) * position_scale;
		present_velocity[i] = present_raw[i].present_velocity * velocity_scale;
		present_effort[i] = present_raw[i].present_load * effort_scale;

		// Move to the next buffer
		++i;
	}

	return 0;
}

int kaya_base::KayaJoints::write()
{
	// Build the cmd on the stack
	size_t i = 0;
	uint8_t *cmd = static_cast<uint8_t *>(alloca(joint_ids.size() * 5));
	for (auto id : joint_ids) {
		cmd[i * 5] = id;
		int32_t raw_velocity = std::lround(commanded_velocity[i] / velocity_scale);
		memcpy(&cmd[(i * 5) + 1], &raw_velocity, sizeof(raw_velocity));
		++i;
	}

	// Send it off
	return protocol_handler->syncWriteTxOnly(port_handler.get(), goal_velocity_addr, sizeof(int32_t), cmd, joint_ids.size() * 5);
}
