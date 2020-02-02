#ifndef _KAYA_JOINTS_HPP_
#define _KAYA_JOINTS_HPP_

#include <memory>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

namespace dynamixel {
class PortHandlerLinux;
class Protocol2PacketHandler;
}

namespace kaya_base {

class KayaJoints
{
	public:
		KayaJoints(rclcpp_lifecycle::LifecycleNode* parent);
		~KayaJoints();

		int init();
		int deinit();
		int enable();
		int disable();
		int read();
		int write();

		std::array<double, 3> get_position()
		{
			return present_position;
		}

		std::array<double, 3> get_velocity()
		{
			return present_velocity;
		}

		std::array<double, 3> get_effort()
		{
			return present_effort;
		}

		void set_velocity(const std::array<double, 3>& velocity)
		{
			commanded_velocity = velocity;
		}

	private:
		struct present_raw
		{
			uint16_t realtime_tick;
			uint8_t moving;
			uint8_t moving_status;
			int16_t present_pwm;
			int16_t present_load;
			int32_t present_velocity;
			int32_t present_position;
		} __attribute__((packed));


		int set_torque(bool on);
		int set_operating_mode(int mode);
		int set_led(bool on);

		rclcpp_lifecycle::LifecycleNode *parent;

		std::array<struct present_raw, 3> present_raw;
		std::array<long int, 3> position_offset;
		std::array<double, 3> present_position;
		std::array<double, 3> present_velocity;
		std::array<double, 3> present_effort;

		std::array<double, 3> commanded_velocity;

		std::unique_ptr<dynamixel::PortHandlerLinux> port_handler;
		dynamixel::Protocol2PacketHandler* protocol_handler;

		std::array<std::string, 3> joint_names;
		std::array<uint8_t, 3> joint_ids;
};

}

#endif
