#ifndef _KAYA_TELEOP_HPP_
#define _KAYA_TELEOP_HPP_

#include <string>
#include <vector>
#include <functional>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/node.hpp>

#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include "kaya_teleop/visibility_control.h"

namespace kaya_teleop {

class KayaTeleop : public rclcpp::Node
{
	public:
		KayaTeleop(const std::string& node_name, const rclcpp::NodeOptions& options);
		virtual ~KayaTeleop();

	private:
		std::vector<int> axes;
		std::vector<double> scale;

		std::map<int, std::function<void(int, bool)>> switches;
		std::map<int, bool> button_state;

		int teleop = -1;
		int deadman = -1;
		bool omni = false;

		rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscriber;
		rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher;

		void joy_callback(sensor_msgs::msg::Joy::SharedPtr msg);

		void deadman_action(int button, bool state);
		void activate_action(int button, bool state);
		void explore_action(int button, bool state);
		void patrol_action(int button, bool state);
		void halt_action(int button, bool state);
		void spin_action(int button, bool state);
		void teleop_action(int button, bool state);
		void omni_action(int button, bool state);

		bool is_teleop()
		{
			return teleop != -1 && button_state[teleop];
		}

		bool is_deadman()
		{
			return deadman != -1 && button_state[deadman];
		}
};

}

#endif
