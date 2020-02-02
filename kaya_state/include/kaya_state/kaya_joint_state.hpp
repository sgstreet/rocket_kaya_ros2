/*
 * kaya_joint_state.hpp
 *
 *  Created on: Jan 4, 2020
 *      Author:  Stephen Street
 */

#ifndef _KAYA_JOINT_STATE_HPP_
#define _KAYA_JOINT_STATE_HPP_


#include <rclcpp/rclcpp.hpp>
#include <rclcpp/node.hpp>

#include <sensor_msgs/msg/joint_state.hpp>

namespace kaya_state {

class KayaJointState final : public rclcpp::Node
{
	public:
		explicit KayaJointState(const rclcpp::NodeOptions& options);
		~KayaJointState();

	private:

		void joint_state_timer_callback();

		sensor_msgs::msg::JointState joint_state;
		rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher;
		rclcpp::TimerBase::SharedPtr joint_state_timer;
};

}

#endif
