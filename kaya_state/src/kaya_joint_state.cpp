/*
 * kaya_joint_state.cpp
 *
 *  Created on: Jan 4, 2020
 *      Author:  Stephen Street
 */

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <kaya_state/kaya_joint_state.hpp>

kaya_state::KayaJointState::KayaJointState(const rclcpp::NodeOptions& options):
	rclcpp::Node("kaya_joint_state", options)
{
	// Initialize the joint state message, Really should read urdf, but need this fast
	joint_state.header.stamp = now();
	joint_state.name = { "wheel_1_joint", "wheel_2_joint", "wheel_3_joint" };
	joint_state.position = { 0.0, 0.0, 0.0 };
	joint_state.velocity = { 0.0, 0.0, 0.0 };
	joint_state.effort = { 0.0, 0.0, 0.0 };

	// Create the publisher
	joint_state_publisher = create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

	// Create the timer
	joint_state_timer = create_wall_timer(rclcpp::Rate(50).period(), std::bind(&KayaJointState::joint_state_timer_callback, this));
}

kaya_state::KayaJointState::~KayaJointState()
{
}

void kaya_state::KayaJointState::joint_state_timer_callback()
{
	joint_state.header.stamp = now();

	joint_state_publisher->publish(joint_state);
}

RCLCPP_COMPONENTS_REGISTER_NODE(kaya_state::KayaJointState)

