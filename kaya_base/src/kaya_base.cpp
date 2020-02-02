/*
 * kaya_base.cpp
 *
 *  Created on: Sep 2, 2019
 *      Author: Stephen Street (stephen@redrocketcomputing.com)
 */

#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include "kaya_base/kaya_base.hpp"

using namespace std::chrono_literals;

//joints:
//  u2d2:
//    port: "/dev/ttyUSB0"
//    baud_rate: 4000000
//  names: ["wheel_1_joint", "wheel_2_joint", "wheel_3_joint"]
//  ids: [1, 2, 3]
//  model: 1060
//  update_rate_hz: 50.0
//kinematics:
//  r: 0.041275
//  d: 0.116720
//  alpha: [ 1.047197551196598, 3.141592653589793, -1.047197551196598 ]

kaya_base::KayaBase::KayaBase(const rclcpp::NodeOptions& options) :
rclcpp_lifecycle::LifecycleNode("kaya_base", options)
{
	// Setup the paramters
	declare_parameter("joints.u2d2.port");
	declare_parameter("joints.u2d2.baud_rate");
	declare_parameter("joints.names");
	declare_parameter("joints.ids");
	declare_parameter("joints.model");
	declare_parameter("joints.update_rate_hz");
	declare_parameter("kinematics.r");
	declare_parameter("kinematics.d");
	declare_parameter("kinematics.alpha");

	// Construct the joints
	joints = std::make_unique<kaya_base::KayaJoints>(this);

	// Initialize the joint state message
	joint_state.name = get_parameter("joints.names").as_string_array();
	joint_state.header.stamp = now();
	joint_state.position = { 0.0, 0.0, 0.0 };
	joint_state.velocity = { 0.0, 0.0, 0.0 };
	joint_state.effort = { 0.0, 0.0, 0.0 };
}

kaya_base::KayaBase::~KayaBase()
{
}

kaya_base::KayaBase::CallbackReturn kaya_base::KayaBase::on_configure(const rclcpp_lifecycle::State& previous_state)
{
	RCLCPP_DEBUG(get_logger(), "on_configure invoked from %s\n", previous_state.label().c_str());

	// Create the joint period
	joint_period = rclcpp::Rate(get_parameter("joints.update_rate_hz").as_double()).period();

	// Extract the kinematic constants
	d = get_parameter("kinematics.d").as_double();
	r = get_parameter("kinematics.r").as_double();
	alpha = get_parameter("kinematics.alpha").as_double_array();

	// Initialize the velocity projection matrices
	J << 	sin(alpha[0]), -cos(alpha[0]), -d,
			sin(alpha[1]), -cos(alpha[1]), -d ,
			sin(alpha[2]), -cos(alpha[2]), -d;
	G = J.inverse();

	// Setup the publishers and subscribers
	joint_state_publisher = create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
	cmd_vel_subscriber = create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 10, std::bind(&KayaBase::cmd_vel_callback, this, std::placeholders::_1));

	return joints->init() < 0 ? CallbackReturn::FAILURE : CallbackReturn::SUCCESS;
}

kaya_base::KayaBase::CallbackReturn kaya_base::KayaBase::on_cleanup(const rclcpp_lifecycle::State& previous_state)
{
	RCLCPP_DEBUG(get_logger(), "on_cleanup invoked from %s\n", previous_state.label().c_str());

	// Cleanup the publishers and subscribers
	joint_state_publisher.reset();
	cmd_vel_subscriber.reset();

	// Disable the joints
	if (joints->disable() < 0)
		return CallbackReturn::FAILURE;

	return CallbackReturn::SUCCESS;
}

kaya_base::KayaBase::CallbackReturn kaya_base::KayaBase::on_shutdown(const rclcpp_lifecycle::State& previous_state)
{
	RCLCPP_DEBUG(get_logger(), "on_shutdown invoked from %s\n", previous_state.label().c_str());

	// Cleanup the publishers and subscribers
	joint_state_publisher.reset();
	cmd_vel_subscriber.reset();

	// Disable the joints
	if (joints->disable() < 0)
		return CallbackReturn::FAILURE;

	return CallbackReturn::SUCCESS;
}

kaya_base::KayaBase::CallbackReturn kaya_base::KayaBase::on_activate(const rclcpp_lifecycle::State& previous_state)
{
	RCLCPP_DEBUG(get_logger(), "on_activate invoked from %s\n", previous_state.label().c_str());

	// Enable the joints
	if (joints->enable() < 0)
		return CallbackReturn::FAILURE;

	// Enable the publisher
	joint_state_publisher->on_activate();

	// Start up the joint timer to pump the joint engine
	joint_timer = create_wall_timer(joint_period, std::bind(&KayaBase::drive_joints_callback, this));

	// All good
	return CallbackReturn::SUCCESS;
}

kaya_base::KayaBase::CallbackReturn kaya_base::KayaBase::on_deactivate(const rclcpp_lifecycle::State& previous_state)
{
	RCLCPP_DEBUG(get_logger(), "on_deactivate invoked from %s\n", previous_state.label().c_str());

	// Stop the joint engine
	joint_timer.reset();

	// Disable the joints
	if (joints->disable() < 0)
		return CallbackReturn::FAILURE;

	// Disable the joint state publisher
	joint_state_publisher->on_deactivate();

	// All good
	return CallbackReturn::SUCCESS;
}

kaya_base::KayaBase::CallbackReturn kaya_base::KayaBase::on_error(const rclcpp_lifecycle::State& previous_state)
{
	RCLCPP_DEBUG(get_logger(), "on_error invoked from %s\n", previous_state.label().c_str());

	// Cleanup the publishers and subscribers
	joint_state_publisher.reset();
	cmd_vel_subscriber.reset();
	joints->deinit();

	return CallbackReturn::FAILURE;
}

void kaya_base::KayaBase::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
	Eigen::Matrix<double, 3, 1> qdot;
	Eigen::Matrix<double, 3, 1> udot;

	// Extract the commanded velocity
	qdot << msg->linear.x, msg->linear.y, msg->angular.z;

	// Calculate the wheel velocities
	udot = (J * qdot).array() * (1 / r);

	// Set the joints
	joints->set_velocity({udot(0, 0), udot(1, 0), udot(2, 0)});
}

void kaya_base::KayaBase::drive_joints_callback()
{
	// Read joint data
	if (joints->read() < 0) {
		RCLCPP_ERROR(get_logger(), "problem reading joints");
		joints->disable();
		return;
	}

	// Write joint data
	if (joints->write() < 0) {
		RCLCPP_ERROR(get_logger(), "problem writing joints");
		joints->disable();
	}

	// Publish the joint state if anyone is interested
	if (joint_state_publisher->get_subscription_count() > 0) {
		auto position = joints->get_position();
		auto velocity = joints->get_velocity();
		auto effort = joints->get_effort();
		joint_state.header.stamp = now();
		joint_state.position.assign(position.begin(), position.end());
		joint_state.velocity.assign(velocity.begin(), velocity.end());
		joint_state.effort.assign(effort.begin(), effort.end());
		joint_state_publisher->publish(joint_state);
	}
}

RCLCPP_COMPONENTS_REGISTER_NODE(kaya_base::KayaBase)

