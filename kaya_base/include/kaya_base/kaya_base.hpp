/*
 * kaya_base.hpp
 *
 *  Created on: Sep 2, 2019
 *      Author: Stephen Street (stephen@redrocketcomputing.com)
 */

#ifndef _KAYA_BASE_HPP_
#define _KAYA_BASE_HPP_

#include <string>
#include <memory>
#include <chrono>

#include <eigen3/Eigen/Dense>

#include <rclcpp/rclcpp.hpp>

#include <lifecycle_msgs/msg/transition.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include <kaya_base/kaya_joints.hpp>

namespace kaya_base {

class KayaBase : public rclcpp_lifecycle::LifecycleNode
{
	public:
		using rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

		explicit KayaBase(const rclcpp::NodeOptions& options);
		virtual ~KayaBase();

		virtual CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state);
		virtual CallbackReturn on_cleanup(const rclcpp_lifecycle::State& previous_state);
		virtual CallbackReturn on_shutdown(const rclcpp_lifecycle::State& previous_state);
		virtual CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state);
		virtual CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state);
		virtual CallbackReturn on_error(const rclcpp_lifecycle::State& previous_state);

	private:
		void drive_joints_callback();
		void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);

		std::unique_ptr<kaya_base::KayaJoints> joints;
		sensor_msgs::msg::JointState joint_state;
		std::chrono::nanoseconds joint_period;

		std::vector<double> alpha;
		double d;
		double r;
		Eigen::Matrix3d J;
		Eigen::Matrix3d G;
		rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscriber;
		rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher;
		rclcpp::TimerBase::SharedPtr joint_timer;
};

}

#endif
