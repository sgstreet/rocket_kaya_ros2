/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Stephen Street */
/* Author: Wim Meeussen */

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <std_msgs/msg/string.hpp>
#include <urdf/model.h>
#include <tf2/LinearMath/Quaternion.h>

#include <chrono>
#include <fstream>
#include <functional>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "kaya_state/kaya_state.hpp"

static inline geometry_msgs::msg::TransformStamped kdlToTransform(const KDL::Frame& k)
{
	geometry_msgs::msg::TransformStamped t;
	t.transform.translation.x = k.p.x();
	t.transform.translation.y = k.p.y();
	t.transform.translation.z = k.p.z();
	k.M.GetQuaternion(t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w);
	return t;
}

kaya_state::KayaState::KayaState(const rclcpp::NodeOptions& options) :
	rclcpp::Node("kaya_state", options)
{
	// Get the option urdf parameter
	std::string urdf_xml("");
	std::string urdf = declare_parameter("urdf", std::string(""));
	if (!urdf.empty()) {
		std::ifstream urdf_file(get_parameter("urdf").as_string());
		urdf_xml =  std::string(std::istreambuf_iterator<char>(urdf_file), std::istreambuf_iterator<char>());
		RCLCPP_INFO(get_logger(), "using urdf %s", get_parameter("urdf").as_string().c_str());
	}

	// get the XML
	urdf_xml = declare_parameter("robot_description", urdf_xml);
	if (urdf_xml.empty())
		throw std::runtime_error("robot_description parameter must be provided");

	// set publish frequency
	double publish_freq = declare_parameter("publish_frequency", 20.0);
	if (publish_freq > 1000.0)
		throw std::runtime_error("publish_frequency must be <= 1000.0");
	publish_interval_ms = std::chrono::milliseconds(static_cast<uint64_t>(1000.0 / publish_freq));

	// set whether to use the /tf_static latched static transform broadcaster
	use_tf_static = declare_parameter("use_tf_static", true);

	// ignore_timestamp_ == true, joint_state messages are accepted, no matter their timestamp
	ignore_timestamp = declare_parameter("ignore_timestamp", false);

	tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(this);
	static_tf_broadcaster = std::make_unique<tf2_ros::StaticTransformBroadcaster>(this);

	// Transient local is similar to latching in ROS 1.
	description_pub = create_publisher<std_msgs::msg::String>("robot_description", rclcpp::QoS(1).transient_local());

	// Set up the urdf
	setupURDF(urdf_xml);

	// Extract the kinematic constants and initialize the inverse wheel velocity projection
	double d = declare_parameter("kinematics.d", NAN);
	if (isnan(d))
		throw std::runtime_error("kinematics.d not found");
	double r = declare_parameter("kinematics.r", NAN);
	if (isnan(r))
		throw std::runtime_error("kinematics.r not found");
	std::vector<double> alpha = declare_parameter("kinematics.alpha", std::vector<double>());
	if (alpha.size() == 0)
		throw std::runtime_error("kinematics.alpha not found");
	Eigen::Matrix3d J;
	J << 	sin(alpha[0])/r, -cos(alpha[0])/r, -d/r ,
			sin(alpha[1])/r, -cos(alpha[1])/r, -d/r ,
			sin(alpha[2])/r, -cos(alpha[2])/r, -d/r;
	G = J.inverse();

	// Create the odometry publisher
	odometry_pub = create_publisher<nav_msgs::msg::Odometry>("odom", 10);

	// Initialize the odometery
	last_q.Zero();
	odometry.header.frame_id = "odom";
	odometry.child_frame_id = "base_link";

	// subscribe to joint state
	joint_state_sub = create_subscription<sensor_msgs::msg::JointState>("joint_states", 10, std::bind(&KayaState::callbackJointState, this, std::placeholders::_1));

	// Now that we have successfully declared the parameters and done all
	// necessary setup, install the callback for updating parameters.
	param_cb = add_on_set_parameters_callback(std::bind(&KayaState::parameterUpdate, this, std::placeholders::_1));

	// trigger to publish fixed joints
	timer = create_wall_timer(publish_interval_ms, std::bind(&KayaState::publishFixedTransforms, this));
}

kaya_state::KayaState::~KayaState()
{
}

void kaya_state::KayaState::setupURDF(const std::string& urdf_xml)
{
	model = std::make_unique<urdf::Model>();

	// Initialize the model
	if (!model->initString(urdf_xml))
		throw std::runtime_error("Unable to initialize urdf::model from robot description");

	// Initialize the KDL tree
	KDL::Tree tree;
	if (!kdl_parser::treeFromUrdfModel(*model, tree))
		throw std::runtime_error("Failed to extract kdl tree from robot description");

	// Initialize the mimic map
	mimic.clear();
	for (const std::pair<std::string, urdf::JointSharedPtr> &i : model->joints_)
		if (i.second->mimic)
			mimic.insert(std::make_pair(i.first, i.second->mimic));

	KDL::SegmentMap segments_map = tree.getSegments();
	for (const std::pair<std::string, KDL::TreeElement> &segment : segments_map)
		RCLCPP_INFO(get_logger(), "got segment %s", segment.first.c_str());

	// walk the tree and add segments to segments_
	segments.clear();
	segments_fixed.clear();
	addChildren(tree.getRootSegment());

	auto msg = std::make_unique<std_msgs::msg::String>();
	msg->data = urdf_xml;

	// Publish the robot description
	description_pub->publish(std::move(msg));
}

// add children to correct maps
void kaya_state::KayaState::addChildren(const KDL::SegmentMap::const_iterator segment)
{
	const std::string& root = GetTreeElementSegment(segment->second).getName();

	std::vector<KDL::SegmentMap::const_iterator> children = GetTreeElementChildren(segment->second);
	for (unsigned int i = 0; i < children.size(); i++) {
		const KDL::Segment &child = GetTreeElementSegment(children[i]->second);
		SegmentPair s(GetTreeElementSegment(children[i]->second), root, child.getName());
		if (child.getJoint().getType() == KDL::Joint::None) {
			if (model->getJoint(child.getJoint().getName()) && model->getJoint(child.getJoint().getName())->type == urdf::Joint::FLOATING)
				RCLCPP_INFO(get_logger(), "Floating joint. Not adding segment from %s to %s.", root.c_str(), child.getName().c_str());
			else {
				segments_fixed.insert(make_pair(child.getJoint().getName(), s));
				RCLCPP_DEBUG(get_logger(), "Adding fixed segment from %s to %s", root.c_str(), child.getName().c_str());
			}
		} else {
			segments.insert(make_pair(child.getJoint().getName(), s));
			RCLCPP_DEBUG(get_logger(), "Adding moving segment from %s to %s", root.c_str(), child.getName().c_str());
		}
		addChildren(children[i]);
	}
}

void kaya_state::KayaState::publishTransforms(const std::map<std::string, double>& joint_positions, const Eigen::Vector3d& q, const builtin_interfaces::msg::Time& time)
{
	RCLCPP_DEBUG(get_logger(), "Publishing transforms for moving joints");
	std::vector<geometry_msgs::msg::TransformStamped> tf_transforms;

	// loop over all joints
	for (const std::pair<std::string, double> &jnt : joint_positions) {
		std::map<std::string, SegmentPair>::iterator seg = segments.find(jnt.first);
		if (seg != segments.end()) {
			geometry_msgs::msg::TransformStamped tf_transform = kdlToTransform(seg->second.segment.pose(jnt.second));
			tf_transform.header.stamp = time;
			tf_transform.header.frame_id = seg->second.root;
			tf_transform.child_frame_id = seg->second.tip;
			tf_transforms.push_back(tf_transform);
		}
	}


	// Add the odometry transform. since all odometry is 6DOF we'll need a quaternion created from yaw
	tf2::Quaternion quaternion;
	quaternion.setRPY(0.0, 0.0, q(2,0));
	geometry_msgs::msg::TransformStamped odom_trans;
	odom_trans.header.stamp = time;
	odom_trans.header.frame_id = "odom";
	odom_trans.child_frame_id = "base_link";
	odom_trans.transform.translation.x = q(0,0);
	odom_trans.transform.translation.y = q(1,0);
	odom_trans.transform.translation.z = 0;
	odom_trans.transform.rotation.x = quaternion.x();
	odom_trans.transform.rotation.y = quaternion.y();
	odom_trans.transform.rotation.z = quaternion.z();
	odom_trans.transform.rotation.w = quaternion.w();
	tf_transforms.push_back(odom_trans);

	// Send it on the way
	tf_broadcaster->sendTransform(tf_transforms);
}

void kaya_state::KayaState::publishFixedTransforms()
{
	RCLCPP_DEBUG(get_logger(), "Publishing transforms for fixed joints");
	std::vector<geometry_msgs::msg::TransformStamped> tf_transforms;
	geometry_msgs::msg::TransformStamped tf_transform;

	// loop over all fixed segments
	for (const std::pair<std::string, SegmentPair> &seg : segments_fixed) {
		geometry_msgs::msg::TransformStamped tf_transform = kdlToTransform(seg.second.segment.pose(0));
		rclcpp::Time now = this->now();
		if (!use_tf_static)
			now = now + rclcpp::Duration(std::chrono::milliseconds(500));
		tf_transform.header.stamp = now;

		tf_transform.header.frame_id = seg.second.root;
		tf_transform.child_frame_id = seg.second.tip;
		tf_transforms.push_back(tf_transform);
	}
	if (use_tf_static)
		static_tf_broadcaster->sendTransform(tf_transforms);
	else
		tf_broadcaster->sendTransform(tf_transforms);
}

void kaya_state::KayaState::callbackJointState(const sensor_msgs::msg::JointState::SharedPtr state)
{
	if (state->name.size() != state->position.size()) {
		if (state->position.empty())
			RCLCPP_WARN(get_logger(), "Robot state publisher ignored a JointState message about joint(s) \"%s\"(,...) whose position member was empty.", state->name[0].c_str());
		else
			RCLCPP_ERROR(get_logger(), "Robot state publisher ignored an invalid JointState message");
		return;
	}

	// check if we moved backwards in time (e.g. when playing a bag file)
	rclcpp::Time now = this->now();
	if (last_callback_time.nanoseconds() > now.nanoseconds()) {
		// Force re-publish of joint transforms
		RCLCPP_WARN(get_logger(), "Moved backwards in time, re-publishing joint transforms!");
		last_publish_time.clear();
	}
	last_callback_time = now;

	// determine least recently published joint
	rclcpp::Time last_published = now;
	for (size_t i = 0; i < state->name.size(); i++) {
		rclcpp::Time t(last_publish_time[state->name[i]]);
		last_published = (t.nanoseconds() < last_published.nanoseconds()) ? t : last_published;
	}

	// Check if we need to publish
	// note: if a joint was seen for the first time, then last_published is zero.
	rclcpp::Time current_time(state->header.stamp);
	rclcpp::Time max_publish_time = last_published + rclcpp::Duration(publish_interval_ms);
	if (ignore_timestamp || current_time.nanoseconds() >= max_publish_time.nanoseconds()) {

		// get joint positions from state message
		std::map<std::string, double> joint_positions;
		for (size_t i = 0; i < state->name.size(); i++)
			joint_positions.insert(std::make_pair(state->name[i], state->position[i]));

		for (const std::pair<std::string, urdf::JointMimicSharedPtr> &i : mimic)
			if (joint_positions.find(i.second->joint_name) != joint_positions.end()) {
				double pos = joint_positions[i.second->joint_name] * i.second->multiplier + i.second->offset;
				joint_positions.insert(std::make_pair(i.first, pos));
			}

		// Calculate the odometry
		Eigen::Vector3d u(joint_positions["wheel_1_joint"], joint_positions["wheel_2_joint"] , joint_positions["wheel_3_joint"]);
		Eigen::Vector3d delta_u = u - last_u;
		Eigen::Vector3d vb = G * delta_u;
		double vbx = vb(0,0);
		double vby = vb(1,0);
		double vbw = vb(2,0);
		Eigen::Vector3d delta_qb;
		if (vbw != 0) {
			double sbw = sin(vbw);
			double cbw = cos(vbw);
			delta_qb << (vbx * sbw + vby * (cbw - 1)) / vbw,
						(vby * sbw + vbx * (1 - cbw)) / vbw,
						vbw;
		} else
			delta_qb << vbx, vby, 0;

		Eigen::Matrix3d tsb;
		double ctheta = cos(last_q(2,0));
		double stheta = sin(last_q(2,0));
		tsb << 	ctheta, -stheta, 0,
				stheta, ctheta, 0,
				0, 0, 1;
		Eigen::Vector3d delta_q = tsb * delta_qb;
		Eigen::Vector3d q = last_q + delta_q;

		// Publish the transforms
		publishTransforms(joint_positions, q, state->header.stamp);

		// Assemble the odometry msg
		odometry.header.stamp = current_time;
		odometry.pose.pose.position.x = q(0,0);
		odometry.pose.pose.position.y = q(1,0);
		odometry.pose.pose.orientation.z = q(2,0);
		odometry.twist.twist.linear.x = delta_q(0,0);
		odometry.twist.twist.linear.y = delta_q(1,0);
		odometry.twist.twist.angular.z = delta_q(2,0);

		// Publish it
		odometry_pub->publish(odometry);

		// Update the odometry state
		last_q = q;
		last_u = u;

		// Store publish time in joint map
		for (size_t i = 0; i < state->name.size(); i++)
			last_publish_time[state->name[i]] = state->header.stamp;
	}
}

rcl_interfaces::msg::SetParametersResult kaya_state::KayaState::parameterUpdate(const std::vector<rclcpp::Parameter>& parameters)
{
	rcl_interfaces::msg::SetParametersResult result;
	result.successful = true;

	for (const rclcpp::Parameter &parameter : parameters) {
		if (parameter.get_name() == "robot_description") {
			// First make sure that it is still a string
			if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_STRING) {
				result.successful = false;
				result.reason = "URDF must be a string";
				break;
			}

			// Now get the parameter
			std::string new_urdf = parameter.as_string();
			// And ensure that it isn't empty
			if (new_urdf.empty()) {
				result.successful = false;
				result.reason = "Empty URDF is not allowed";
				break;
			}

			// Re-initialize the kdl tree
			setupURDF(new_urdf);

		} else if (parameter.get_name() == "use_tf_static") {
			if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_BOOL) {
				result.successful = false;
				result.reason = "use_tf_static must be a boolean";
				break;
			}
			use_tf_static = parameter.as_bool();
		} else if (parameter.get_name() == "ignore_timestamp") {
			if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_BOOL) {
				result.successful = false;
				result.reason = "ignore_timestamp must be a boolean";
				break;
			}
			ignore_timestamp = parameter.as_bool();
		} else if (parameter.get_name() == "publish_frequency") {
			if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
				result.successful = false;
				result.reason = "publish_frequency must be a double";
				break;
			}

			double publish_freq = parameter.as_double();
			if (publish_freq > 1000.0) {
				result.successful = false;
				result.reason = "publish_frequency must be <= 1000.0";
				break;
			}
			std::chrono::milliseconds new_publish_interval = std::chrono::milliseconds(static_cast<uint64_t>(1000.0 / publish_freq));

			if (new_publish_interval != publish_interval_ms) {
				publish_interval_ms = new_publish_interval;
				timer->cancel();
				timer = this->create_wall_timer(publish_interval_ms, std::bind(&KayaState::publishFixedTransforms, this));
			}
		}
	}

	return result;
}

RCLCPP_COMPONENTS_REGISTER_NODE(kaya_state::KayaState)
