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

#ifndef _KAYA_STATE_HPP_
#define _KAYA_STATE_HPP_

#include <chrono>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <eigen3/Eigen/Dense>

#include <kdl/tree.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <urdf/model.h>


typedef std::map<std::string, urdf::JointMimicSharedPtr> MimicMap;

namespace kaya_state {

class SegmentPair final
{
	public:
		explicit SegmentPair(const KDL::Segment &p_segment, const std::string &p_root, const std::string &p_tip) :
			segment(p_segment), root(p_root), tip(p_tip)
		{
		}

		KDL::Segment segment;
		std::string root;
		std::string tip;
};

class KayaState final : public rclcpp::Node
{
	public:
		explicit KayaState(const rclcpp::NodeOptions& options);
		~KayaState();

	private:
		void publishFixedTransforms();
		void publishTransforms(const std::map<std::string, double>& joint_positions, const Eigen::Vector3d& q, const builtin_interfaces::msg::Time& time);
		void addChildren(const KDL::SegmentMap::const_iterator segment);
		void callbackJointState(const sensor_msgs::msg::JointState::SharedPtr state);
		void setupURDF(const std::string &urdf_xml);

		rcl_interfaces::msg::SetParametersResult parameterUpdate(const std::vector<rclcpp::Parameter> &parameters);

		bool use_tf_static;
		bool ignore_timestamp;
		std::chrono::milliseconds publish_interval_ms;

		std::unique_ptr<urdf::Model> model;

		MimicMap mimic;
		std::map<std::string, SegmentPair> segments;
		std::map<std::string, SegmentPair> segments_fixed;

		rclcpp::TimerBase::SharedPtr timer;
		rclcpp::Time last_callback_time;
		std::map<std::string, builtin_interfaces::msg::Time> last_publish_time;

		Eigen::Matrix3d G;
		Eigen::Vector3d last_u;
		Eigen::Vector3d last_q;
		nav_msgs::msg::Odometry odometry;

		rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub;

		std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
		std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster;
		rclcpp::Publisher<std_msgs::msg::String>::SharedPtr description_pub;
		rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_pub;

		rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb;
};

}

#endif
