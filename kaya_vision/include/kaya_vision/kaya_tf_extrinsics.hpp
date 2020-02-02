/*
 * kaya_tf_extrinsics.hpp
 *
 *  Created on: Jan 18, 2020
 *      Author: Stephen Street (stephen@redrocketcomputing.com)
 */

#ifndef _KAYA_TF_EXTRINSICS_HPP_
#define _KAYA_TF_EXTRINSICS_HPP_


#include <rclcpp/rclcpp.hpp>
#include <rclcpp/node.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>

#include <tf2_ros/static_transform_broadcaster.h>

namespace kaya_vision {

class KayaTFExtinsics final : public rclcpp::Node
{
	public:
		explicit KayaTFExtinsics(const rclcpp::NodeOptions& options);
		~KayaTFExtinsics();

	private:
		void extrinsics_callback(const geometry_msgs::msg::TransformStamped::SharedPtr msg);

		std::vector<rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::SharedPtr> extrinsics_subs;
		std::unique_ptr<tf2_ros::StaticTransformBroadcaster> transform_broadcaster;
};

}

#endif
