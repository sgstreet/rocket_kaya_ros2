/*
 * kaya_realsense_camera.hpp
 *
 *  Created on: Jan 11, 2020
 *      Author: Stephen Street (stephen@redrocketcomputing.com)
 */

#ifndef _KAYA_REALSENSE_CAMERA_HPP_
#define _KAYA_REALSENSE_CAMERA_HPP_

#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/node.hpp>

#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wignored-qualifiers"
#pragma GCC diagnostic ignored "-Wsign-compare"
#pragma GCC diagnostic ignored "-Wunused-function"

#include <librealsense2/rs.hpp>
#include <librealsense2/rs_advanced_mode.hpp>
#include <librealsense2/rsutil.h>

#pragma GCC diagnostic pop

namespace kaya_vision {

class KayaRealSenseCamera final : public rclcpp::Node
{
	public:
		explicit KayaRealSenseCamera(const rclcpp::NodeOptions& options);
		~KayaRealSenseCamera();

	private:
		struct StreamInfo
		{
			std::string frame_id;
			std::string image_encoding;
			std::pair<rs2_stream, int> stream_id;
			bool synthetic;
			geometry_msgs::msg::TransformStamped extrinsics;
			sensor_msgs::msg::CameraInfo intrinsics;
			rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub;
			rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_pub;
			rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr extrinsics_pub;
		};

		void stream_publisher();
		void extrinsics_publisher();
		void initialize_extrinsics(StreamInfo& stream, const rs2::stream_profile& base_profile, const rs2::stream_profile& target_profile);
		void initialize_intrinsics(StreamInfo& stream, const rs2::video_stream_profile& target_profile);
		void publish_video_frame(StreamInfo& stream, rs2::video_frame& frame, rclcpp::Time timestamp);

		std::chrono::nanoseconds extrinsics_pub_period;
		std::string base_link;
		std::string preset_json;
		std::map<std::string, StreamInfo> streams;

		rs2::config rs_config;
		rs2::pipeline pipeline;
		rs2::pipeline_profile profile;

		volatile bool running = true;
		std::unique_ptr<std::thread> stream_thread;
};

}

#endif
