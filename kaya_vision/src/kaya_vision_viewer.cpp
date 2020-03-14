/*
 * kaya_vision_viewer.cpp
 *
 *  Created on: Mar 8, 2020
 *      Author: Stephen Street (stephen@redrocketcomputing.com)
 */

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <kaya_vision/kaya_vision_viewer.hpp>

static int encoding2mat_type(const std::string& encoding)
{
	if (encoding == "mono8") {
		return CV_8UC1;
	} else if (encoding == "bgr8" || encoding == "rgb8") {
		return CV_8UC3;
	} else if (encoding == "mono16") {
		return CV_16SC1;
	} else if (encoding == "rgba8") {
		return CV_8UC4;
	}

	throw std::runtime_error("Unsupported mat type");
}

kaya_vision::KayaVisionViewer::KayaVisionViewer(const rclcpp::NodeOptions& options) :
	Node("kaya_vision_viewer", options)
{
	camera_sub = create_subscription<sensor_msgs::msg::Image>("/camera/color/image_raw", rclcpp::SensorDataQoS(), std::bind(&KayaVisionViewer::image_callback, this, std::placeholders::_1));
}

kaya_vision::KayaVisionViewer::~KayaVisionViewer()
{

}

void kaya_vision::KayaVisionViewer::image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
	// Create a cv::Mat from the image message (without copying).
	cv::Mat cv_mat(msg->height, msg->width, encoding2mat_type(msg->encoding), msg->data.data());

	// Show the image.
	cv::Mat c_mat;
	cv::cvtColor(cv_mat, c_mat, cv::COLOR_RGB2BGR);
	cv::imshow(get_name(), c_mat);
	cv::waitKey(1);
}


