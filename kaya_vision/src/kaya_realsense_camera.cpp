/*
 * kaya_realsense_camera.cpp
 *
 *  Created on: Jan 11, 2020
 *      Author: Stephen Street (stephen@redrocketcomputing.com)
 */

#include <fstream>

#include <eigen3/Eigen/Dense>
#include <opencv2/opencv.hpp>

#include <resource_retriever/retriever.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <cv_bridge/cv_bridge.h>
#include <tf2/LinearMath/Quaternion.h>

#include <kaya_vision/kaya_realsense_camera.hpp>

using namespace std::chrono_literals;

struct StreamConfiguration
{
	rs2_stream stream_id;
	int stream_index;
	rs2_format stream_format;
	std::string ros_encoding;
	int opencv_encoding;
	std::string frame_id;
	std::string sensor_topic;
	std::string info_topic;
	std::string extrinsics_topic;
};

static const std::vector<std::string> sensor_names = { "infra1", "infra2", "depth", "color", "aligned", "pose", "imu" };

static const std::map<std::string, rs2_format> to_rs2_format =
{
	{ "RS2_FORMAT_Z16", RS2_FORMAT_Z16 },
	{ "RS2_FORMAT_DISPARITY16",  RS2_FORMAT_DISPARITY16 },
	{ "RS2_FORMAT_XYZ32F",  RS2_FORMAT_XYZ32F },
	{ "RS2_FORMAT_YUYV",  RS2_FORMAT_YUYV },
	{ "RS2_FORMAT_RGB8",  RS2_FORMAT_RGB8 },
	{ "RS2_FORMAT_BGR8",  RS2_FORMAT_BGR8 },
	{ "RS2_FORMAT_RGBA8",  RS2_FORMAT_RGBA8 },
	{ "RS2_FORMAT_BGRA8",  RS2_FORMAT_BGRA8 },
	{ "RS2_FORMAT_Y8",  RS2_FORMAT_Y8 },
	{ "RS2_FORMAT_Y16",  RS2_FORMAT_Y16 },
	{ "RS2_FORMAT_RAW10",  RS2_FORMAT_RAW10 },
	{ "RS2_FORMAT_RAW16",  RS2_FORMAT_RAW16 },
	{ "RS2_FORMAT_RAW8",  RS2_FORMAT_RAW8 },
	{ "RS2_FORMAT_UYVY",  RS2_FORMAT_UYVY },
	{ "RS2_FORMAT_MOTION_RAW",  RS2_FORMAT_MOTION_RAW },
	{ "RS2_FORMAT_MOTION_XYZ32F",  RS2_FORMAT_MOTION_XYZ32F },
	{ "RS2_FORMAT_GPIO_RAW",  RS2_FORMAT_GPIO_RAW },
	{ "RS2_FORMAT_6DOF",  RS2_FORMAT_6DOF },
	{ "RS2_FORMAT_DISPARITY32",  RS2_FORMAT_DISPARITY32 },
	{ "RS2_FORMAT_Y10BPACK",  RS2_FORMAT_Y10BPACK },
	{ "RS2_FORMAT_DISTANCE",  RS2_FORMAT_DISTANCE },
	{ "RS2_FORMAT_MJPEG",  RS2_FORMAT_MJPEG },
	{ "RS2_FORMAT_Y8I",  RS2_FORMAT_Y8I },
	{ "RS2_FORMAT_Y12I",  RS2_FORMAT_Y12I },
	{ "RS2_FORMAT_INZI",  RS2_FORMAT_INZI },
	{ "RS2_FORMAT_INVI",  RS2_FORMAT_INVI },
	{ "RS2_FORMAT_W10",  RS2_FORMAT_W10 },
	{ "RS2_FORMAT_Z16H",  RS2_FORMAT_Z16H },
};

static const std::map<std::string, rs2_stream> to_rs2_stream =
{
	{ "RS2_STREAM_DEPTH", RS2_STREAM_DEPTH },
	{ "RS2_STREAM_COLOR", RS2_STREAM_COLOR },
	{ "RS2_STREAM_INFRARED", RS2_STREAM_INFRARED },
	{ "RS2_STREAM_FISHEYE", RS2_STREAM_FISHEYE },
	{ "RS2_STREAM_GYRO", RS2_STREAM_GYRO },
	{ "RS2_STREAM_ACCEL", RS2_STREAM_ACCEL },
	{ "RS2_STREAM_GPIO", RS2_STREAM_GPIO },
	{ "RS2_STREAM_POSE", RS2_STREAM_POSE },
	{ "RS2_STREAM_CONFIDENCE", RS2_STREAM_CONFIDENCE },
};

static const std::map<rs2_stream, int> to_cv_format =
{
	{ RS2_STREAM_COLOR, CV_8UC3 },
	{ RS2_STREAM_DEPTH, CV_16UC1 },
	{ RS2_STREAM_INFRARED, CV_8UC1 },
	{ RS2_STREAM_FISHEYE, CV_8UC1 },
};

static const std::map<rs2_stream, std::string> to_image_encoding =
{
	{ RS2_STREAM_COLOR, sensor_msgs::image_encodings::RGB8 },
	{ RS2_STREAM_DEPTH, sensor_msgs::image_encodings::TYPE_16UC1 },
	{ RS2_STREAM_INFRARED, sensor_msgs::image_encodings::TYPE_8UC1 },
	{ RS2_STREAM_FISHEYE, sensor_msgs::image_encodings::TYPE_8UC1 },
};

//static const std::map<std::string, StreamConfiguration> stream_configuration_map = {
//	{"aligned", {RS2_STREAM_DEPTH, -1, RS2_FORMAT_Z16, sensor_msgs::image_encodings::TYPE_16UC1, CV_16UC1, "camera_aligned_optical", "camera/aligned/image_rect_raw", "camera/aligned/camera_info", "camera/aligned/extrinsics"}},
//	{"depth", {RS2_STREAM_DEPTH, -1, RS2_FORMAT_Z16, sensor_msgs::image_encodings::TYPE_16UC1, CV_16UC1, "camera_depth_optical", "camera/depth/image_rect_raw", "camera/depth/camera_info", "camera/depth/extrinsics"}},
//	{"color", {RS2_STREAM_COLOR, -1, RS2_FORMAT_RGB8, sensor_msgs::image_encodings::RGB8, CV_8UC3, "camera_color_optical", "camera/color/image_raw", "camera/color/camera_info", "camera/color/extrinsics"}},
//	{"infra1", {RS2_STREAM_INFRARED, 1, RS2_FORMAT_Y8, sensor_msgs::image_encodings::TYPE_8UC1, CV_8UC1, "camera_infra1_optical", "camera/infra1/image_rect_raw", "camera/infra1/camera_info", "camera/infra1/extrinsics"}},
//	{"infra2", {RS2_STREAM_INFRARED, 2, RS2_FORMAT_Y8, sensor_msgs::image_encodings::TYPE_8UC1, CV_8UC1, "camera_infra2_optical", "camera/infra2/image_rect_raw", "camera/infra2/camera_info", "camera/infra2/extrinsics"}},
//	{"accel", {RS2_STREAM_ACCEL, -1, RS2_FORMAT_MOTION_XYZ32F, std::string(), INT_MIN, "camera_accel_optical", "camera/accel/sample", "camera/accel/imu_info", std::string()}},
//	{"gyro", {RS2_STREAM_GYRO, -1, RS2_FORMAT_MOTION_XYZ32F, std::string(), INT_MIN, "camera_gyro_optical", "camera/gyro/sample", "camera/gyro/imu_info", std::string()}},
//};

static std::pair<rs2_stream, int> to_stream_id(const std::string& value)
{

	// Is there and index specifications?
	std::string id_string = value;
	int index = -1;
	auto index_pos = id_string.find('@');
	if (index_pos != std::string::npos) {
		index = std::stoi(id_string.substr(index_pos + 1));
		id_string = id_string.substr(0, index_pos);
	}

	// Now decode the string
	auto entry = to_rs2_stream.find(id_string);
	if (entry != to_rs2_stream.end())
		return { entry->second, index };
	else
		return { RS2_STREAM_ANY, index };
}

rs2_format to_stream_format(const std::string& value)
{
	// Now decode the string
	auto entry = to_rs2_format.find(value);
	if (entry != to_rs2_format.end())
		return entry->second;
	else
		return RS2_FORMAT_ANY;
}

static tf2::Quaternion to_quaternion(const float rotation[9])
{
	// This is from ros2_realsense
	// We need to be careful about the order, as RS2 rotation matrix is column-major, while Eigen::Matrix3f expects row-major.
	Eigen::Matrix3f m;
	m << rotation[0], rotation[3], rotation[6],
		 rotation[1], rotation[4], rotation[7],
		 rotation[2], rotation[5], rotation[8];

	Eigen::Quaternionf q(m);

	return tf2::Quaternion(q.x(), q.y(), q.z(), q.w());
}

kaya_vision::KayaRealSenseCamera::KayaRealSenseCamera(const rclcpp::NodeOptions& options) :
	rclcpp::Node("kaya_realsense_camera", options)
{
	// Declare the parameters
	declare_parameter("serial_no", "");
	declare_parameter("preset", "");
	declare_parameter("base_frame_id", "camera_link");
	declare_parameter("extrinsics_rate_hz", 1.0);
	declare_parameter("extrinsics_reference", "infra1");
	declare_parameter("camera.resolution", std::vector<int64_t>{848,480});
	declare_parameter("camera.fps", 30);
	for (auto const &sensor_name : sensor_names) {
		declare_parameter("camera." + sensor_name + ".enabled", false);
		declare_parameter("camera." + sensor_name + ".synthetic", sensor_name == "aligned");
		declare_parameter("camera." + sensor_name + ".frame_id", "");
		declare_parameter("camera." + sensor_name + ".rs_stream", "");
		declare_parameter("camera." + sensor_name + ".rs_encoding", "");
		declare_parameter("camera." + sensor_name + ".image_encoding", "");
		declare_parameter("camera." + sensor_name + ".image_topic", "");
		declare_parameter("camera." + sensor_name + ".info_topic", "");
		declare_parameter("camera." + sensor_name + ".extrinsics_topic", "");
	}

	// Extract the base information
	std::string serial_number = get_parameter("serial_no").as_string();
	std::string preset_filename = get_parameter("preset").as_string();
	base_link = get_parameter("base_frame_id").as_string();
	extrinsics_pub_period = rclcpp::Rate(get_parameter("extrinsics_rate_hz").as_double()).period();
	std::string extrinsics_reference = get_parameter("extrinsics_reference").as_string();

	std::vector resolution = get_parameter("camera.resolution").as_integer_array();
	int fps = get_parameter("camera.fps").as_int();

	// Try of find a sensor
	if (serial_number.length() > 0)
		rs_config.enable_device(serial_number);

	// Load any preset json
	if (preset_filename.length() > 0) {
		resource_retriever::MemoryResource resource = resource_retriever::Retriever().get(preset_filename);
		preset_json = std::string(reinterpret_cast<char *>(resource.data.get()), resource.size);
	}

	// Build up the configuration
	for (auto const &sensor_name : sensor_names) {

		// If the sensor is enabled
		if (get_parameter("camera." + sensor_name + ".enabled").as_bool()) {

			// Extract the base stream info
			StreamInfo stream_info;
			stream_info.stream_id = to_stream_id(get_parameter("camera." + sensor_name + ".rs_stream").as_string());
			stream_info.frame_id = get_parameter("camera." + sensor_name + ".frame_id").as_string();
			stream_info.image_encoding = get_parameter("camera." + sensor_name + ".image_encoding").as_string();
			stream_info.synthetic = get_parameter("camera." + sensor_name + ".synthetic").as_bool();

			// Create the publishers
			stream_info.image_pub = create_publisher<sensor_msgs::msg::Image>(get_parameter("camera." + sensor_name + ".image_topic").as_string(), 1);
			stream_info.info_pub = create_publisher<sensor_msgs::msg::CameraInfo>(get_parameter("camera." + sensor_name + ".info_topic").as_string(), 1);
			stream_info.extrinsics_pub = create_publisher<geometry_msgs::msg::TransformStamped>(get_parameter("camera." + sensor_name + ".extrinsics_topic").as_string(), 1);

			// Add to realsense configuration is not a synthetic stream
			if (!stream_info.synthetic)
				rs_config.enable_stream(stream_info.stream_id.first, stream_info.stream_id.second, resolution.at(0), resolution.at(1), to_stream_format(get_parameter("camera." + sensor_name + ".rs_encoding").as_string()), fps);

			// Save the stream info
			streams[sensor_name] = stream_info;
		}
	}

	// Everything else requires the pipeline to be running, so start it up
	profile = pipeline.start(rs_config);

	// Load the preset if available
	if (preset_json.length() > 0)
		profile.get_device().as<rs400::advanced_mode>().load_json(preset_json);

	// Configure the sensor attributes
	auto sensors = profile.get_device().query_sensors();
	for (auto & sensor : sensors) {
		// Enable the global time stamp
		if (sensor.supports(RS2_OPTION_GLOBAL_TIME_ENABLED))
			sensor.set_option(RS2_OPTION_GLOBAL_TIME_ENABLED, 1);

		// Disable auto exposure priority
		if (sensor.supports(RS2_OPTION_AUTO_EXPOSURE_PRIORITY))
			sensor.set_option(RS2_OPTION_AUTO_EXPOSURE_PRIORITY, 0);

	}

	// To initialize the extrinsics we need the profile of the reference frame
	rs2::stream_profile base_profile;
	if (get_parameter("camera.depth.enabled").as_bool())
		base_profile = profile.get_stream(RS2_STREAM_DEPTH);
	else
	    RCLCPP_WARN(get_logger(), "No TF is available, enable depth stream first");

	// If we got a base profile initialize the extrinsics
	if (base_profile)
		for (auto & [sensor_name, info] : streams) {
			if (!info.synthetic) {
				rs2::stream_profile target_profile = profile.get_stream(info.stream_id.first, info.stream_id.second);
				std::string target_frame_id = get_parameter("camera." + sensor_name + ".frame_id").as_string();
				initialize_extrinsics(info, base_profile, target_profile);
				if (target_profile.is<rs2::video_stream_profile>())
					initialize_intrinsics(info, target_profile.as<rs2::video_stream_profile>());
			}
		}

	// Ready to go, start the camera publisher thread
	stream_thread = std::make_unique<std::thread>(&KayaRealSenseCamera::stream_publisher, this);
}

kaya_vision::KayaRealSenseCamera::~KayaRealSenseCamera()
{
	// Shutdown the thread
	if (stream_thread->joinable()) {
		running = false;
		stream_thread->join();
	}
}

void kaya_vision::KayaRealSenseCamera::stream_publisher()
{
	// Start the extrinsics publisher
	auto extrinsics_timer = create_wall_timer(extrinsics_pub_period, std::bind(&KayaRealSenseCamera::extrinsics_publisher, this));

	// Loop until canceled
	while (running) {

		// Grab the frame set
		rs2::frameset frameset = pipeline.wait_for_frames();

		// Capture the timestamp
		auto timestamp = now();

		// Loop through all the streams
		for (auto & [name, stream] : streams) {

			// Find the matching frame in the frame set
			switch (stream.stream_id.first) {
				case RS2_STREAM_DEPTH: {
					auto frame = frameset.get_depth_frame();
					publish_video_frame(stream, frame, timestamp);
					break;
				}
				case RS2_STREAM_COLOR: {
					auto frame = frameset.get_color_frame();
					publish_video_frame(stream, frame, timestamp);
					break;
				}
				case RS2_STREAM_INFRARED: {
					auto frame = frameset.get_infrared_frame(stream.stream_id.second);
					publish_video_frame(stream, frame, timestamp);
					break;
				}
				default: {
					RCLCPP_WARN(get_logger(), "unhandled frame type %s on stream %s\n", name.c_str(), rs2_stream_to_string(stream.stream_id.first));
					break;
				}
			}
		}
	}
}

void kaya_vision::KayaRealSenseCamera::extrinsics_publisher()
{
	for (auto & entry : streams) {
		if (entry.second.extrinsics_pub->get_subscription_count() > 0) {
			entry.second.extrinsics.header.stamp = now();
			entry.second.extrinsics_pub->publish(entry.second.extrinsics);
		}
	}
}

void kaya_vision::KayaRealSenseCamera::publish_video_frame(StreamInfo& stream, rs2::video_frame& frame, rclcpp::Time timestamp)
{
	// Is the a subscriber
	if (stream.image_pub->get_subscription_count() > 0 || stream.info_pub->get_subscription_count() > 0) {

		// Extract the frame data as an opencv matrix, zero copy
		cv::Mat cv_image = cv::Mat(frame.get_height(), frame.get_width(), to_cv_format.at(stream.stream_id.first));
		cv_image.data = const_cast<uchar *>(reinterpret_cast<const uchar *>(frame.get_data()));

		// Build the ros image message
		auto ros_image = std::make_unique<sensor_msgs::msg::Image>();
		cv_bridge::CvImage(std_msgs::msg::Header(), to_image_encoding.at(stream.stream_id.first), cv_image).toImageMsg(*ros_image);
		ros_image->header.frame_id = stream.frame_id;
		ros_image->header.stamp = timestamp;

		// Send it on the way
		stream.image_pub->publish(std::move(ros_image));

		// Publish the camera info
		stream.intrinsics.header.stamp = timestamp;
		stream.info_pub->publish(stream.intrinsics);
	}
}

void kaya_vision::KayaRealSenseCamera::initialize_extrinsics(StreamInfo& stream, const rs2::stream_profile& base_profile, const rs2::stream_profile& target_profile)
{
	// Get the matching extrinsic
	rs2_extrinsics camera_extrinsics = {{1, 0, 0, 0, 1, 0, 0, 0, 1}, {0,0,0}};
	if (target_profile.stream_name() != base_profile.stream_name())
		camera_extrinsics = target_profile.get_extrinsics_to(base_profile);

	// Build up relative rotation from the extrinsic, clean up if not an optical frame
	tf2::Quaternion Q;
	Q.setRPY(-M_PI / 2, 0.0, -M_PI / 2);
	Q = Q * to_quaternion(camera_extrinsics.rotation) * Q.inverse();

	// Invert if this is stream pose
	if (target_profile.stream_type() == RS2_STREAM_POSE)
		Q = Q.inverse();

	// Initialize the transform
	stream.extrinsics.header.frame_id = base_link;
	stream.extrinsics.child_frame_id = stream.frame_id;
	stream.extrinsics.transform.translation.x = camera_extrinsics.translation[2];
	stream.extrinsics.transform.translation.y = -camera_extrinsics.translation[0];
	stream.extrinsics.transform.translation.z = -camera_extrinsics.translation[1];
	stream.extrinsics.transform.rotation.x = Q.getX();
	stream.extrinsics.transform.rotation.y = Q.getY();
	stream.extrinsics.transform.rotation.z = Q.getZ();
	stream.extrinsics.transform.rotation.w = Q.getW();
}

void kaya_vision::KayaRealSenseCamera::initialize_intrinsics(StreamInfo& stream, const rs2::video_stream_profile& target_profile)
{
	// Get the instrics
	auto intrinsic = target_profile.get_intrinsics();

	// Initialize the camera info
	stream.intrinsics.header.frame_id = stream.frame_id;

	// Shape of image
	stream.intrinsics.width = intrinsic.width;
	stream.intrinsics.height = intrinsic.height;

	// Intrinsic camera matrix
	stream.intrinsics.k.at(0) = intrinsic.fx;
	stream.intrinsics.k.at(2) = intrinsic.ppx;
	stream.intrinsics.k.at(4) = intrinsic.fy;
	stream.intrinsics.k.at(5) = intrinsic.ppy;
	stream.intrinsics.k.at(8) = 1;

	// Projection matrix
	stream.intrinsics.p.at(0) = stream.intrinsics.k.at(0);
	stream.intrinsics.p.at(1) = 0;
	stream.intrinsics.p.at(2) = stream.intrinsics.k.at(2);
	stream.intrinsics.p.at(3) = 0;
	stream.intrinsics.p.at(4) = 0;
	stream.intrinsics.p.at(5) = stream.intrinsics.k.at(4);
	stream.intrinsics.p.at(6) = stream.intrinsics.k.at(5);
	stream.intrinsics.p.at(7) = 0;
	stream.intrinsics.p.at(8) = 0;
	stream.intrinsics.p.at(9) = 0;
	stream.intrinsics.p.at(10) = 1;
	stream.intrinsics.p.at(11) = 0;

//	if (type_index == DEPTH && enable_[DEPTH] && enable_[COLOR]) {
//		camera_info_[type_index].p.at(3) = 0;     // Tx
//		camera_info_[type_index].p.at(7) = 0;     // Ty
//	}

	// Stereo rotations matrix, for realsense this is identity
	stream.intrinsics.r.at(0) = 1.0;
	stream.intrinsics.r.at(1) = 0.0;
	stream.intrinsics.r.at(2) = 0.0;
	stream.intrinsics.r.at(3) = 0.0;
	stream.intrinsics.r.at(4) = 1.0;
	stream.intrinsics.r.at(5) = 0.0;
	stream.intrinsics.r.at(6) = 0.0;
	stream.intrinsics.r.at(7) = 0.0;
	stream.intrinsics.r.at(8) = 1.0;

	// Distortion model
	stream.intrinsics.distortion_model = "plumb_bob";
	stream.intrinsics.d.resize(5);
	for (int i = 0; i < 5; i++)
		stream.intrinsics.d.at(i) = intrinsic.coeffs[i];
}

RCLCPP_COMPONENTS_REGISTER_NODE(kaya_vision::KayaRealSenseCamera)

