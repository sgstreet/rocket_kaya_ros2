#include <gst/gst.h>
#include <gst/app/gstappsrc.h>
#include <glib.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include "kaya_sound_core/kaya_sound_core.hpp"

using namespace std::chrono_literals;

gboolean static bus_event(GstBus* /*bus*/, GstMessage* msg, gpointer data)
{
	GMainLoop* loop = static_cast<GMainLoop*>(data);

	switch (GST_MESSAGE_TYPE(msg)) {
		case GST_MESSAGE_ERROR:
		case GST_MESSAGE_EOS:
			g_main_loop_quit(loop);
			break;
		default:
			break;
	}
	return true;
}

void static new_pad(GstElement* /*decodebin*/, GstPad* pad, gpointer data)
{
	// Retrieve the sink element
	GstElement* converter = gst_bin_get_by_name(GST_BIN(data), "converter");

	// Only link pad once
	GstPad* audiopad = gst_element_get_static_pad(converter, "sink");
	if (GST_PAD_IS_LINKED(audiopad)) {
		g_object_unref(audiopad);
		g_object_unref(converter);
		return;
	}

	// check media type
	GstCaps* caps = gst_pad_query_caps(pad, NULL);
	GstStructure* str = gst_caps_get_structure(caps, 0);
	if (!g_strrstr(gst_structure_get_name(str), "audio")) {
		gst_caps_unref(caps);
		gst_object_unref(audiopad);
		g_object_unref(converter);
		return;
	}

	// Link and play
	gst_pad_link(pad, audiopad);

	// Clean up
	gst_caps_unref(caps);
	g_object_unref(audiopad);
	g_object_unref(converter);
}

kaya_sound::KayaSoundCore::KayaSoundCore(const rclcpp::NodeOptions& options) :
	Node("kaya_sound_core", options)
{
	// Hook the interface
	text_to_speech_client = create_client<kaya_sound_interface::srv::TextToSpeech>("text_to_speech");
	play_text_sub = create_subscription<std_msgs::msg::String>("play_text", 10, std::bind(&KayaSoundCore::play_text_callback, this, std::placeholders::_1));
	play_text_pub = create_publisher<std_msgs::msg::String>("play_text", rclcpp::SystemDefaultsQoS());

	// Initialize the gstream library
	gst_init(nullptr, nullptr);

	// Start the the play thread
	play_thread = std::make_unique<std::thread>(&KayaSoundCore::play_audio, this);

	// Wait for the text to speech engine to start
	while (!text_to_speech_client->wait_for_service(1s)) {
		if (!rclcpp::ok()) {
			RCLCPP_ERROR(get_logger(), "client interrupted while waiting for text_to_speech service to appear.");
			throw std::runtime_error("client interrupted while waiting for text_to_speech service to appear");
		}
		RCLCPP_INFO(get_logger(), "waiting for text_to_speech service to appear...");
	}

	// Let every know
	play_text("kaya: sound subsystem running.");
}

kaya_sound::KayaSoundCore::~KayaSoundCore()
{
	// Clean up the gst thread if needed
	if (play_thread->joinable()) {
		push_audio_data(kaya_sound_interface::msg::AudioData::SharedPtr());
		play_thread->join();
	}
}

void kaya_sound::KayaSoundCore::play_audio()
{
	// Setup the loop engine
	GMainLoop* engine = g_main_loop_new(0, false);

	while (true) {

		// Wait for something to do
		kaya_sound_interface::msg::AudioData::SharedPtr audio = pop_audio_data();
		if (!audio)
			break;

		// Setup the pipeline
		GstElement* pipeline = gst_pipeline_new("pipeline");

		// Connect to the bus
		GstBus* bus = gst_pipeline_get_bus(GST_PIPELINE(pipeline));
		gst_bus_add_watch(bus, bus_event, engine);
		gst_object_unref(bus);

		// Create the elements of the pipeline
		GstElement* source = gst_element_factory_make("appsrc", "source");
		GstElement* decoder = gst_element_factory_make("decodebin", "decoder");
		GstElement* converter = gst_element_factory_make("audioconvert", "converter");
		GstElement* sink = gst_element_factory_make("alsasink", "sink");

		// Add elements to the pipeline
		gst_bin_add_many(GST_BIN(pipeline), source, decoder, converter, sink, NULL);

		// Setup the source
		g_object_set(G_OBJECT(source), "do-timestamp", true, NULL);
		g_object_set(G_OBJECT(source), "num-buffers", 1, NULL);
		g_object_set(G_OBJECT(source), "emit-signals", true, NULL);

		// Setup the decoder
		g_signal_connect(decoder, "pad-added", G_CALLBACK(new_pad), pipeline);

		// Set up the sink
		GstPad* pad = gst_element_get_static_pad(sink, "sink");
		gst_pad_activate_mode (pad, GST_PAD_MODE_PULL, true);
		g_object_unref(pad);

		// Link up the pipe line
		gst_element_link(source, decoder);
		gst_element_link(decoder, converter);
		gst_element_link(converter, sink);

		// Mark as playing
		gst_element_set_state(pipeline, GST_STATE_PLAYING);

		// The the gstreamer buffer
		GstBuffer* buffer = gst_buffer_new_and_alloc(audio->data.size());
		gst_buffer_fill(buffer, 0, &audio->data[0], audio->data.size());
		gst_app_src_push_buffer(GST_APP_SRC(source), buffer);

		// Run the main loop
		g_main_loop_run(engine);

		// Clean up the pipeline
		gst_element_set_state(pipeline, GST_STATE_NULL);
		gst_object_unref(GST_OBJECT(pipeline));
	}

	// Clean up the engine
	g_main_loop_unref(engine);
}

void kaya_sound::KayaSoundCore::play_text(const std::string& text)
{
	std_msgs::msg::String msg;
	msg.data = text;
	play_text_pub->publish(msg);
}

void kaya_sound::KayaSoundCore::push_audio_data(kaya_sound_interface::msg::AudioData::SharedPtr audio)
{
	std::unique_lock<std::mutex> lock(audio_queue_lock);
	audio_queue.push_back(audio);
	lock.unlock();
	audio_queue_cv.notify_one();
}

kaya_sound_interface::msg::AudioData::SharedPtr kaya_sound::KayaSoundCore::pop_audio_data()
{
	std::unique_lock<std::mutex> lock(audio_queue_lock);
	while (audio_queue.empty())
		audio_queue_cv.wait(lock);
	auto audio = audio_queue.front();
	audio_queue.pop_front();
	return audio;
}

void kaya_sound::KayaSoundCore::play_speech(rclcpp::Client<kaya_sound_interface::srv::TextToSpeech>::SharedFuture future)
{
	// Publish it
	auto audio = std::make_shared<kaya_sound_interface::msg::AudioData>();
	audio->data = future.get()->speech.data;
	push_audio_data(audio);
}

void kaya_sound::KayaSoundCore::play_text_callback(const std_msgs::msg::String::SharedPtr msg)
{
	// Create the request
	auto request = std::make_shared<kaya_sound_interface::srv::TextToSpeech::Request>();
	request->text = msg->data;

	// Fire the request
	text_to_speech_client->async_send_request(request, std::bind(&KayaSoundCore::play_speech, this, std::placeholders::_1));
}

RCLCPP_COMPONENTS_REGISTER_NODE(kaya_sound::KayaSoundCore)
