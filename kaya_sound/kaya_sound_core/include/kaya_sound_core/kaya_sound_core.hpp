#ifndef _KAYA_SOUND_CORE_HPP_
#define _KAYA_SOUND_CORE_HPP_

#include <mutex>
#include <condition_variable>
#include <deque>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/node.hpp>

#include <std_msgs/msg/string.hpp>
#include <kaya_sound_interface/msg/audio_data.hpp>
#include <kaya_sound_interface/srv/text_to_speech.hpp>

namespace kaya_sound {

class KayaSoundCore : public rclcpp::Node
{
	public:
		KayaSoundCore(const rclcpp::NodeOptions& options);
		virtual ~KayaSoundCore();

	private:
		void play_text(const std::string& text);
		void play_text_callback(const std_msgs::msg::String::SharedPtr msg);
		void play_speech(rclcpp::Client<kaya_sound_interface::srv::TextToSpeech>::SharedFuture future);
		void play_audio();

		void push_audio_data(kaya_sound_interface::msg::AudioData::SharedPtr audio);
		kaya_sound_interface::msg::AudioData::SharedPtr pop_audio_data();

		std::deque<kaya_sound_interface::msg::AudioData::SharedPtr> audio_queue;
		std::mutex audio_queue_lock;
		std::condition_variable audio_queue_cv;

		std::unique_ptr<std::thread> play_thread;

		rclcpp::Subscription<std_msgs::msg::String>::SharedPtr play_text_sub;
		rclcpp::Publisher<std_msgs::msg::String>::SharedPtr play_text_pub;
		rclcpp::Client<kaya_sound_interface::srv::TextToSpeech>::SharedPtr text_to_speech_client;
};

}

#endif
