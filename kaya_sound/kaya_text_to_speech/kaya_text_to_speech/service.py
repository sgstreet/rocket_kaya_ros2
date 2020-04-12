import os
import rclpy

from rclpy.node import Node
from google.cloud import texttospeech
from kaya_sound_interface.srv import TextToSpeech



class TextToSpeechService(Node):

    def __init__(self):
        super().__init__('text_to_speech')
        os.environ['GOOGLE_APPLICATION_CREDENTIALS'] = '/home/stephen/Workspace/rocket-kaya/keys/rocket-kaya-sound-bd4fac987021.json'
        self.client = texttospeech.TextToSpeechClient()
        self.voice = texttospeech.types.VoiceSelectionParams(language_code='en-US', name='en-AU-Wavenet-C', ssml_gender=texttospeech.enums.SsmlVoiceGender.FEMALE)
        self.audio_config = texttospeech.types.AudioConfig(audio_encoding=texttospeech.enums.AudioEncoding.MP3)
        self.srv = self.create_service(TextToSpeech, 'text_to_speech', self.text_to_speech_callback)
        self.get_logger().info('ready')

    def text_to_speech_callback(self, request, response):
        self.get_logger().debug('translating: {}'.format(request.text))
        response.speech.data = self.client.synthesize_speech(texttospeech.types.SynthesisInput(text=request.text), self.voice, self.audio_config).audio_content
        self.get_logger().debug('done translating: {}'.format(request.text))
        return response


def main(args=None):
 
    rclpy.init(args=args)
 
    service = TextToSpeechService()
 
    try:
        rclpy.spin(service)
    except: KeyboardInterrupt
 
    rclpy.shutdown()
 
 
if __name__ == '__main__':
    main()
