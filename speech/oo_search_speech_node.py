from __future__ import division

import re
import sys

from google.cloud import speech
from google.cloud.speech import enums
from google.cloud.speech import types
from google.oauth2 import service_account
import pyaudio
from six.moves import queue
import time, os
import rospy
from std_msgs.msg import String
import json
# Audio recording parameters
RATE = 16000
CHUNK = int(RATE / 10)  # 100ms

record_status = "stop"
stream_audio = None

def record_callback(data):
    global record_status
    global stream_audio

    if str(data.data) == 'stop':
        if stream_audio is not None:
            stream_audio._audio_stream.stop_stream()
            stream_audio._audio_stream.close()
            stream_audio.closed = True        
            stream_audio._buff.put(None)
            stream_audio._audio_interface.terminate()
            stream_audio = None

    record_status = str(data.data)

pub = rospy.Publisher('/pidrone/speech_final', String, queue_size=10)
pub_temp = rospy.Publisher('/pidrone/speech_temp', String, queue_size=10)
rospy.init_node('pidrone_speech_node')
rate = rospy.Rate(10)
rospy.Subscriber("/hololens/record", String, record_callback)


class MicrophoneStream(object):
    """Opens a recording stream as a generator yielding the audio chunks."""
    def __init__(self, rate, chunk):
        self._rate = rate
        self._chunk = chunk

        # Create a thread-safe buffer of audio data
        self._buff = queue.Queue()
        self.closed = True

    def __enter__(self):
        self._audio_interface = pyaudio.PyAudio()
        self._audio_stream = self._audio_interface.open(
            format=pyaudio.paInt16,
            # The API currently only supports 1-channel (mono) audio
            # https://goo.gl/z757pE
            channels=1, rate=self._rate,
            input=True, frames_per_buffer=self._chunk,
            # Run the audio stream asynchronously to fill the buffer object.
            # This is necessary so that the input device's buffer doesn't
            # overflow while the calling thread makes network requests, etc.
            stream_callback=self._fill_buffer,
        )

        self.closed = False

        return self

    def __exit__(self, type, value, traceback):
        self._audio_stream.stop_stream()
        self._audio_stream.close()
        self.closed = True
        # Signal the generator to terminate so that the client's
        # streaming_recognize method will not block the process termination.
        self._buff.put(None)
        self._audio_interface.terminate()

    def _fill_buffer(self, in_data, frame_count, time_info, status_flags):
        """Continuously collect data from the audio stream, into the buffer."""
        self._buff.put(in_data)
        return None, pyaudio.paContinue

    def generator(self):
        while not self.closed:
            # Use a blocking get() to ensure there's at least one chunk of
            # data, and stop iteration if the chunk is None, indicating the
            # end of the audio stream.
            chunk = self._buff.get()
            if chunk is None:
                return
            data = [chunk]

            # Now consume whatever other data's still buffered.
            while True:
                try:
                    chunk = self._buff.get(block=False)
                    if chunk is None:
                        return
                    data.append(chunk)
                except queue.Empty:
                    break

            yield b''.join(data)


def listen_print_loop(responses):
    """Iterates through server responses and prints them.

    The responses passed is a generator that will block until a response
    is provided by the server.

    Each response may contain multiple results, and each result may contain
    multiple alternatives; for details, see https://goo.gl/tjCPAU.  Here we
    print only the transcription for the top alternative of the top result.

    In this case, responses are provided for interim results as well. If the
    response is an interim one, print a line feed at the end of it, to allow
    the next result to overwrite it, until the response is a final one. For the
    final one, print a newline to preserve the finalized transcription.
    """
    global record_status

    num_chars_printed = 0
    for response in responses:
        if not response.results:
            continue

        # The `results` list is consecutive. For streaming, we only care about
        # the first result being considered, since once it's `is_final`, it
        # moves on to considering the next utterance.
        result = response.results[0]
        if not result.alternatives:
            continue

        # Display the transcription of the top alternative.
        transcript = result.alternatives[0].transcript

        # Display interim results, but with a carriage return at the end of the
        # line, so subsequent lines will overwrite them.
        #
        # If the previous result was longer than this one, we need to print
        # some extra spaces to overwrite the previous result
        overwrite_chars = ' ' * (num_chars_printed - len(transcript))
        if not result.is_final:
            sys.stdout.write(transcript + overwrite_chars + '\r')
            sys.stdout.flush()
            pub_temp.publish(transcript.lower().replace(".",""))

            num_chars_printed = len(transcript)

        else:
            # replace one, two, three to 1, 2, 3
            transcript = transcript.replace("one", "1")
            transcript = transcript.replace("two", "2")
            transcript = transcript.replace("three", "3")
            transcript = transcript.replace("twice", "2 times")
            # Redbox to red box
            transcript = transcript.replace("Redbox", "red box")
            transcript = transcript.replace("restroom", "red room")
            transcript = transcript.replace("go away", "go west")
            transcript = transcript.replace("Reebok", "red box")
            transcript = transcript.replace("Roblox", "one block")
            transcript = transcript.replace("grade", "grid")
            
            print(transcript + overwrite_chars)
            pub_temp.publish(transcript.lower().replace(".",""))
            pub.publish(transcript.lower().replace(".",""))
            # Exit recognition if any of the transcribed phrases could be
            # one of our keywords.
            if re.search(r'\b(exit|quit)\b', transcript, re.I):
                print('Exiting..')
                break

            num_chars_printed = 0


def main():
    global record_status
    global stream_audio
    # See http://g.co/cloud/speech/docs/languages
    # for a list of supported languages.
    language_code = 'en-US'  # a BCP-47 language tag
    #TODO Replace with your credentials
    credentials = service_account.Credentials.from_service_account_file('drone-de9fb6cb6c72.json')

    client = speech.SpeechClient(credentials = credentials)
    config = types.RecognitionConfig(
        encoding=enums.RecognitionConfig.AudioEncoding.LINEAR16,
        sample_rate_hertz=RATE,
        language_code=language_code,
        speech_contexts=[speech.types.SpeechContext(phrases=['fly', 'hover', 'red', 'green', 'blue', 'box', 'room', 'photo', 'photograph', 'photo', 'picture', 'cube', 'block', 'square',
        'down', 'up', 'foward', 'back', 'backward','left', 'right', 'move', 'space', 'grid', 'cell', 'times', '1', '2', '3', 'one', 'two', 'three', 'twice', 'step',
        'grid units', 'grid spaces', 'go west', 'three blocks',
        'two times', 'three times', 'two spaces', 'three spaces', 'two grids', 'three grids', 'two cells', 'three celss', 'three blocks', 'one block',
        'red box', 'blue box', 'green box', 'red room', 'blue room', 'green room'])])
    streaming_config = types.StreamingRecognitionConfig(
        config=config,
        interim_results=True)
    print "speech node start"
    while True:
        
        if record_status == 'start':
            print "stream started"
            with MicrophoneStream(RATE, CHUNK) as stream:
                stream_audio = stream
                audio_generator = stream.generator()
                requests = (types.StreamingRecognizeRequest(audio_content=content)
                            for content in audio_generator)

                responses = client.streaming_recognize(streaming_config, requests)

                # Now, put the transcription responses to use.
                listen_print_loop(responses)
            record_status = 'stop'
            stream_audio = None
            print "stream ended"


if __name__ == '__main__':
    main()