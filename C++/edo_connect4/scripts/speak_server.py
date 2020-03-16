#!/usr/bin/env python2
from google.cloud import texttospeech
from os import path, environ, system, listdir
from hashlib import md5
from edo_connect4.srv import *
import rospy
import urllib2
import pyaudio
import wave

hashes_list = []
dir_name = path.dirname(__file__)
wav_dir = "hashed_wavs/"
mentioned_lack_of_internet = False


def gcloud_call(say_text):
    # Google Text to Speech instantiations
    environ["GOOGLE_APPLICATION_CREDENTIALS"] = dir_name + "/gcloud_keys.json"
    client = texttospeech.TextToSpeechClient()
    voice = texttospeech.types.VoiceSelectionParams(
        language_code='en-US',
        ssml_gender=texttospeech.enums.SsmlVoiceGender.MALE)
    audio_config = texttospeech.types.AudioConfig(
        audio_encoding=texttospeech.enums.AudioEncoding.LINEAR16)

    # Set the text input to be synthesized
    synthesis_input = texttospeech.types.SynthesisInput(text=say_text)

    response = client.synthesize_speech(synthesis_input, voice, audio_config)

    return response.audio_content


def internet_works():
    try:
        urllib2.urlopen('http://216.58.192.142', timeout=1)
        return True
    except urllib2.URLError as err:
        print(err)
        return False


def play_wav(file_name):
    # PyAudio instatiations
    chunk = 1024
    p = pyaudio.PyAudio()

    # Read out the reply
    wf = wave.open(dir_name+"/"+file_name, 'rb')

    stream = p.open(
        format=p.get_format_from_width(wf.getsampwidth()),
        channels=wf.getnchannels(),
        rate=wf.getframerate(),
        output=True)
    data = wf.readframes(chunk)

    while data != '':
        stream.write(data)
        data = wf.readframes(chunk)

    stream.close()
    p.terminate()


def load_stored_filenames():
    for filename in listdir(dir_name+"/"+wav_dir[:-1]):
        if filename.endswith(".wav"):
            hashes_list.append(filename[:-4])


def hash_string(text):
    hash_object = md5(text)  # len32 hex result
    return hash_object.hexdigest()


def store_file(audio, file_name):
    # The response's audio_content is binary.
    with open(dir_name+"/"+wav_dir+file_name, 'wb') as out:
        out.write(audio)
        print('Audio content written to hashed_wavs/'+file_name)


def handle_speech(req):
    # The main function here. Goes through all of the steps necessary to speak.
    transcript = req.say
    hex_digest = hash_string(transcript)

    # The fist word of the transcript is now added to the file name, so as to delete clutter without listening to files
    file_to_store = transcript[: transcript.find(' ')] + '-' + hex_digest

    if any(file_to_store in s for s in hashes_list):
        print("Found string among stored.")
        play_wav(wav_dir + file_to_store + ".wav")

    else:
        if internet_works():
            audio = gcloud_call(transcript)
            file_name = file_to_store+'.wav'
            store_file(audio, file_name)
            play_wav(wav_dir+file_name)
            hashes_list.append(file_to_store)

        # Offline mode
        else:
            global mentioned_lack_of_internet
            if not mentioned_lack_of_internet:
                play_wav(wav_dir+"Seems-bc9983333d2dbda1b82fb79931e8c99b.wav")  # Seems like we are offline file play
                mentioned_lack_of_internet = True
            print("Switching to OS in-built Text-to-Speech.")
            system('spd-say -p 0 -t child_male "{}"'.format(req.say))  # Using Speech Dispatcher

    return EdoSpeakResponse(True)


def speech_server():
    rospy.init_node('edo_speak_server')
    rospy.Service('edo_speak', EdoSpeak, handle_speech)
    print("Ready to speak.")
    rospy.spin()


if __name__ == "__main__":
    load_stored_filenames()
    mentioned_lack_of_internet = False
    speech_server()


# Google Cloud Supported Voices: https://cloud.google.com/text-to-speech/docs/voices

# DESCRIPTION OF THE OFFLINE TEXT-TO-SPEECH PARAMS
# Link: http://manpages.ubuntu.com/manpages/bionic/man1/spd-say.1.html
# In case it doesnt work install: sudo apt install speech-dispatcher
#
# -p pitch
# -r rate
# -i volume
# -t voice type
# -s spelling: Spell the message
# -m Punctuation mode: none, some, all
#
# system('spd-say -p 0 -t child_male "your program has finished"')
