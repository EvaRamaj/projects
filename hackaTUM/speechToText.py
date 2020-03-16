import azure.cognitiveservices.speech as speechsdk
import time
import string, re

# load the item category database
# bad_chars = [';', ':', '!', "*", '?']    # for parsing string
categories = ['cars', 'trees', 'human']

speech_key, service_region = "4365d7f0244e409cb31d4eeb0e248a90", "westeurope"
speech_config = speechsdk.SpeechConfig(subscription=speech_key, region=service_region)
speech_config.speech_recognition_language="en-US"

# Creates a recognizer with the given settings
speech_recognizer = speechsdk.SpeechRecognizer(speech_config=speech_config)

print("Say something...")
done = False

# Connect callbacks to the events fired by the speech recognizer
all_results = []
def handle_final_result(evt):
    current_result = evt.result.text.lower()
    all_results.append(evt.result.text)
    # print(current_result)

    # TODO: parse the current message for classifying graph categories, stop
    current_result = re.sub(r'[^\w\s]', '', current_result)
    print(current_result)
    str_list = current_result.split()
    if 'stop' in current_result:
        speech_recognizer.stop_continuous_recognition_async()
    else:
        for word in str_list:
            if word in categories:
                print(word)     # this is the output that we need
    return current_result
    

# catch the speech
speech_recognizer.recognized.connect(lambda evt: handle_final_result(evt))

# Starts speech recognition and hold on for long time
result = speech_recognizer.start_continuous_recognition()

while not done:
    time.sleep(.5)