# Copyright (c) Microsoft. All rights reserved.
# Licensed under the MIT license. See LICENSE.md file in the project root for full license information.

# <code>
import azure.cognitiveservices.speech as speechsdk
from gtts import gTTS
from pygame import mixer
from PIL import Image, ImageDraw, ImageFont
import smtplib, ssl, email
import time
import os
from email import encoders
from email.mime.base import MIMEBase
from email.mime.multipart import MIMEMultipart
from email.mime.text import MIMEText


def share(receiver_address, file):
    subject = "Merry Christmas"
    body = "I wish to you and to your family all the best"
    port = 465  # For SSL
    password = "HackaTum2019"
    sender_email = "hacktum@gmail.com"
    receiver_email = receiver_address

    message = MIMEMultipart()
    message["From"] = sender_email
    message["To"] = receiver_email
    message["Subject"] = subject
    message["Bcc"] = receiver_email  # Recommended for mass emails

    # Add body to email
    message.attach(MIMEText(body, "plain"))
    filename = file
    with open(filename, "rb") as attachment:
        part = MIMEBase("application", "octet-stream")
        part.set_payload(attachment.read())
    encoders.encode_base64(part)
    part.add_header(
        "Content-Disposition",
        f"attachment; filename= {filename}",
    )
    message.attach(part)
    text = message.as_string()

    # Create a secure SSL context
    context = ssl.create_default_context()

    with smtplib.SMTP_SSL("smtp.gmail.com", port, context=context) as server:
        server.login(sender_email, password)
        server.sendmail(sender_email, receiver_email, text)


def sound_player(message, name):
    mixer.init()
    language = 'en'
    player = gTTS(text=message, lang=language, slow=False)
    player.save('{}.mp3'.format(name))
    mixer.music.load('{}.mp3'.format(name))
    mixer.music.play()
    while mixer.music.get_busy():
        continue


def speech_to_text():
    # Creates an instance of a speech config with specified subscription key and service region.
    # Replace with your own subscription key and service region (e.g., "westus").
    speech_key, service_region = "4365d7f0244e409cb31d4eeb0e248a90", "westeurope"
    speech_config = speechsdk.SpeechConfig(subscription=speech_key, region=service_region)
    speech_object = speechsdk.SpeechRecognizer(speech_config=speech_config)
    print("Give Object...")
    result = speech_object.recognize_once()
    print(result.text)
    return result.text


def draw_text(image, width_idx, height_idx, text, color, letter_font):
    ImageDraw.Draw(image).text((width_idx*cell_width, height_idx*cell_height), text, color, letter_font)


image_dict = {
    "sun": './sun.png',
    "son": './sun.png',
    "tree": './tree.png',
    "car": './car.png',
    "cloud": './cloud.png',
    "house": './house.png',
    "3": './tree.png',
    "santa claus": './santa_claus.png',
    "snowman": './snowman.jpeg',
    "christmas tree": './christmas_tree.png',
    "christmas 3": './christmas_tree.png',
    "christmas market": './christmas_market.png',
    "jingle bells": 'jingle_bells.jpeg'
}
'''
grid_dict = {
    "top left": (0, 0),
    "top middle": (1, 0),
    "top right": (2, 0),
    "middle left": (0, 1),
    "middle": (1, 1),
    "middle right": (2, 1),
    "bottom left": (0, 2),
    "bottom middle": (1, 2),
    "bottom right": (2, 2)
}
'''

grid_dict = {
    "top left": (0, 0),
    "top right": (1, 0),
    "bottom left": (0, 1),
    "bottom right": (1, 1)
}

addresses_dict = {
    "anna": "eua.ramaj@gmail.com"
}

# cell_width = 1278//3
# cell_height = 1020//3
font = ImageFont.truetype("Arial.ttf", 32)
cell_width = 1278//2
cell_height = 1020//2

keep_up = "Yes."
drawing = Image.new('RGB', (1278, 1020), (255, 255, 255))
file = "./drawing.png"
# WELCOME THE USER
sound_player('Hi! It seems like you want to draw something today. Lets start!', "welcome")

while keep_up.find("Yes") != -1:
    # ASK FOR THE OBJECT
    sound_player('Give me an object that you want to draw?', "draw")
    draw_result = speech_to_text()
    try:
        # ASK FOR THE POSITION
        # image_path = image_dict[draw_result.text.replace(".", "").replace(",", "").lower()]
        found = False
        for key in image_dict.keys():
            if key in draw_result.lower():
                image_path = image_dict[key]
                found = True
                break
        if found is False:
            raise KeyError
        sound_player('In which position would you like it?', "position")
        position_result = speech_to_text()
        width_pos, height_pos = grid_dict[position_result.lower().replace(".", '')]
        im = Image.open(image_path)
        im = im.resize((cell_width, cell_height))
        drawing.paste(im, (width_pos * cell_width, height_pos * cell_height))
    except KeyError:
        sound_player('Sorry I did not get you, try again?', "sorry")
        continue

    # ASK FOR TERMINATION
    sound_player('Would you like to continue?', "keep_going")
    keep_up = speech_to_text()

    if keep_up.find("No") != -1:
        sound_player('Do you want to share your creation?', "share")
        share_result = speech_to_text()
        if share_result.find("Yes") != -1:
            sound_player('Lets write a wish on the card?', "wish")
            wish = speech_to_text()
            draw_text(drawing, 1.2, 1.5, wish, (255, 0, 0), font)
            drawing.save("./drawing.png")
            not_sent = True
            while not_sent:
                sound_player('To whom would you like to send it?', "receiver")
                receiver_result = speech_to_text()
                try:
                    for key in addresses_dict.keys():
                        if key in receiver_result.lower():
                            email_receiver = addresses_dict[key]
                            share(email_receiver, file)
                            sound_player('Mission completed. See you next time!', "sent_mail")
                            not_sent = False
                except KeyError:
                    continue

        else:
            sound_player('Perfect! I will save your image. See you!', "keep_going")
            drawing.save("./drawing.png")

drawing.show()

# Starts speech recognition, and returns after a single utterance is recognized. The end of a
# single utterance is determined by listening for silence at the end or until a maximum of 15
# seconds of audio is processed.  The task returns the recognition text as result. 
# Note: Since recognize_once() returns only a single utterance, it is suitable only for single
# shot recognition like command or query. 
# For long-running multi-utterance recognition, use start_continuous_recognition() instead.


'''
# Checks result.
if result.reason == speechsdk.ResultReason.RecognizedSpeech:
    print("Recognized: {}".format(result.text))
elif result.reason == speechsdk.ResultReason.NoMatch:
    print("No speech could be recognized: {}".format(result.no_match_details))
elif result.reason == speechsdk.ResultReason.Canceled:
    cancellation_details = result.cancellation_details
    print("Speech Recognition canceled: {}".format(cancellation_details.reason))
    if cancellation_details.reason == speechsdk.CancellationReason.Error:
        print("Error details: {}".format(cancellation_details.error_details))
'''
# </code>
