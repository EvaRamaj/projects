#!/usr/bin/env python2
import cv2
import rospy
from sensor_msgs.msg import Image
from edo_connect4.srv import *
from cv_bridge import CvBridge
import sys
from random import shuffle

latest_image = None


def whos_first(ids, options):
    for i, marker_id in enumerate(ids):
        first_player = options[marker_id[i]]
        return first_player


def callback(img):
    global latest_image
    latest_image = img


def handle_dice_roll(req):

    global latest_image

    # Doesn't work for odd numbers
    if req.num_markers < 2:
        virtual_roll = [1, 2]
    else:
        multiply_factor = req.num_markers/2
        virtual_roll = [1, 2]*multiply_factor

    shuffle(virtual_roll)

    for item in virtual_roll:
        print(item)

    r = rospy.Rate(0.5)
    while not rospy.is_shutdown():
        r.sleep()

        # Wait until we get an image
        while latest_image is None:
            if rospy.is_shutdown():
                sys.exit(0)
            r.sleep()

        # Store the 1st image incoming and the following in img
        img = latest_image
        latest_image = None  # not thread safe, but doesn't really matter here

        # Process the frame
        cv_image = bridge.imgmsg_to_cv2(img, desired_encoding="passthrough")
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        try:
            # Extract all of the ARUCO markers in it
            corners, ids, rejected_img_points = cv2.aruco.detectMarkers(gray, ARUCO_DICT)
            if ids is None:
                continue

            winner = whos_first(ids, virtual_roll)

            print("The first player is:", winner)
            return DiceResponse(winner)

        except:
            print("Detection failed. Will try again.")



if __name__ == '__main__':
    rospy.init_node("aruco_dice")
    s = rospy.Service('roll_dice', Dice, handle_dice_roll)
    print "Ready to find a first player."

    rospy.Subscriber("asus_xtion/rgb/image_raw", Image, callback)
    bridge = CvBridge()
    ARUCO_DICT = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)

    rospy.spin()
