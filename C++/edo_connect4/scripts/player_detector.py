import cv2
from matplotlib import pyplot as plt
import rospy
import numpy as np
from scipy.cluster.vq import kmeans2 as KMeans
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import sys
import time



latest_image = None

def callback(img):
    global latest_image
    latest_image = img

def player_detector():
    #1 for the red player 2 for the blue player
    player = 0
    global latest_image

    img_sub = rospy.Subscriber("asus_xtion/rgb/image_raw", Image, callback)
    bridge = CvBridge()

    r = rospy.Rate(0.5)
    while not rospy.is_shutdown():
        while latest_image is None:
            if rospy.is_shutdown():
                sys.exit(0)
        img = latest_image
        latest_image = None  # not thread safe, but doesn't really matter here

        cv_image = bridge.imgmsg_to_cv2(img, desired_encoding="passthrough")
        #take one token and detect if it is red or blue
        count_blue = 0
        count_red = 0
        for r in range (276,307):
            for c in range (587,618):
                if cv_image[r][c][0] > cv_image[r][c][2]:
                    count_red += 1
                else:
                    count_blue += 1
        if count_red > count_blue:
            print "I am the red player"
            player = 1
            return player
        else:
            print "I am the blue player"
            player = 2
            return player
        count_red = 0
        count_blue = 0

        plt.subplot(224)
        plt.imshow(cv_image)
        plt.show()

#result = player_detector()
#print result
