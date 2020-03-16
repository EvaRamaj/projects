import cv2
from matplotlib import pyplot as plt
import rospy
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import argparse
import sys
import column_detector
import canny_detector
import neural_detector


if __name__ == '__main__':
    rospy.init_node("detector_node")


    parser = argparse.ArgumentParser(description="Vision node")
    parser.add_argument('--no-sim', dest='sim', default=True, action='store_false', help="set to true, if this is in gazebo simulation")
    parser.add_argument('--tokens', type=int, default=8, required=False, help="set to true, if this is in gazebo simulation")
    args = parser.parse_args()

    sim = args.sim
    tokens = args.tokens

    if sim:
        print('Starting in simulation')
    else:
        print('Starting with real robot')

    latest_image = None
    def callback(img):
        global latest_image
        latest_image = img

    img_sub = rospy.Subscriber("/edo/asus_xtion/rgb/image_raw", Image, callback)
    bridge = CvBridge()

    r = rospy.Rate(0.5)
    while not rospy.is_shutdown():
        r.sleep()
        while latest_image is None:
            if rospy.is_shutdown():
                sys.exit(0)
            r.sleep()
        img = latest_image
        latest_image = None #not thread safe, but doesn't really matter here

        cv_image = bridge.imgmsg_to_cv2(img, desired_encoding="passthrough")

        print('Detecting positions...')
        #detector = column_detector.HanoiDetector(sim, tokens)
        #detector = canny_detector.HanoiDetector(sim, tokens)
        detector = neural_detector.HanoiDetector(sim, tokens)
        positions = detector.detect_positions(cv_image)
        print(positions)

        print('Visualizing...')
        imgs = detector.visualize_detection(cv_image)
        print('Close the window to detect the next image!')

        plt.subplot(441), plt.imshow(cv_image)
        plt.title('Original Image'), plt.xticks([]), plt.yticks([])
        plt.subplot(442), plt.imshow(detector.visualize_cutoffs(cv_image))
        plt.title('Lines Image'), plt.xticks([]), plt.yticks([])
        plt.subplot(443), plt.imshow(detector.cut_columns(cv_image))
        plt.title('Columns Image'), plt.xticks([]), plt.yticks([])
        for i in range(len(imgs)):
            plt.subplot(4, 4, 4+i), plt.imshow(imgs[i])
            plt.title('Token #%s' % i), plt.xticks([]), plt.yticks([])
        plt.show()


