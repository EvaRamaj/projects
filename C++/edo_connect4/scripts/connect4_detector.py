import rospy
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2
from cv_bridge import CvBridge
import cv2
import sys
import numpy as np
import matplotlib.pyplot as plt
from skimage.measure import compare_ssim
from connect4_board_detector import BoardDetector

COLUMNS=7
ROWS=6

latest_image = None
latest_depth = None
image_counter = 20
use_loaded_images = False

def callback(img):

    global latest_image
    latest_image = img

def depth_callback(depth):
    global latest_depth
    latest_depth = depth


img_sub = rospy.Subscriber("asus_xtion/rgb/image_raw", Image, callback)
depth_sub = rospy.Subscriber("asus_xtion/depth/image_raw", Image, depth_callback)
bridge = CvBridge()


def get_img():
    global latest_image
    global image_counter
    global use_loaded_images
    if use_loaded_images:
        cv_image = cv2.imread("image"+str(image_counter)+".png")
        image_counter += 1
        #if image_counter >=6: image_counter = 0
    else:
        r = rospy.Rate(4.0)
        while latest_image is None:
            if rospy.is_shutdown():
                sys.exit(0)
            r.sleep()
        img = latest_image
        latest_image = None  # not thread safe, but doesn't really matter here
        cv_image = bridge.imgmsg_to_cv2(img, desired_encoding="passthrough")
        #cv2.imwrite("image"+str(image_counter)+".png", cv_image)
        #image_counter += 1
        #print "written"
    return cv_image


def get_depth():
    global latest_depth
    r = rospy.Rate(4.0)
    while latest_image is None:
        if rospy.is_shutdown():
            sys.exit(0)
        r.sleep()
    depth = latest_depth
    latest_depth = None  # not thread safe, but doesn't really matter here
    depth_image= bridge.imgmsg_to_cv2(depth, desired_encoding="passthrough")

    return depth_image


def visualize_probs(probs, path=None):
    img = np.zeros((600,700,3), dtype=np.float) +1.
    for c in range(COLUMNS):
        for r in range(ROWS):
            middle = (600 - (50+r*100), 50+c*100)
            rectRed = (middle[0]-15, middle[1]-15, middle[0]+15, middle[1])
            rectBlue = (middle[0]-15, middle[1], middle[0]+15, middle[1]+15)

            #to make it more visible
            pr = probs[c,r,1]*0.8 + 0.2 if probs[c,r,1] > 1e-8 else probs[c,r,1]
            pb = probs[c,r,2]*0.8 + 0.2 if probs[c,r,2] > 1e-8 else probs[c,r,2]

            colorRed = np.array([1.,0.,0.])*pr + np.array([1.,1.,1.])*(1.-pr)
            colorBlue = np.array([0.,0.,1.])*pb + np.array([1.,1.,1.])*(1.-pb)

            img[rectRed[0]:rectRed[2],rectRed[1]:rectRed[3],:] = colorRed
            img[rectBlue[0]:rectBlue[2],rectBlue[1]:rectBlue[3],:] = colorBlue
            #border:
            img[rectRed[0],rectRed[1]:rectBlue[3],:] = 0.
            img[rectRed[2],rectRed[1]:rectBlue[3],:] = 0.
            img[rectRed[0]:rectRed[2],rectRed[1],:] = 0.
            img[rectRed[0]:rectRed[2],rectBlue[3],:] = 0.
    img *= 255.
    img = img.astype(np.uint8)
    img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    if path:
        cv2.imwrite(path, img)
    else:
        cv2.imshow("img", img)
        cv2.waitKey(0)





#Detector class - use this class as interface to detect the connect 4 game
class Connect4Detector(object):
    def __init__(self):
        pass
    def reset(self):
        raise NotImplemented("Needs to be overwritten by child class!")
    def detect_state(self, sim, lastMove):
        raise NotImplemented("Needs to be overwritten by child class!")
    def calculate_probs(self, img):
        raise NotImplemented("Needs to be overwritten by child class!")


class HSVDetector(Connect4Detector):
    def __init__(self):
        self.probs = np.zeros(shape=(COLUMNS, ROWS, 3), dtype=np.float64)
        self.output_matrix = np.zeros(shape=(COLUMNS, ROWS))

        img = get_img()
        detector_image = np.copy(img)
        self.board_detector = BoardDetector()
        for i in xrange(3):
            success = self.board_detector.calculate_board_parameters(detector_image)
            if success:
                break
            img = get_img()
            detector_image = np.copy(img)

    def reset(self):
    	pass

    def detect_state(self, sim):
        print "detect state"
        img = get_img()
        img = img.astype(np.uint8)
        #self.depth = get_depth()
        probs = self.calculate_probs(img)
        #return np.argmax(probs, axis=2)
        self.probs_to_output()
        print(self.output_matrix)
        return self.output_matrix.astype(np.int)

    def calculate_probs(self, img):
        #single detection using the difference between the two images
        img = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
        for r in range(0, ROWS):
            for c in range(0, COLUMNS):
                self._get_evidence(img, c, r)

        return self.probs

    def _get_evidence(self, img, col, row):
        #count colors in a rectangle around the piece

        x, y = self.board_detector.get_circle_center_in_image(col, ROWS-row-1)
        x, y = int(x), int(y)
        d = int(self.board_detector.radius)
        img_cut = img[y-d:y+d+1, x-d:x+d+1, :].astype(np.float64) / 255.0

        cutoff_degrees = 50.

        #cut off based on saturation and brightness
        img_cut[img_cut[:,:,2]<0.25] = 1./3.
        img_cut[img_cut[:,:,2]>0.9] = 1./3.
        img_cut[img_cut[:,:,1]<0.1] = 1./3.

        img_cut_blue = 1.0 - np.abs(220./360. - img_cut[:,:,0]) / (cutoff_degrees/360.)
        img_cut_blue[img_cut_blue[:,:] <0.0] = 0.0
        img_blue = np.average(img_cut_blue)

        img_cut_red = img_cut[:,:,0]
        img_cut_red[img_cut_red[:,:] < 0.5] = 1.0 - img_cut_red[img_cut_red[:,:] < 0.5]
        img_cut_red = 1.0 - np.abs(1.0 - img_cut_red[:,:]) / (cutoff_degrees/360.)
        img_cut_red[img_cut_red[:,:] <0.0] = 0.0
        img_red = np.average(img_cut_red)

        self.probs[col, row, 0] = 1.0 - img_red - img_blue
        self.probs[col, row, 1] = img_red
        self.probs[col, row, 2] = img_blue
        #print(img_red, img_blue, col, row)


    def visualize_output(self, img):
        visual_output = np.copy(img)

        for r in xrange(ROWS):
            for c in xrange(COLUMNS):
                x, y = self.board_detector.get_circle_center_in_image(c, r)
                x, y = int(x), int(y)
                radius = int(self.board_detector.radius)
                color_num = np.argmax(self.probs[c,r,:])
                colors = [(0,0,0), (127, 0, 0), (0, 0, 127)]
                cv2.circle(img=visual_output, center=(x,y), radius = radius, color=colors[color_num], thickness=-1)

        return visual_output

    def probs_to_output(self):
        self.output_matrix = np.argmax(self.probs, axis=2)


class ColorDetector(Connect4Detector):
    def __init__(self):
        self.probs = np.zeros(shape=(COLUMNS, ROWS, 3), dtype=np.float64)
        self.output_matrix = np.zeros(shape=(COLUMNS, ROWS))

        img = get_img()
        img = get_img()
        img = get_img()
        detector_image = np.copy(img)
        self.board_detector = BoardDetector()
        for i in xrange(3):
            success = self.board_detector.calculate_board_parameters(detector_image)
            if success:
                break
            img = get_img()
            detector_image = np.copy(img)

    def reset(self):
        pass

    def detect_state(self, sim):
        print("detect state")
        img = get_img()
        img = img.astype(np.uint8)
        probs = self.calculate_probs(img)
        self.output_matrix = np.argmax(self.probs, axis=2)
        print(self.output_matrix)
        return self.output_matrix.astype(np.int)

    def calculate_probs(self, img):
        #single detection using the difference between the two images
        for r in range(0, ROWS):
            for c in range(0, COLUMNS):
                self._get_evidence(img, c, r)

                # make a prob out of it
                self.probs[c, r, :] -= np.min(self.probs[c, r, :])
                normalizer = np.sum(self.probs[c, r, :])
                if normalizer < 1e-9:
                    self.probs[c, r, :] = 1. / 3.
                else:
                    self.probs[c, r, :] /= normalizer

        #plt.subplot(221)
        #plt.imshow(diff_image, "gray")
        #plt.subplot(222)
        #img = img.astype(np.uint8)
        #plt.imshow(img)
        #vis_img = self.visualize_output(img)
        #plt.subplot(223)
        #plt.imshow(vis_img)
        #plt.show()
        return self.probs

    def _get_evidence(self, img, col, row):
        #count colors in a rectangle around the piece
        x, y = self.board_detector.get_circle_center_in_image(col, ROWS-row-1)
        x, y = int(x), int(y)
        d = int(self.board_detector.radius)


        if col == 6 and row == 3:
            img_cut = img[y-d:y+1, x-d:x+d+1, :].astype(np.float64)
        elif col == 0 and row == 2:
            #img_cut = img[y-d*2/3:y-d*1/3+1, x-d:x+d+1, :].astype(np.float64)
            img_cut = img[y:y+d+1, x-d:x+d+1, :].astype(np.float64)
        else:
            img_cut = img[y-d:y+d+1, x-d:x+d+1, :].astype(np.float64)

        img_red = np.sum(img_cut[:,:,0])
        img_green = np.sum(img_cut[:,:,1])
        img_blue = np.sum(img_cut[:,:,2])

        #self.probs[col, row, 0] = 150-img_red + 150-img_blue
        self.probs[col, row, 0] = (img_red+img_green+img_blue)/3. #grey
        self.probs[col, row, 1] = 2*img_red - img_green - img_blue #red
        self.probs[col, row, 2] = 2*img_blue - img_red - img_green #blue


    def visualize_output(self, img):
        visual_output = np.copy(img)

        for r in xrange(ROWS):
            for c in xrange(COLUMNS):
                x, y = self.board_detector.get_circle_center_in_image(c, r)
                x, y = int(x), int(y)
                radius = int(self.board_detector.radius)
                color_num = np.argmax(self.probs[c,r,:])
                colors = [(0,0,0), (127, 0, 0), (0, 0, 127)]
                cv2.circle(img=visual_output, center=(x,y), radius = radius, color=colors[color_num], thickness=-1)

        return visual_output



class SimulationDetector(Connect4Detector):
    def __init__(self):
        self.probs = np.zeros(shape=(COLUMNS, ROWS, 3), dtype=np.float64)
        self.output_matrix = np.zeros(shape=(COLUMNS, ROWS))

    def reset(self):
        pass

    def detect_state(self, sim):
        print("detect state")
        img = get_img()
        img = img.astype(np.uint8)
        probs = self.calculate_probs(img)
        self.output_matrix = np.argmax(self.probs, axis=2)
        print(self.output_matrix)
        return self.output_matrix.astype(np.int)

    def calculate_probs(self, img):
        #single detection using the difference between the two images
        for r in range(0, ROWS):
            for c in range(0, COLUMNS):
                self._get_evidence(img, c, r)

                # make a prob out of it
                self.probs[c, r, :] -= np.min(self.probs[c, r, :])
                normalizer = np.sum(self.probs[c, r, :])
                if normalizer < 1e-9:
                    self.probs[c, r, :] = 1. / 3.
                else:
                    self.probs[c, r, :] /= normalizer
        return self.probs

    def _get_evidence(self, img, col, row):
        #count colors in a rectangle around the piece
        top_left = np.array([158,143 -5])
        bottom_right = np.array([554,409 -5])
        step = (bottom_right-top_left)/np.array([COLUMNS-1,ROWS-1])
        middle = np.array([top_left[0] + step[0]*col, top_left[1] + step[1]*(ROWS-1-row)])
        y,x = middle[1],middle[0]
        d = 15
        img_cut = img[y-d:y+d+1, x-d:x+d+1, :].astype(np.float64)

        img_red = np.sum(img_cut[:,:,0])
        img_green = np.sum(img_cut[:,:,1])
        img_blue = np.sum(img_cut[:,:,2])

        self.probs[col, row, 0] = (img_red+img_green+img_blue)/3.0 #grey
        self.probs[col, row, 1] = img_red - img_green - img_blue #red
        self.probs[col, row, 2] = img_blue - img_red - img_green #blue





if __name__=='__main__':
    print("In test mode.")
    rospy.init_node("detector_test")

    detector = DifferenceDetector()
    #detector = ColorDetector()

    if not use_loaded_images:
        r = rospy.Rate(1)
        while not rospy.is_shutdown():
            print("not use_loaded")
            r.sleep()
            state = detector.detect_state(True)

    else:
        #while True:
        for i in xrange(2):
            print("use loaded")
            state = detector.detect_state(True)
