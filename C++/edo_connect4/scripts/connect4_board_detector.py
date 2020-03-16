import rospy
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2
from cv_bridge import CvBridge
import cv2
import sys
import numpy as np
import matplotlib.pyplot as plt
import threading
import math
from scipy.optimize import minimize
from scipy.optimize import basinhopping as global_minimum
COLUMNS=7
ROWS=6

latest_image = None
latest_depth = None
image_counter = 0
use_loaded_images = True
use_hardcoded = False

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
        if image_counter >=6: image_counter = 0
    else:
        r = rospy.Rate(4.0)
        while latest_image is None:
            if rospy.is_shutdown():
                sys.exit(0)
            r.sleep()
        img = latest_image
        latest_image = None  # not thread safe, but doesn't really matter here
        cv_image = bridge.imgmsg_to_cv2(img, desired_encoding="passthrough")
        cv2.imwrite("image"+str(image_counter)+".png", cv_image)
        image_counter += 1
        print "written"
    return cv_image

class BoardDetector:
    def __init__(self):
        #self.cv_image = get_img()
        pass
    def calculate_board_parameters(self, image=None):
        print("Detect bord parameters")
        if image is not None:
            self.cv_image = image
        #self.cv_image = self.cv_image.astype(np.uint8)
        gray = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2GRAY)
        thresh = cv2.adaptiveThreshold(gray,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY,11,2)
        edges = cv2.Canny(gray, 100, 200)
        circle_image = np.zeros(shape=edges.shape)

        rows = gray.shape[0]
        circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1, rows / 16,
                                  param1=150, param2=30,
                                  minRadius=1, maxRadius=30)

        if circles is not None:
            circles = np.uint16(np.around(circles))
            for i in circles[0, :]:
                center = (i[0], i[1])
                # circle center
                cv2.circle(gray, center, 1, (0, 100, 100), 3)
                cv2.circle(circle_image, center, 1, (0, 100, 100), 3)
                # circle outline
                radius = i[2]
                cv2.circle(gray, center, radius, (255, 0, 255), 3)
                cv2.circle(circle_image, center, radius, (255, 0, 255), 3)

        self.circles = circles
        print circles

        x = np.zeros(shape=(3,2))

        x[0, 0] = np.min(self.circles[:,:, 0])
        x[0, 1] = np.min(self.circles[:,:, 1])
        print x[0, 0]
        print x[0, 1]
        #x[0,0]=140
        #x[0,1]=132

        x[1,0]=60
        x[1,1]=2
        x[2,0]=0
        x[2,1]=42

        x_start = x[0, 0]
        y_start = x[0, 1]
        for col in range(0, COLUMNS):
            for row in range(0, ROWS):
                cur_x = x_start + x[1, 0] * col
                cur_y = y_start + x[1, 1] * col

                cur_x = cur_x + x[2, 0] * row
                cur_y = cur_y + x[2, 1] * row

                #cv2.circle(self.cv_image, (int(cur_x), int(cur_y)), 3, (0, 255, 255), 5)

        x = np.reshape(x, newshape=6)
        print "Before minimize"
        #res = minimize(self.circle_error, x, method='nelder-mead', options = {'xtol': 1e-8, 'disp': True} )
        res = minimize(self.circle_error, x, method='Powell', options={'xtol': 1e-8, 'disp': True})
        #res = global_minimum(self.circle_error, x)

        print "After minimize"



        x = res['x']
        print "minimized error:", self.circle_error_print(x)
        print self.position_already_used.T
        print np.sum(self.position_already_used)
        x = np.reshape(x, newshape=(3,2))
        print "x[0, 0]", x[0, 0]
        print "x[0, 1]", x[0, 1]
        print "x[1, 0]", x[1, 0]
        print "x[1, 1]", x[1, 1]
        print "x[2, 0]", x[2, 0]
        print "x[2, 1]", x[2, 1]

        for col in range(0, COLUMNS):
            for row in range(0, ROWS):
                cur_x, cur_y = self.get_circle_center_in_image(col, row, x)

                cv2.circle(self.cv_image, (int(cur_x), int(cur_y)), 3, (255, 0, 255), 3)


        self.offsets = x
        self.radius = np.mean(circles[0, :, 2])

        plt.subplot(221)
        plt.imshow(self.cv_image)
        plt.subplot(222)
        plt.imshow(gray, 'gray')
        plt.subplot(223)
        plt.imshow(circle_image, 'gray')
        plt.subplot(224)
        plt.imshow(edges, 'gray')
        plt.show()

        return res['success']
    def circle_error(self, x):
        x_prime=np.reshape(x,newshape=(3,2))
        x_start = x_prime[0,0]
        y_start = x_prime[0,1]

        cumulated_error = 0
        old_col, old_row = 0, 0
        self.position_already_used = np.zeros(shape=(COLUMNS, ROWS))
        for c in self.circles[0, :]:
            min_single_error = None
            for col in range(0, COLUMNS):
                for row in range(0, ROWS):
                    cur_x, cur_y = self.get_circle_center_in_image(col, row, x_prime)

                    single_error = math.sqrt(math.pow(c[0]-cur_x, 2)) + math.sqrt(math.pow(c[1]-cur_y, 2))
                    #single_error = math.pow(c[0] - cur_x, 2) + math.pow(c[1] - cur_y, 2)

                    if (min_single_error>single_error or min_single_error is None) and not self.position_already_used[col, row]:
                        if not min_single_error is None:
                            self.position_already_used[old_col, old_row] = 0

                        old_col, old_row = (col, row)
                        self.position_already_used[col, row] = 1

                        min_single_error = single_error
                    """
                    if (min_single_error > single_error or min_single_error is None) :

                        min_single_error = single_error
                    """
            cumulated_error += min_single_error

        return cumulated_error

    def circle_error_print(self, x):
        x_prime=np.reshape(x,newshape=(3,2))
        x_start = x_prime[0,0]
        y_start = x_prime[0,1]

        cumulated_error = 0
        old_col, old_row = 0, 0
        self.position_already_used = np.zeros(shape=(COLUMNS, ROWS))
        for c in self.circles[0, :]:
            min_single_error = None
            print "circle:", c
            for col in range(0, COLUMNS):
                for row in range(0, ROWS):
                    cur_x, cur_y = self.get_circle_center_in_image(col, row, x_prime)

                    #single_error = math.sqrt(math.pow(c[0]-cur_x, 2)) + math.sqrt(math.pow(c[1]-cur_y, 2))
                    single_error = math.pow(c[0] - cur_x, 2) + math.pow(c[1] - cur_y, 2)

                    if (min_single_error>single_error or min_single_error is None) and not self.position_already_used[col, row]:
                        if not min_single_error is None:
                            self.position_already_used[old_col, old_row] = 0

                        old_col, old_row = (col, row)
                        self.position_already_used[col, row] = 1

                        min_single_error = single_error
                    """
                    if (min_single_error > single_error or min_single_error is None) :

                        min_single_error = single_error
                    """
            cumulated_error += min_single_error
            print "pos:", self.get_circle_center_in_image(old_col, old_row, np.reshape(x, newshape=(3,2)))

        return cumulated_error
    def get_circle_center_in_image(self, col, row, x=None):
        if x is None:
            x = self.offsets
        if use_hardcoded:

            x = np.zeros(shape=(3, 2))
            """
            x[0, 0] = 140
            x[0, 1] = 132
            x[1, 0] = 60
            x[1, 1] = 2
            x[2, 0] = 0
            x[2, 1] = 42
            """
            x[0, 0] = 131.9997903388379
            x[0, 1] = 110.58740105538192
            x[1, 0] = 57.999567317377924
            x[1, 1] = 1.0430934408427115
            x[2, 0] = 0.0006927898225013082
            x[2, 1] = 46.648050929545946

            x_start = x[0, 0]
            y_start = x[0, 1]

            cur_x = x_start + x[1, 0] * col
            cur_y = y_start + x[1, 1] * col

            cur_x = cur_x + x[2, 0] * row
            cur_y = cur_y + x[2, 1] * row
        else:
            x_start = x[0, 0]
            y_start = x[0, 1]
            cur_x = x_start + x[1,0]* col
            cur_y = y_start + x[1,1] * col

            cur_x = cur_x + x[2,0] * row
            cur_y = cur_y + x[2,1] * row

        return cur_x, cur_y


if __name__ == '__main__':
    b_d = BoardDetector()
    b_d.get_board_parameters()
