import numpy as np
import keras
from keras import backend as K
import cv2

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import time

import sys
sys.path.insert(0, 'neural')
import network

model_path = "neural/cnn.model"
    

#interface to use in hanoi_state_server
latest_image = None
def detect_positions(sim, tokens):
    num_tokens = tokens

    global latest_image
    def callback(img):
        global latest_image
        latest_image = img

    img_sub = rospy.Subscriber("/edo/asus_xtion/rgb/image_raw", Image, callback)
    bridge = CvBridge()

    K.clear_session()
    hd = HanoiDetector(sim, tokens)

    r = rospy.Rate(0.5)
    while not rospy.is_shutdown():
        r.sleep()
        while latest_image is None:
            if rospy.is_shutdown():
                sys.exit(0)
            r.sleep()
        img = latest_image
        latest_image = None  # not thread safe, but doesn't really matter here

        cv_image = bridge.imgmsg_to_cv2(img, desired_encoding="passthrough")
        print("Picture Taken")
        return hd.detect_positions(cv_image)

class HanoiDetector:
    def __init__(self, sim, tokens):
        if sim:
            raise ValueError("Sim is set to true. This hanoi detector only works with the real robot!")

        self.sim = sim
        self.tokens = tokens

        if sim:
            self.column_width = 120
            left_rod_x = 230
            middle_rod_x = 350
            right_rod_x = 480
            self.column_left = (left_rod_x-self.column_width/2, left_rod_x+self.column_width/2)
            self.column_mid = (middle_rod_x-self.column_width/2, middle_rod_x+self.column_width/2)
            self.column_right = (right_rod_x-self.column_width/2, right_rod_x+self.column_width/2)
            self.y_cutoff = (230,450)
        else:
            self.column_width = 100
            left_rod_x = 205
            middle_rod_x = 315
            right_rod_x = 427
            self.column_left = (left_rod_x-self.column_width/2, left_rod_x+self.column_width/2)
            self.column_mid = (middle_rod_x-self.column_width/2, middle_rod_x+self.column_width/2)
            self.column_right = (right_rod_x-self.column_width/2, right_rod_x+self.column_width/2)
            self.y_cutoff = (260,406)


        self.model = keras.models.load_model(model_path)
        self.config = network.get_network_config()


    # Detect all tokens, returns their rods
    def detect_positions(self, img):
        if self.config['use_gray']:
            img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            if self.config['channels'] == 1:
                img = np.expand_dims(img, axis=2)
            else:
                img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        #img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

        img = img/255.0 - 0.5

        if not self.config['only_one_rod']:
            img_cut = self.cut_columns(img)
            pred = self.model.predict(np.array([img_cut]))
            positions = []
            for i in range(8):
                positions.append(np.argmax(pred[i][0]))
            return positions
        else:
            img_cuts = self.cut_columns(img, separate=True)

            preds = []
            probs = []
            for token in range(8):
                preds.append([])
                probs.append([])

            for img in img_cuts:
                pred = self.model.predict(np.array([img]))
                for token in range(8):
                    #add prob that token is at that rod
                    print(token, pred[token][0])
                    preds[token].append(np.argmax(pred[token][0]))
                    probs[token].append(pred[token][0][0])

            positions_present = []
            positions_all = []
            for token in range(8):
                if (preds[token][0] == preds[token][1] and
                    preds[token][1] == preds[token][2] and
                    preds[token][2] == 1):
                    positions_all.append(-1)
                else:
                    positions_present.append(np.argmax(probs[token]))
                    positions_all.append(np.argmax(probs[token]))

            print(positions_all)
            return positions_all

    # Makes an image for each token depicted the evidence
    def visualize_detection(self, img):
        #nothing to visualize here
        return []

    # Returns an image depicting the columns
    def visualize_cutoffs(self, img):
        lines_img = np.copy(img)
        lines_img[:,self.column_left[0]-5:self.column_left[0]+5] = (255,0,0)
        lines_img[:,self.column_left[1]-5:self.column_left[1]+5] = (255,0,0)
        lines_img[:,self.column_mid[0]-5:self.column_mid[0]+5] = (0,255,0)
        lines_img[:,self.column_mid[1]-5:self.column_mid[1]+5] = (0,255,0)
        lines_img[:,self.column_right[0]-5:self.column_right[0]+5] = (0,0,255)
        lines_img[:,self.column_right[1]-5:self.column_right[1]+5] = (0,0,255)
        lines_img[self.y_cutoff[0]-5:self.y_cutoff[0]+5,:] = (0,0,0)
        lines_img[self.y_cutoff[1]-5:self.y_cutoff[1]+5,:] = (0,0,0)
        return lines_img

    def cut_columns(self, img, separate=False):
        if separate:
            height, width = self.y_cutoff[1]-self.y_cutoff[0],self.column_width
            cut_imgs = [np.zeros((height,width,img.shape[2]), dtype=img.dtype),
                       np.zeros((height,width,img.shape[2]), dtype=img.dtype),
                       np.zeros((height,width,img.shape[2]), dtype=img.dtype)]

            cut_imgs[0][:,0:self.column_width] = img[self.y_cutoff[0]:self.y_cutoff[1],self.column_left[0]:self.column_left[1]]
            cut_imgs[1][:,0:self.column_width] = img[self.y_cutoff[0]:self.y_cutoff[1],self.column_mid[0]:self.column_mid[1]]
            cut_imgs[2][:,0:self.column_width] = img[self.y_cutoff[0]:self.y_cutoff[1],self.column_right[0]:self.column_right[1]]
            return cut_imgs
        else:
            height, width = self.y_cutoff[1]-self.y_cutoff[0],self.column_width*3
            cut_img = np.zeros((height,width,img.shape[2]), dtype=img.dtype)

            cut_img[:,self.column_width*0:self.column_width*1] = img[self.y_cutoff[0]:self.y_cutoff[1],self.column_left[0]:self.column_left[1]]
            cut_img[:,self.column_width*1:self.column_width*2] = img[self.y_cutoff[0]:self.y_cutoff[1],self.column_mid[0]:self.column_mid[1]]
            cut_img[:,self.column_width*2:self.column_width*3] = img[self.y_cutoff[0]:self.y_cutoff[1],self.column_right[0]:self.column_right[1]]
            return cut_img


    def _cutoff_img(self, img, x_cut, y_cut):
        return img[y_cut[0]:y_cut[1],x_cut[0]:x_cut[1]]
        




