import numpy as np
import cv2


#an even easier interface
def DETECT_TOKENS(img, sim, tokens):
    hd = HanoiDetector(sim, tokens)
    return hd.detect_positions(img)

class HanoiDetector:
    def __init__(self, sim, tokens):
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


    # Detect all tokens, returns their rods
    def detect_positions(self, img):
        edges = cv2.cvtColor(self.cut_columns(img), cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(edges, 50, 150)
        return [] #TODO

    # Makes an image for each token depicted the evidence
    def visualize_detection(self, img):
        edges = cv2.cvtColor(self.cut_columns(img), cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(edges, 50, 150)
        return [edges]

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

    def cut_columns(self, img):
        height, width = self.y_cutoff[1]-self.y_cutoff[0],self.column_width*3
        cut_img = np.zeros((height,width,3), dtype=img.dtype)

        cut_img[:,self.column_width*0:self.column_width*1] = img[self.y_cutoff[0]:self.y_cutoff[1],self.column_left[0]:self.column_left[1]]
        cut_img[:,self.column_width*1:self.column_width*2] = img[self.y_cutoff[0]:self.y_cutoff[1],self.column_mid[0]:self.column_mid[1]]
        cut_img[:,self.column_width*2:self.column_width*3] = img[self.y_cutoff[0]:self.y_cutoff[1],self.column_right[0]:self.column_right[1]]
        return cut_img


    def _cutoff_img(self, img, x_cut, y_cut):
        return img[y_cut[0]:y_cut[1],x_cut[0]:x_cut[1]]
        




