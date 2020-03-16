import numpy as np
import cv2
#from sklearn.cluster import KMeans

#Predefined colors
TOKEN_PURPLE_REAL = np.array((80, 0, 80))*1.5
TOKEN_RED_REAL = np.array((150, 0, 0))
TOKEN_ORANGE_REAL = np.array((170, 50, 20))
TOKEN_YELLOW_REAL = np.array((255, 150, 100)) 
TOKEN_GREEN_REAL = np.array((70, 170, 30))

TOKEN_PURPLE_SIM = np.array((102, 0, 102))
TOKEN_RED_SIM = np.array((100, 0, 0))
TOKEN_ORANGE_SIM = np.array((100, 50, 0))
TOKEN_YELLOW_SIM = np.array((100, 100, 0)) 
TOKEN_GREEN_SIM = np.array((0, 100, 0)) 

TOKEN_COLOR_SIM = [
    TOKEN_PURPLE_SIM,
    TOKEN_RED_SIM,
    TOKEN_ORANGE_SIM,
    TOKEN_YELLOW_SIM,
    TOKEN_GREEN_SIM,
    TOKEN_ORANGE_SIM,
    TOKEN_RED_SIM,
    TOKEN_PURPLE_SIM,
]

TOKEN_COLOR_REAL = [
    TOKEN_PURPLE_REAL,
    TOKEN_RED_REAL,
    TOKEN_ORANGE_REAL,
    TOKEN_YELLOW_REAL,
    TOKEN_GREEN_REAL,
    TOKEN_ORANGE_REAL,
    TOKEN_RED_REAL,
    TOKEN_PURPLE_REAL,
]

TOKEN_SIZE = [
    True,
    True,
    True,
    True,
    True,
    False,
    False,
    False,
]

X_CUTOFF = 0.2

#an even easier interface
def DETECT_TOKENS(img, sim, tokens):
    hd = HanoiDetector(sim, tokens)
    return hd.detect_positions(img)

class HanoiDetector:
    def __init__(self, sim, tokens):
        self.sim = sim
        self.tokens = tokens

        if sim:
            self.token_colors_org = TOKEN_COLOR_SIM
            self.token_colors = TOKEN_COLOR_SIM
            self.column_width = 120
            left_rod_x = 230
            middle_rod_x = 350
            right_rod_x = 480
            self.column_left = (left_rod_x-self.column_width/2, left_rod_x+self.column_width/2)
            self.column_mid = (middle_rod_x-self.column_width/2, middle_rod_x+self.column_width/2)
            self.column_right = (right_rod_x-self.column_width/2, right_rod_x+self.column_width/2)
            self.y_cutoff = (230,450)
        else:
            self.token_colors_org = TOKEN_COLOR_REAL
            self.token_colors = TOKEN_COLOR_REAL
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
        #if not self.sim: img = self._cluster_colors(img) 
        return [self.detect_token(img, token)[0] for token in range(self.tokens)]

    # Detect a single token, returns its rod and evidence value
    def detect_token(self, img, token):
        color = self.token_colors[token]
        outer = TOKEN_SIZE[token]

        if outer: func = self._func_outer
        else: func = self._func_inner

        probs = np.array([0.,0.,0.])
        probs[0] = self._disk_prob(self._cutoff_img(img, self.column_left,  self.y_cutoff), color, func)
        probs[1] = self._disk_prob(self._cutoff_img(img, self.column_mid,   self.y_cutoff), color, func)
        probs[2] = self._disk_prob(self._cutoff_img(img, self.column_right, self.y_cutoff), color, func)
        print(probs)
        return np.argmax(probs), np.max(probs)

    # Makes an image for each token depicted the evidence
    def visualize_detection(self, img):
        #if not self.sim: img = self._cluster_colors(img) 
        imgs = []
        for token in range(self.tokens):
            color = self.token_colors[token]
            outer = TOKEN_SIZE[token]

            if outer: func = self._func_outer
            else: func = self._func_inner
            imgs.append(self._disk_prob_vis(img, color, func))

        return imgs

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


    '''
    def _cluster_colors(self, img):
        num_token_colors = min(self.tokens, 5)
        num_clusters = num_token_colors+4

        #calc clusters (using only the columns!)
        inital_colors = np.zeros((num_clusters, 3))
        inital_colors[:num_token_colors] = self.token_colors_org[:num_token_colors]
        inital_colors[num_token_colors+0] = np.array([200,75,60]) #red board
        inital_colors[num_token_colors+1] = np.array([245,245,245]) #white background
        inital_colors[num_token_colors+2] = np.array([180,130,76]) #brown poles
        inital_colors[num_token_colors+3] = np.array([15,15,15]) #black tips

        img_cut = self.cut_columns(img)
        height,width,c = img_cut.shape
        cluster = KMeans(n_clusters=num_clusters, max_iter=5)
        cluster.cluster_centers_ = inital_colors
        img_data = np.reshape(img_cut, (height*width,c))
        cluster.fit(img_data)

        #save new colors
        self.token_colors = list(cluster.cluster_centers_[:num_token_colors])
        self.token_colors.append(self.token_colors[2])#orange2
        self.token_colors.append(self.token_colors[1])#red2
        self.token_colors.append(self.token_colors[0])#purple2
        self.token_colors = np.array(self.token_colors)

        #create image with cluster colors
        height,width,c = img.shape
        img_data = np.reshape(img, (height*width,c))
        img_predict = cluster.predict(img_data)
        img = np.array([cluster.cluster_centers_[label] for label in img_predict]).astype(img.dtype)
        img = np.reshape(img, (height,width,c))
        return img
    '''


    def _func_outer(self, x, y, val):
        x = (1.0*x)/self.column_width -0.5
        #return val * (x**4)
        return val if abs(x) > X_CUTOFF else 0

    def _func_inner(self, x, y, val):
        x = (1.0*x)/self.column_width -0.5
        #return val * (-x**2 +0.5)
        return val if abs(x) < X_CUTOFF else -0.5*val

    def _distance(self, c1, c2):
        testo = c2[0] == TOKEN_RED_REAL[0]


        #return -np.linalg.norm(c1-c2)**2 + 1
        c1 = cv2.cvtColor(np.array([[c1]], dtype=np.uint8), cv2.COLOR_RGB2HSV)[0][0]
        c2 = cv2.cvtColor(np.array([[c2]], dtype=np.uint8), cv2.COLOR_RGB2HSV)[0][0]
        c1 = c1.astype(np.float64)/255.0
        c2 = c2.astype(np.float64)/255.0

        d1 = np.abs(c1[0]-c2[0])
        d2 = np.abs(c1[0]-c2[0]+1)
        d3 = np.abs(c1[0]-c2[0]-1)
        d = np.min([d1,d2,d3])


        if c1[2] > 0.9: #or c1[2] < 0.05:
            return 0.
        #if d > 0.2:
        #    return 0.
        return 1./(d**2 +0.01)
        #return 1/(np.linalg.norm(c1-c2) +0.01)

    def _disk_prob(self, img, color, func):
        prob = 0.
        for y in range(img.shape[0]):
            for x in range(img.shape[1]):
                prob += func(x, y, self._distance(img[y,x], color))
        return prob

    def _cutoff_img(self, img, x_cut, y_cut):
        return img[y_cut[0]:y_cut[1],x_cut[0]:x_cut[1]]
        

    def _disk_prob_vis(self, img, color, func):
        imgp = np.zeros_like(img).astype(np.float64)
        for cutoff in [self.column_left, self.column_mid, self.column_right]:
            img_cut = self._cutoff_img(img, cutoff, self.y_cutoff)
            for y in range(img_cut.shape[0]):
                for x in range(img_cut.shape[1]):
                    imgp[y+self.y_cutoff[0],x+cutoff[0],:] = func(x, y, self._distance(img_cut[y,x], color))
        imgp = self.cut_columns(imgp)
        #normalize and convert image
        imgp = imgp - np.min(imgp)
        if np.max(imgp) > 1e-8: imgp = imgp/np.max(imgp)
        return imgp





