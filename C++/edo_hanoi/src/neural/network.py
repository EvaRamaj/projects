import keras
from keras.models import *
from keras.layers import *
import csv
import cv2

#Config of the final network:
#True (USE_GRAY)
#1 (CHANNELS)
#True (ONLY_ONE_ROD)

#use gray images instead of colored ones
USE_GRAY = True
#number of channels in an image (only 1 or 3)
CHANNELS = 1
#whether to take all 3 rods as input, or only one rod at a time
ONLY_ONE_ROD = True

def get_network_config():
    return {'use_gray' : USE_GRAY, 'channels' : CHANNELS, 'only_one_rod' : ONLY_ONE_ROD}


COLUMN_WIDTH = 100
COLUMN_HEIGHT = 146


def create_model():
    if ONLY_ONE_ROD:
        width = COLUMN_WIDTH
        num_classes = 2
    else:
        width = COLUMN_WIDTH*3
        num_classes = 4

    img_input = Input(shape=(COLUMN_HEIGHT,width,CHANNELS), dtype="float64", name='img_input')
    x = img_input
    x = Conv2D(8, 5, activation="relu")(x)
    x = BatchNormalization()(x)
    x = MaxPooling2D((2,2))(x)
    x = Dropout(0.2)(x)
    x = Conv2D(16, 5, activation="relu")(x)
    x = BatchNormalization()(x)
    x = MaxPooling2D((4,4))(x)
    x = Dropout(0.2)(x)

    x = Flatten()(x)
    cnn_out = Dense(32, activation="relu")(x)
    x = Dropout(0.1)(x)
    cnn_out = Dense(16, activation="relu")(x)
    x = Dropout(0.1)(x)

    token_models = []
    for i in range(8):
        token_name = "token_%s" % i
        x = Dense(num_classes, activation="softmax", name=token_name)(cnn_out)
        token_models.append(x)

    return Model(inputs=[img_input], outputs=token_models)


def load_data(filename):
    csvfile = open(filename)
    reader = csv.reader(csvfile)
    imgs = []
    truths = []

    for i in range(8):
        truths.append([])

    for line in reader:
        #data augmentation DA1 - flip
        #data augmentation DA2 - brighter
        #data augmentation DA3 - darker

        img = cv2.imread("imgs/%s" % line[0])
        
        if ONLY_ONE_ROD:
            img_cuts = cut_columns(img, separate=True)
            
            tokens_on_rod = [[],[],[]]
            for i,rod in enumerate(line[1:]):
                if int(rod) != 3:
                    tokens_on_rod[int(rod)].append(i)
            
            for k in range(3):
                tokens = tokens_on_rod[k]
                if not tokens:#ignore empty poles
                    continue
                    
                for i in range(8):
                    if i in tokens:
                        truths[i].append([1,0])
                        #truths[i].append([1,0])#DA1
                        truths[i].append([1,0])#DA2
                        truths[i].append([1,0])#DA3
                    else:
                        truths[i].append([0,1])
                        #truths[i].append([0,1])#DA1
                        truths[i].append([0,1])#DA2
                        truths[i].append([0,1])#DA3
                
                img = img_cuts[k]
                imgs.append(img)
                #DA1
                #imgs.append(cv2.flip(img, 1))
                #DA2
                imgs.append(change_brightness(img, 80))
                #DA3
                imgs.append(change_brightness(img, -80))
                  
        else:
            img_cut = cut_columns(img, separate=False)
            imgs.append(img_cut)
            for i,rod in enumerate(line[1:]):
                pos = [0,0,0,0] #4 classes: 3 rods and not present
                pos[int(rod)] = 1
                truths[i].append(pos)

                #DA1
                pos_rev = list(reversed(pos[:3]))
                pos_rev.append(pos[3])
                truths[i].append(pos_rev)
                #DA2, DA3
                truths[i].append(pos)
                truths[i].append(pos)

            #DA1
            imgs.append(cv2.flip(img_cut, 1))
            #DA2
            imgs.append(change_brightness(img_cut, 50))
            #DA3
            imgs.append(change_brightness(img_cut, -50))
    
    #this means grey image with only one channel
    if USE_GRAY:
        for i in range(len(imgs)):
            img = cv2.cvtColor(imgs[i], cv2.COLOR_BGR2GRAY)
            if CHANNELS == 1: img = np.expand_dims(img, axis=2)
            else: img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
            imgs[i] = img
    imgs = np.array(imgs)
    #normalize to -0.5,0.5
    imgs = imgs.astype(np.float64)/255.0 - 0.5
    for i in range(8):
        truths[i] = np.array(truths[i])

    return [imgs],truths


def change_brightness(img, v):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    if v > 0:
        hsv[:,:,2] = np.clip(hsv[:,:,2], 0, 255-v) + v
    else:
        hsv[:,:,2] = np.clip(hsv[:,:,2], -v, 255) + v
    return cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)

def change_saturation(img, v):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    if v > 0:
        hsv[:,:,1] = np.clip(hsv[:,:,1], 0, 255-v) + v
    else:
        hsv[:,:,1] = np.clip(hsv[:,:,1], -v, 255) + v
    return cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)



def cut_columns(img, separate=False):
    column_width = COLUMN_WIDTH
    left_rod_x = 205
    middle_rod_x = 315
    right_rod_x = 427
    column_left = (left_rod_x-column_width//2, left_rod_x+column_width//2)
    column_mid = (middle_rod_x-column_width//2, middle_rod_x+column_width//2)
    column_right = (right_rod_x-column_width//2, right_rod_x+column_width//2)
    y_cutoff = (260,260+COLUMN_HEIGHT)


    if separate:
        height, width = y_cutoff[1]-y_cutoff[0],column_width
        cut_imgs = [np.zeros((height,width,img.shape[2]), dtype=img.dtype),
                   np.zeros((height,width,img.shape[2]), dtype=img.dtype),
                   np.zeros((height,width,img.shape[2]), dtype=img.dtype)]

        cut_imgs[0][:,0:column_width] = img[y_cutoff[0]:y_cutoff[1],column_left[0]:column_left[1]]
        cut_imgs[1][:,0:column_width] = img[y_cutoff[0]:y_cutoff[1],column_mid[0]:column_mid[1]]
        cut_imgs[2][:,0:column_width] = img[y_cutoff[0]:y_cutoff[1],column_right[0]:column_right[1]]
        return cut_imgs
    else:
        height, width = y_cutoff[1]-y_cutoff[0],column_width*3
        cut_img = np.zeros((height,width,img.shape[2]), dtype=img.dtype)
        

        cut_img[:,column_width*0:column_width*1] = img[y_cutoff[0]:y_cutoff[1],column_left[0]:column_left[1]]
        cut_img[:,column_width*1:column_width*2] = img[y_cutoff[0]:y_cutoff[1],column_mid[0]:column_mid[1]]
        cut_img[:,column_width*2:column_width*3] = img[y_cutoff[0]:y_cutoff[1],column_right[0]:column_right[1]]
        return cut_img