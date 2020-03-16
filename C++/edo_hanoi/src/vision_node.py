import cv2
from matplotlib import pyplot as plt
import rospy
import numpy as np
from scipy.cluster.vq import kmeans2 as KMeans
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import sys
import time

import hanoi_detector

#whether or not to use the column detector
USE_COLUMN = True

class vision_node:
    color_name_array =np.array(['orange', 'red', 'purple', 'green', 'yellow', 'beige', 'bottom_red', 'white', 'black'])

    initial_cluster_centers = np.array(
        [[126, 0, 0],
        [96, 0, 0],
        [57, 0, 33],
        [41, 65, 0],
        [165, 115, 0],
        [166, 115, 66],
        [186, 42, 41],
        [214, 191, 179],
        [12, 5, 7]
        ])




    def __init__(self, cv_image, cluster_size, num_tokens):
            print "init"
            self.cv_image = np.copy(cv_image)
            self.num_tokens = num_tokens
            # cv_image is a numpy array of shape 640x480x3
            labels, partings, partings_number = self.create_partings(cv_image, cluster_size)

            likeliness = np.zeros(shape=(8, 8))
            self.getCenterLikeliness(partings, partings_number)
            """
            pixels_in_token = (
            ("bigPurple", 1237), ("bigRed", 1174), ("bigOrange", 1206), ("yellow", 795), ("green", 698),
            ("smallOrange", 632), ("smallRed", 599), ("smallPurple", 402))
            print
            pixels_in_token

            # try to find the position
            current_token = 0
            count_purple = 0
            count_green = 0
            count_yellow = 0
            count_red_orange = 0
            position = 8
            for i in range(25, 144, 16):
                position = position - 1
                for j in range(0, 355):
                    if labels[i][j] == 2:
                        count_purple = count_purple + 1
                    if labels[i][j] == 3:
                        count_green = count_green + 1
                    if labels[i][j] == 4:
                        count_yellow = count_yellow + 1
                    if labels[i][j] == 0 or labels[i][j] == 1:
                        count_red_orange = count_red_orange + 1
                current_token = max(count_purple, count_green, count_yellow, count_red_orange)
                if current_token == count_red_orange:
                    if current_token * 14 <= 800:
                        print
                        "I found small red or orange in position:", position
                    else:
                        print
                        "I found a big orange or red in position:", position
                if current_token == count_purple:
                    if current_token * 14 <= 500:
                        print
                        "I found a small purple token in position:", position
                    else:
                        print
                        "I found a big purple token in position: ", position
                if current_token == count_green:
                    print
                    "I found a green token in position:", position
                if current_token == count_yellow:
                    print
                    "I found a yellow token in position:", position
                count_purple = 0
                count_green = 0
                count_yellow = 0
                count_red_orange = 0
                current_token = 0
                i = i - 16
            """
            # PLEASE DO NOT REMOVE THESE COMMENTS..WE MAY NEED THEM AGAIN
            # go through the labels and find the pixels of each token
            """counter1 = 0
            count_bigPurple = 0
            for i in range(144,130,-1):
                counter1 = counter1 + 1
                for j in range(0,355):
                    if labels[i][j] == 2:
                        count_bigPurple = count_bigPurple + 1
            print counter1
            print "this is the pixel_bigPurpleCounter"
            print count_bigPurple #1237

            counter2 =0
            count_bigRed = 0
            for i in range(129,115,-1):
                counter2 = counter2 + 1
                for j in range(0,355):
                    if labels[i][j] == 0 or labels[i][j] == 1 :
                        count_bigRed = count_bigRed + 1
            print counter2
            print "this is the pixel_bigRedCounter"
            print count_bigRed #1113

            count_bigOrange = 0
            counter = 0
            for i in range(114,100,-1):
                counter = counter + 1
                for j in range(0,355):
                    if labels[i][j] == 0 or labels[i][j] == 1 :
                        count_bigOrange = count_bigOrange + 1
            print counter
            print "this is the pixel_bigOrangeCounter"
            print count_bigOrange #1206

            count_yellow = 0
            counter3 = 0
            for i in range(99,85,-1):
                counter3 = counter3 + 1
                for j in range(0,355):
                    if labels[i][j] == 4:
                        count_yellow = count_yellow + 1
            print counter
            print "this is the pixel_yellow"
            print count_yellow #1206

            count_green = 0
            counter4 = 0
            for i in range(84,70,-1):
                counter4 = counter4 + 1
                for j in range(0,355):
                    if labels[i][j] == 3 :
                        count_green = count_green + 1
            print counter4
            print "this is the pixel_green"
            print count_green #1206

            count_smallOrange = 0
            counter5 = 0
            for i in range(69,55,-1):
                counter5 = counter5 + 1
                for j in range(0,355):
                    if labels[i][j] == 0 or labels[i][j] == 1 :
                        count_smallOrange = count_smallOrange + 1
            print counter5
            print "this is the pixel_smallOrnage"
            print count_smallOrange #1206

            count_smallRed = 0
            counter6 = 0
            for i in range(54,40,-1):
                counter6 = counter6 + 1
                for j in range(0,355):
                    if labels[i][j] == 0 or labels[i][j] == 1 :
                        count_smallRed = count_smallRed + 1
            print counter6
            print "this is the pixel_smallRed"
            print count_smallRed #1206

            count_smallPurple = 0
            counter7 = 0
            for i in range(39,25,-1):
                counter7 = counter7 + 1
                for j in range(0,355):
                    if labels[i][j] == 2 :
                        count_smallPurple = count_smallPurple + 1
            print counter7
            print "this is the pixel_smallPurple"
            print count_smallPurple #1206"""
            """
            plt.subplot(221), plt.imshow(partings[0])
            plt.subplot(222), plt.imshow(partings[1])
            plt.subplot(223), plt.imshow(partings[2])
            plt.subplot(224), plt.imshow(cv_image)

            plt.show()
            """
            """edges = cv2.Canny(contour_image_array[4],10,200)
            imgray = cv2.cvtColor(clustered_img, cv2.COLOR_BGR2GRAY)
            ret, thresh = cv2.threshold(imgray, 127, 255, 0)
            im2, contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            #edges = cv2.Canny(contour_image_array[0],0,0)
            cv2.drawContours(im2, contours, -1, (0,255,0), 3)

            imgray = cv2.cvtColor(clustered_img, cv2.COLOR_BGR2GRAY)
            ret, thresh = cv2.threshold(imgray, 127, 255, 0)
            im2, contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

            plt.subplot(441),plt.imshow(cv_image)

            plt.subplot(442),plt.imshow(new_selection)
            plt.subplot(443),plt.imshow(color_image,cmap = 'gray')
            plt.subplot(442),plt.imshow(im2)
            #plt.subplot(443),plt.imshow(edges,cmap = 'gray')

            for i in range(0,cluster_size):
                plt.subplot(4,4,i+4), plt.imshow(color_image_array[i])"""

            # print('Cube pose found, sending transform!')
            # pose_pub.sendTransform(tvec, quaternion,
            #    rospy.Time.now(),
            #    "/edo/cube",
            #    "/edo/camera_opencv")

            # for visualization
            # for i in range(len(ids)):
            #    cv2.aruco.drawDetectedMarkers(cv_image, corners, ids);
            #    cv2.aruco.drawAxis(cv_image, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], 0.1);
            # cv2.imshow("image", cv_image)
            # cv2.waitKey(0)

    def create_partings(self, cv_image, cluster_size):
        print "create Partings"
        top_left_corner = 250, 135
        height = 400 - 250
        width = 490 - 135
        # selection = np.zeros((width, height, 3), np.uint8)
        selection = cv_image[top_left_corner[0]:top_left_corner[0] + height, top_left_corner[1]:top_left_corner[1] + width]
        new_selection = np.zeros(selection.shape, np.uint8)
        alpha = 1.
        beta = 0
        # left middle and right selections have the same size
        for i in range(0, selection.shape[0]):
            for j in range(0, selection.shape[1]):
                new_selection[i, j] = alpha * selection[i, j] + beta
        train_selection = np.reshape(selection, newshape=(selection.shape[0] * selection.shape[1], selection.shape[2]))
        print self.initial_cluster_centers
        centroid, labels = KMeans(data=train_selection.astype(float), k=self.initial_cluster_centers.astype(float), iter=500,
                                  minit='matrix')
        print "centroid"
        print centroid
        self.centroid = centroid
        labels = np.reshape(labels, newshape=selection.shape[:-1])
        clustered_img = np.zeros((cv_image.shape[0], cv_image.shape[1], 3), np.uint8)
        contour_image_array = np.zeros((cluster_size, cv_image.shape[0], cv_image.shape[1]), np.uint8)
        color_image_array = np.zeros((cluster_size, cv_image.shape[0], cv_image.shape[1], 3), np.uint8)
        color_image = np.zeros((cv_image.shape[0], cv_image.shape[1], 3), np.uint8)
        center_of_rod = np.array([[405, 204], [405, 316], [405, 426]])
        width = center_of_rod[1, 1] - center_of_rod[0, 1]
        height = 405 - 265
        partings = np.zeros((3, height, width, 3), np.uint8)
        partings_number = np.full(shape=(3, height, width), fill_value=-1, dtype=np.uint8)
        pixel_counter = np.zeros((3, 5))
        print selection.shape
        print partings.shape  # (3, 140, 112, 3)
        print top_left_corner
        print center_of_rod[0]

        for row in range(0, selection.shape[0]):
            for column in range(0, selection.shape[1]):
                # r = selection[row, column].reshape(1, -1)
                cluster_number = labels[row, column]
                c_r = centroid[cluster_number]
                # clustered_img[top_left_corner_of_right_rod[0]+row,top_left_corner_of_right_rod[1]+column]=c[0]

                color_image[top_left_corner[0] + row, top_left_corner[1] + column] = c_r
                contour_image_array[cluster_number, top_left_corner[0] + row, top_left_corner[1] + column] = 1
                color_image_array[cluster_number, top_left_corner[0] + row, top_left_corner[1] + column] = c_r
                
                self.cv_image[row+top_left_corner[0], column+top_left_corner[1]] = c_r
                            
                for i in range(0, len(partings)):
                    row_in_partings = int(top_left_corner[0] + row - (center_of_rod[i, 0] - height))
                    column_in_partings = int(top_left_corner[1] + column - (center_of_rod[i, 1] - width / 2))
                    if 0 <= row_in_partings < len(partings[i]) and 0 <= column_in_partings < len(partings[i, 0]):
                        if not np.array_equal(c_r, centroid[-1]) and not np.array_equal(c_r, centroid[-2]) and not np.array_equal(c_r, centroid[-3]) and not np.array_equal(c_r, centroid[-4]):
                            partings[i, row_in_partings, column_in_partings] = c_r
                            partings_number[i, row_in_partings, column_in_partings] = cluster_number

                            pixel_counter[i, cluster_number] += 1

        """for i in range(0, len(pixel_counter)):
            print str(i)+"th column:"
            for j in range(0, len(pixel_counter[i])):
                print "Color: " + str(color_name_array[j]) + " pixels: " + str(pixel_counter[i,j])"""

        return labels, partings, partings_number

    def order_by_pixel_number(self, pixel_numbers):

        arg_sort=np.zeros(shape=(pixel_numbers.shape[-1], pixel_numbers.shape[0], pixel_numbers.shape[1]))
        for i in range(pixel_numbers.shape[-1]):
            re = np.reshape(a = pixel_numbers[:, :, i], newshape=(pixel_numbers.shape[0], pixel_numbers.shape[1]))
            sorted_temp = np.argsort(re)
            arg_sort[i] = sorted_temp

        return arg_sort

    def order_by_width(self, ring_width):
        arg_sort = np.zeros(shape=(pixel_numbers.shape[-1], pixel_numbers.shape[0], pixel_numbers.shape[1]))
        for i in range(pixel_numbers.shape[-1]):
            re = np.reshape(a=pixel_numbers[:, :, i], newshape=(pixel_numbers.shape[0], pixel_numbers.shape[1]))

            sorted_temp = np.argsort(re)
            arg_sort[i] = sorted_temp

        return arg_sort

    def maximize_position_reward(self, probability_matrix):
        one_d_prob = np.zeros(shape=(8, 24))
        order = np.zeros(shape=(8,24))
        for i in range(0,8):
            one_d_prob[i]= np.reshape(probability_matrix[:, i, :], newshape=24)
            order[i] = np.argsort(one_d_prob[i])


        optimizer_steps = np.zeros(shape=8, dtype=int)
        order = order.astype(dtype=int)
        positions = order[:,-1]

        while len(np.unique(positions))!=len(positions):
            print(np.unique(positions))
            print "positions"
            print(positions)
            for i in range (0,8-1):
                for j in range(i+1,8):
                    if positions[i]==positions[j]:
                        print i,j
                        print positions[i]
                        print positions[j]
                        print positions
                        if one_d_prob[i, positions[i]]<one_d_prob[j, positions[j]]:
                            optimizer_steps[i]+=1
                            positions[i]=order[i,-(1+optimizer_steps[i])]
                        else:
                            optimizer_steps[j]+=1
                            positions[j] = order[j, -(1 + optimizer_steps[j])]
        ring_positions = np.zeros(shape=(8))
        for i in range(0,8):
            ring_positions[i] = np.floor(positions[i]/8)
        print "probability_matrix"
        print probability_matrix

        print "ring_positions"
        print ring_positions

        return ring_positions

    def get_highest_probable_positions(self, pixel_number, ring_width):
        color_name_array = np.array(
            ['orange', 'red', 'purple', 'green', 'yellow', 'beige', 'bottom_red', 'white', 'black'])

        ring_prop = np.array([[2, 35], [1, 45], [0, 49], [3, 64], [4, 69], [0, 81], [1, 95],[2,100]])
        probability_matrix = np.zeros(shape=(3,8,8))
        print pixel_number.shape
        print ring_width.shape
        np.set_printoptions(precision=3)
        for rod in range(0, probability_matrix.shape[0]):
            for row in range(0, probability_matrix.shape[1]):
                for ring in range(0, probability_matrix.shape[2]):
                    distance = np.power(ring_width[rod, row]-ring_prop[ring, 1], 2)
                    color_number = pixel_number[rod, row, ring_prop[ring,0]]
                    probability_matrix[rod, row, ring]=color_number / (1+ distance)
                    print rod, row, ring, color_number, distance, color_number / (1+ distance)
        print pixel_number
        print ring_width
        print probability_matrix
        return self.maximize_position_reward(probability_matrix)

    def getCenterLikeliness(self, partings, partings_number):
        likeliness_i = np.zeros(shape=(8))
        pixel_numbers = np.zeros(shape=(3,8,6), dtype=int)
        ring_width = np.zeros(shape=(3,8))
        print partings_number.shape
        print partings.shape
        for rod in range(0,3):
            for ring_boundary in [(10,23,0),(23,39,1),(39,54,2),(54,68,3),(68,83,4),(83,102,5),(102, 115,6),(115,131,7)]:
                for row in range(ring_boundary[0], ring_boundary[1]):
                    for column in range(0, partings.shape[2]):
                        #print rod, row, column, partings_number[rod, row, column]
                        color = partings_number[rod, row, column]
                        if color!=255:
                            pixel_numbers[rod, ring_boundary[2], color]+=1

                ring_width[rod, ring_boundary[2]] = np.sum(pixel_numbers[rod, ring_boundary[2],:]) / (ring_boundary[1]- ring_boundary[0])

        print "pixel_numbers"
        print pixel_numbers
        print "ring width"
        print ring_width
        print "order"
        sorted_by_color = self.order_by_pixel_number(pixel_numbers)
        #sorted_by_width = self.order_by_width(ring_width)

        if USE_COLUMN:
            self.result = hanoi_detector.hanoi_detect(self.num_tokens, self.cv_image, self.centroid)
        else:
            self.result = self.get_highest_probable_positions(pixel_numbers, ring_width)
            self.result = list(reversed(self.result))
        print(self.result)


        #plt.subplot(221), plt.imshow(partings[0])
        #plt.subplot(222), plt.imshow(partings[1])
        #plt.subplot(223), plt.imshow(partings[2])
        #plt.subplot(224), plt.imshow(self.cv_image)
        #plt.show()


latest_image = None

def detect_positions(sim, tokens):
    num_tokens = tokens
    # We add one color for the background, one colour for the base and one color for the rod
    #cluster_size = num_tokens + 1
    cluster_size = 9

    global latest_image
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
        latest_image = None  # not thread safe, but doesn't really matter here

        cv_image = bridge.imgmsg_to_cv2(img, desired_encoding="passthrough")
        print("Picture Taken")
        try:
            vn = vision_node(cv_image, cluster_size, tokens)
            return vn.result
        except:
            print("Error")
            pass
        print "end"