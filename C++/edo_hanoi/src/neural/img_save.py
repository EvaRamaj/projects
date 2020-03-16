import cv2
import rospy
import csv
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


if __name__ == '__main__':
    rospy.init_node("img_save_node")

    latest_image = None
    def callback(img):
        global latest_image
        latest_image = img

    img_sub = rospy.Subscriber("/edo/asus_xtion/rgb/image_raw", Image, callback)
    bridge = CvBridge()

    csvfile = open("groundtruth.csv")
    reader = csv.reader(csvfile)

    r = rospy.Rate(0.5)

    #get one img first, display and wait for input
    while latest_image is None:
        if rospy.is_shutdown():
            sys.exit(0)
        r.sleep()
    img = latest_image
    latest_image = None #not thread safe, but doesn't really matter here
    cv_image = bridge.imgmsg_to_cv2(img, desired_encoding="passthrough")
    cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)

    for line in reader:
        print("Next up: %s with expected ground truth %s" % (line[0], line[1:]))
        cv2.imshow("img", cv_image)
        cv2.waitKey()

        while latest_image is None:
            if rospy.is_shutdown():
                sys.exit(0)
            r.sleep()
        img = latest_image
        latest_image = None #not thread safe, but doesn't really matter here

        cv_image = bridge.imgmsg_to_cv2(img, desired_encoding="passthrough")
        cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        cv2.imwrite("imgs/%s" % line[0], cv_image)
        
    cv2.imshow("img", cv_image)
    cv2.waitKey()
        


