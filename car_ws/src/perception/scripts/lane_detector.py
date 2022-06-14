#! /usr/bin/python
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import cv2

# Instantiate CvBridge
bridge = CvBridge()

def image_callback(msg):
    print("Received an image!")
    try:
        # Convert your ROS Image message to OpenCV2
        cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError:
        print("An error occured!")
    else:
        # Save your OpenCV2 image as a jpeg 
        time = msg.header.stamp
        upper_range = np.array([20, 100, 100])
        lower_range = np.array([30, 255, 255])

        img = cv2_img
        h, w, _ = img.shape
        img = img[h-400:h-300, 200:w-350]
        img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(img_hsv, upper_range, lower_range)
        output_img = cv2.bitwise_and(img, img, mask=mask)
        
        # cv2.namedWindow("Detected lanes", cv2.WINDOW_NORMAL) 
        # cv2.imshow("Detected lanes", output_img)
        # cv2.waitKey(0)

        M = cv2.moments(mask)
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])

        rospy.sleep(1)

def main():
    rospy.init_node('image_listener')
    # Define your image topic
    image_topic = "/prius/front_camera/image_raw"
    # Set up your subscriber and define its callback
    rospy.Subscriber(image_topic, Image, image_callback)
    # Spin until ctrl + c
    rospy.spin()

if __name__ == '__main__':
    main()