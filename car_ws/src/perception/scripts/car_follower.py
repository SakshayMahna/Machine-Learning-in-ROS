from decimal import DivisionByZero
from math import sqrt
import rospy
from sensor_msgs.msg import Image
from prius_msgs.msg import Control
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import cv2

# Instantiate CvBridge
bridge = CvBridge()

# Initialize OpenCV tracker
tracker = cv2.TrackerKCF_create()

# Command Variables
command_variables = {
    'current_center_value': 0,
    'center_value': None,
    'previous_error': 0,
    'time': None
}

# Image Callback
def image_callback(msg):
    global command_variables

    # rospy.loginfo("Received an image!")
    try:
        # Convert your ROS Image message to OpenCV2
        cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError:
        rospy.loginfo("An error occured!")
    else:
        # Process Image
        time = msg.header.stamp
        command_variables['time'] = time
        img = cv2_img
        h, w, _ = img.shape

        bbox = (332, 359, 136, 112)
        if command_variables['center_value'] == None:
            # bbox = cv2.selectROI(img, False)
            bbox = (332, 359, 136, 112)
            tracker.init(img, bbox)
        else:
            _, bbox = tracker.update(img)

        output_img = np.copy(img)
        (x, y, w, h) = [int(v) for v in bbox]
        cv2.rectangle(output_img, (x, y), (x + w, y + h), (0, 255, 0), 2)
        cv2.imwrite(str(time) + '.jpeg', output_img)

        cX = int(bbox[0] + bbox[2] / 2)
        cY = int(bbox[1] + bbox[3] / 2) 

        command_variables['current_center_value'] = cX
        if command_variables['center_value'] == None:
            command_variables['center_value'] = cX

        

def main():
    global command_variables

    kp, kd = 0.011, 0.011

    rospy.init_node('car_follower')
    # Define your image topic
    image_topic = "follower/prius/front_camera/image_raw"
    # Set up your subscriber and define its callback
    rospy.Subscriber(image_topic, Image, image_callback)

    # Publish moving commands
    pub = rospy.Publisher('follower/prius', Control, queue_size = 1)

    command = Control()
    command.shift_gears = Control.FORWARD
    pub.publish(command)

    rate = rospy.Rate(300)
    while not rospy.is_shutdown():
        command = Control()
        command.shift_gears = Control.NO_COMMAND

        if command_variables['center_value'] != None:
            new_error = command_variables['center_value'] - command_variables['current_center_value']
            proportional = kp * new_error

            error_difference = new_error - command_variables['previous_error']
            command_variables['previous_error'] = error_difference
            derivative = kd * error_difference

            command.throttle = 0.4
            command.steer = proportional + derivative

            rospy.loginfo(f"Lane Value: {command_variables['center_value']}")
            command.header.stamp = command_variables['time']

        # rospy.loginfo(f"v: {command.throttle}, w: {command.steer}")
        pub.publish(command)
        rate.sleep()

    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()