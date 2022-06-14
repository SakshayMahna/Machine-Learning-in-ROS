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

# Command Variables
command_variables = {
    'current_lane_value': 0,
    'lane_value': None,
    'previous_error': 0,
    'lane': 0,
    'straight_counter': 0,
    'turn': 0,
    'turn_counter': 0,
    'turns_completed': 0,
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
        upper_range = np.array([20, 100, 100])
        lower_range = np.array([30, 255, 255])

        img = cv2_img
        h, w, _ = img.shape

        if command_variables['lane'] == 0:
            img = img[h-350:h-250, 250:w-200]
        else:
            img = img[h-350:h-250, 125:w-400]

        img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(img_hsv, upper_range, lower_range)
        output_img = cv2.bitwise_and(img, img, mask=mask)

        # mask_image = np.zeros_like(output_img)
        # mask_image[:,:,0] = mask
        # mask_image[:,:,1] = mask
        # mask_image[:,:,2] = mask

        # show_image = np.concatenate((img, output_img), axis=1)
        # show_image = np.concatenate((show_image, mask_image), axis = 1)
        # cv2.namedWindow("Detected lanes", cv2.WINDOW_NORMAL) 
        # cv2.imshow("Detected lanes", show_image)
        # cv2.waitKey(0)

        M = cv2.moments(mask)

        try:
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])

            command_variables['current_lane_value'] = cX
            if command_variables['lane_value'] == None:
                command_variables['lane_value'] = cX

            # rospy.loginfo(f"Current Lane Value: {command_variables['current_lane_value']}")
            # rospy.loginfo(f"Lane Value: {command_variables['lane_value']}")
        except ZeroDivisionError:
            pass
        

def main():
    global command_variables

    kp, kd = 0.011, 0.011

    rospy.init_node('lane_follower')
    # Define your image topic
    image_topic = "followee/prius/front_camera/image_raw"
    # Set up your subscriber and define its callback
    rospy.Subscriber(image_topic, Image, image_callback)

    # Publish moving commands
    pub = rospy.Publisher('followee/prius', Control, queue_size = 1)

    command = Control()
    command.shift_gears = Control.FORWARD
    pub.publish(command)

    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        command = Control()
        command.shift_gears = Control.NO_COMMAND

        if command_variables['lane_value'] != None:
            new_error = command_variables['lane_value'] - command_variables['current_lane_value']
            proportional = kp * new_error

            error_difference = new_error - command_variables['previous_error']
            command_variables['previous_error'] = error_difference
            derivative = kd * error_difference
            
            if command_variables['turn'] == 0:
                command.throttle = 0.5
                command.steer = proportional + derivative
                command_variables['straight_counter'] += 1

                if command_variables['straight_counter'] == 350:
                    command_variables['straight_counter'] = 0
                    
                    if command_variables['lane'] == 0:
                        command_variables['turn'] = 1
                        command_variables['lane'] = 1
                    else:
                        command_variables['turn'] = -1
                        command_variables['lane'] = 0
            
            else:
                command.steer = 1.0 * command_variables['turn']
                command_variables['turn_counter'] += 1

                if command_variables['turn_counter'] == 60:
                    command_variables['turn_counter'] = 0
                    command_variables['turn'] = 0
                    command_variables['lane_value'] = None

            rospy.loginfo(f"Lane Value: {command_variables['lane_value']}")
            command.header.stamp = command_variables['time']

        # rospy.loginfo(f"v: {command.throttle}, w: {command.steer}")
        pub.publish(command)
        rate.sleep()

    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()