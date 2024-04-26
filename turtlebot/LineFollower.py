
#!/usr/bin/env python2

import cv2
import numpy as np
import rospy
import time
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image

LINEAR_VEL = 1.5
ANGULAR_VEL = 1.5
RGB_LOW = (150,90,0)
RGB_HIGH = (255,180,100)

FPS = 15.0
DISPLAY_FPS = 5.0
DISPLAY_IMG = True
IMG_W = 640
IMG_H = 480
K = ANGULAR_VEL * 2

def followLine():
    bridge = CvBridge()
    img_pub = rospy.Publisher('usb_cam/image_raw', Image, queue_size=1)
    pub = rospy.Publisher('cmd_vel', Twist, queue_size = 1)

    rospy.init_node('webcam_control', anonymous=True)

    cam = cv2.VideoCapture(0)

    rate = rospy.Rate(FPS)
    command = Twist()
    command.linear.y = 0
    command.linear.z = 0
    command.angular.x = 0
    command.angular.y = 0

    fpsT = 0
    displayFpsT = 0

    while not rospy.is_shutdown():
        cx, img = getLinePos(cam, fpsT)
        if img is None:
            continue
        fpsT = time.time()

        if cx != -1: #Drive straight behavior
            error = IMG_W/2 - cx
            errorNorm = error / (IMG_W / 2.0)

            command.linear.x = LINEAR_VEL * (1-np.abs(errorNorm))
            command.angular.z = K * errorNorm
        else: #Turn around behavior
            command.linear.x = 0
            command.angular.z = ANGULAR_VEL
        pub.publish(command)
        if DISPLAY_IMG and time.time() - displayFpsT >= 1/DISPLAY_FPS:
            try:
                imgMsg = bridge.cv2_to_imgmsg(img, encoding = 'bgr8')
            except CvBridgeError as e:
                print(e)
                break
            img_pub.publish(imgMsg)
            displayFpsT = time.time()

def getLinePos(cam, fpsT):
    ret_val, img = cam.read()
    if time.time() - fpsT < 1/FPS:
        return None, None

    H, W, _ = img.shape
    img = img[(H-IMG_H)/2:(H+IMG_H)/2, (W-IMG_W)/2:(W+IMG_W)/2] #crop

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    mask = cv2.inRange(img, RGB_LOW, RGB_HIGH)
    line_img = gray & mask

    contours, _ = cv2.findContours(np.uint8(line_img), cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)

    if len(contours) > 0:
        line = max(contours, key=cv2.contourArea)
        if cv2.contourArea(line) > 3000:
            moments = cv2.moments(line)
            cx = int(moments['m10']/moments['m00'])
            cy = int(moments['m01']/moments['m00'])

            if DISPLAY_IMG:
                cv2.drawContours(img, contours, -1, (180,255,180), 3)
                cv2.drawContours(img, line, -1, (0,255,0), 3)
                cv2.circle(img, (cx,cy), 4, (255,0,0),-1)
        else:
            cx = -1
    else:
        cx = -1

    return cx, img

if __name__ == '__main__':
    try:
        followLine()
    except rospy.ROSInterruptException:
        pass
webcam_control_turtlebot.py
Displaying webcam_control_turtlebot.py.
