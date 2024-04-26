#!/usr/bin/env python3

import cv2
import numpy as np
import rclpy
from threading import Lock
from rclpy.qos import qos_profile_sensor_data
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


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



class WebcamControl():
    def __init__(self):
        print("hello")
        rclpy.init()
        self.mutex = Lock()
        self.node = rclpy.create_node('line_follower_node')
        print("hello1")
        self.img_pub = self.node.create_publisher(Image, 'usb_cam/image_raw', qos_profile=qos_profile_sensor_data)
        self.cmd_vel_pub = self.node.create_publisher(Twist, 'cmd_vel', qos_profile=qos_profile_sensor_data)
        print("hello2")
        self.bridge = CvBridge()

        self.cam = cv2.VideoCapture(0)

        #self.timer = self.node.create_timer(1.0 / FPS, self.process_image)
        try:
            print("Node is running")
            rclpy.spin(self.node)
        except KeyboardInterrupt:
            print("Rospy Spin Shut down")

    def process_image(self):
        ret_val, img = self.cam.read()

        if ret_val:
            cx, img = self.get_line_pos(img)

            if cx is not None:
                error = IMG_W/2 - cx
                error_norm = error / (IMG_W / 2.0)

                command = Twist()
                command.linear.x = LINEAR_VEL * (1 - np.abs(error_norm))
                command.angular.z = K * error_norm
            else:
                command = Twist()
                command.linear.x = 0
                command.angular.z = ANGULAR_VEL

            self.cmd_vel_pub.publish(command)

            if DISPLAY_IMG:
                try:
                    img_msg = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
                    self.img_pub.publish(img_msg)
                except CvBridgeError as e:
                    print(e)

    def get_line_pos(self, img):
        H, W, _ = img.shape
        img = img[(H-IMG_H)//2:(H+IMG_H)//2, (W-IMG_W)//2:(W+IMG_W)//2] #crop

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        mask = cv2.inRange(img, RGB_LOW, RGB_HIGH)
        line_img = cv2.bitwise_and(gray, mask)

        contours, _ = cv2.findContours(np.uint8(line_img), cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)

        if len(contours) > 0:
            line = max(contours, key=cv2.contourArea)
            if cv2.contourArea(line) > 3000:
                moments = cv2.moments(line)
                cx = int(moments['m10']/moments['m00'])
                cy = int(moments['m01']/moments['m00'])

                # if DISPLAY_IMG:
                #     cv2.drawContours(img, contours, -1, (180,255,180), 3)
                #     cv2.drawContours(img, [line], -1, (0,255,0), 3)
                #     cv2.circle(img, (cx,cy))

if __name__ == "__main__":
    try:
        node = WebcamControl()
    except:
        pass