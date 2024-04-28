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
import time


LINEAR_VEL = 1.5
ANGULAR_VEL = 0.4
RGB_LOW = (0,0,0)
RGB_HIGH = (255,180,150)

FPS = 15.0
DISPLAY_FPS = 5.0
DISPLAY_IMG = True
IMG_W = 250
IMG_H = 250


IMG_W_NARROW = 40
IMG_H_NARROW = 50

IMG_W_TURN = 250
IMG_H_TURN = 30

K = ANGULAR_VEL * 2







class WebcamControl():
    def __init__(self):
        self.mutex = Lock()
        rclpy.init()
       
        self.TURN_FOUND = False
        self.LEFT_TURN = True
        self.node = rclpy.create_node('line_follower_node')
        #recieves image from camera
        self.img_sub = self.node.create_subscription(Image, '/oakd/rgb/preview/image_raw', self.process_image, 5)
        self.img_line = self.node.create_publisher(Image, '/image/wide', 3)
        self.states = ["follow line", "go to corner", "rotate"]
        self.state = self.states[1]
        #publishes to create3 to operate robot
        self.cmd_vel_pub = self.node.create_publisher(Twist, 'cmd_vel', qos_profile=qos_profile_sensor_data)
        
        self.bridge = CvBridge()
        
        try:
            print("Node is running")
            rclpy.spin(self.node)
            
        except KeyboardInterrupt:
            print("Rospy Spin Shut down")
    def move_to_edge(self):
        for _ in range(3):
            command = Twist()
            command.linear.x = LINEAR_VEL
            command.angular.z = 0.0
            self.cmd_vel_pub.publish(command)
            time.sleep(0.2)
    def process_image(self, input_image):
        print("recieved image")

    
        try:
            img = self.bridge.imgmsg_to_cv2(input_image, "bgr8")
            print("image shape", img.shape)
        except CvBridgeError as e:
            print(e)
        

        if img is None:
            print ('frame dropped, skipping tracking')
        else:
            
            match self.state:
                case "follow line":
                    cx = self.get_line_pos(img)
                    print("line position (pixel)", cx)
                    if cx is not None:
                        #delta pixels of line from center
                        error = IMG_W/2 - cx
                        error_norm = error / (IMG_W / 2.0)
                        
                        command = Twist()
                        command.linear.x = LINEAR_VEL * (1 - np.abs(error_norm))
                        command.angular.z = K * error_norm
                        #self.cmd_vel_pub.publish(command)
                case "go to corner":
                    self.move_to_edge()
                    self.states = self.states[1]
                case "rotate":
                    command = Twist()
                    command.linear.x = 0.0
                    #if line is on the left rotate at angular velocity 
                    if self.LEFT_TURN:
                        command.angular.z = ANGULAR_VEL
                    else:
                        command.angular.z = -ANGULAR_VEL

    def get_line_pos(self, img):
        H, W, _ = img.shape
        #crops the image (calculate where in the frame the line will be)

        #determines the center of the line
        box1 = img[IMG_H-8:IMG_H, ]

        gray = cv2.cvtColor(box1, cv2.COLOR_BGR2GRAY)
        mask = cv2.inRange(box1, RGB_LOW, RGB_HIGH)
        line_img = cv2.bitwise_and(gray, mask)
        imageOut = self.bridge.cv2_to_imgmsg(line_img)
        self.img_line.publish(imageOut)

    
        contours, _ = cv2.findContours(np.uint8(line_img), cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
        
        
        if len(contours) > 0:
            line = max(contours, key=cv2.contourArea)
            
            print("contours Area", cv2.contourArea(line))
            if cv2.contourArea(line) > 20:
                moments = cv2.moments(line)
                cx = int(moments['m10']/moments['m00'])
                #cy = int(moments['m01']/moments['m00'])
                return cx
        else: 
            return 0  
        
                
                
def main():
    node = WebcamControl()

if __name__ == "__main__":
    main()
        
