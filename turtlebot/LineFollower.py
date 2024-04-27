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
RGB_LOW = (0,0,0)
RGB_HIGH = (255,180,150)

FPS = 15.0
DISPLAY_FPS = 5.0
DISPLAY_IMG = True
IMG_W = 250
IMG_H = 250
K = ANGULAR_VEL * 2



class WebcamControl():
    def __init__(self):
        self.mutex = Lock()
        rclpy.init()

        
        self.node = rclpy.create_node('line_follower_node')
        #recieves image from camera
        self.img_sub = self.node.create_subscription(Image, '/oakd/rgb/preview/image_raw', self.process_image, 5)
        self.img_line = self.node.create_publisher(Image, '/imgage/line', 3)
        
        #publishes to create3 to operate robot
        self.cmd_vel_pub = self.node.create_publisher(Twist, 'cmd_vel', qos_profile=qos_profile_sensor_data)
        
        self.bridge = CvBridge()
        
        try:
            print("Node is running")
            rclpy.spin(self.node)
            
        except KeyboardInterrupt:
            print("Rospy Spin Shut down")

    def process_image(self, input_image):
        print("recieved image")

        #command = Twist()
        #command.linear.x = 0.5
        #self.cmdel_pub.publish(command)
        

        try:
            img = self.bridge.imgmsg_to_cv2(input_image, "bgr8")
            print(img.shape)
        except CvBridgeError as e:
            print(e)


        if img is None:
            print ('frame dropped, skipping tracking')
        else:
            cx = self.get_line_pos(img)
            print(cx)
            if cx is not None:
                error = IMG_W/2 - cx
                error_norm = error / (IMG_W / 2.0)
                
                command = Twist()
                command.linear.x = LINEAR_VEL * (1 - np.abs(error_norm))
                command.angular.z = K * error_norm
                self.cmd_vel_pub.publish(command)
            else:
                command = Twist()
                command.linear.x = 0.0
                command.angular.z = ANGULAR_VEL
                self.cmd_vel_pub.publish(command)
            
        

    def get_line_pos(self, img):
        H, W, _ = img.shape
        #crops the image (calculate where in the frame the line will be)

        img = img[200:250, :]
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        mask = cv2.inRange(img, RGB_LOW, RGB_HIGH)
        line_img = cv2.bitwise_and(gray, mask)
        
        imageOut = self.bridge.cv2_to_imgmsg(line_img)
        self.img_line.publish(imageOut)
        

        contours, _ = cv2.findContours(np.uint8(line_img), cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
        
        
        if len(contours) > 0:
            line = max(contours, key=cv2.contourArea)
            print(cv2.contourArea(line))
            if cv2.contourArea(line) > 30:
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
        
