#!/usr/bin/env python
# -*- coding: utf-8 -*- 
import cv2
from numpy.core.fromnumeric import shape
import os
import numpy as np
import rospy
from cv_bridge import CvBridge
from std_msgs.msg import Float64
from sensor_msgs.msg import Image
from carmaker_node.msg import *
import shutil

class sign_detect():
    def __init__(self):
        print("[start ] sign detect process")
        self.file_num = 0 
        path_here = os.path.dirname(os.path.abspath( __file__ ) )
        self.store_path = path_here + '/traffic_sign_storage'
        shutil.rmtree(self.store_path,ignore_errors=True)                
        os.mkdir(self.store_path)

    def main(self, msg):
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.cut_frame(cv_image)
        self.find_red_circle()
        self.read_sign_num()
        print("---------------------------")
    
    def cut_frame(self,cv_image):
        print("[start ] cut frame")
        self.copy_img = np.copy(cv_image[200:350,800:,:])
        self.rst_img = np.zeros_like(self.copy_img)
        self.cut_circle = np.zeros_like(self.copy_img)
        self.cut_circle[:,:,:] = [225,225,225]
        self.rst_img[:,:,:] = [225,225,225]
    
    def find_red_circle(self):
        print("[start ] find red circle")
        # find red
        b = self.copy_img[:, :, 0]
        g = self.copy_img[:, :, 1]
        r = self.copy_img[:, :, 2]
        green_frame = r > 180
        blue_frame = b > 100
        red_frame = g > 100
        green_mask = np.zeros_like(self.copy_img)
        blue_mask = np.zeros_like(self.copy_img)
        red_mask = np.zeros_like(self.copy_img)
        blue_mask[blue_frame] = 1
        red_mask[red_frame] = 1
        green_mask[green_frame] = 1
        unnecessary_mask = cv2.bitwise_or(1 - blue_mask, 1 - red_mask)
        new_frame = self.copy_img * green_mask * unnecessary_mask

        # find red circle's location
        rect_frame = cv2.cvtColor(new_frame, cv2.COLOR_BGR2GRAY)
        self.circles = cv2.HoughCircles(rect_frame, cv2.HOUGH_GRADIENT, 1, 1,
                                            param1=40, param2=10, minRadius=3, maxRadius=20)

    
    def read_sign_num(self):
        print("[start ] read_sign_num")
        
        if self.circles is not None:
            circles = np.uint16(np.around(self.circles))
            for i in circles[0, :]:
                self.cut_circle[i[1]-i[2]:i[1]+i[2],i[0]-i[2]:i[0]+i[2],:] = self.copy_img[i[1]-i[2]:i[1]+i[2],i[0]-i[2]:i[0]+i[2],:]
                threshold = (self.cut_circle[:,:,0]<150) & (self.cut_circle[:,:,1]<150) & (self.cut_circle[:,:,2]<150)
                self.rst_img[threshold] = [0,0,0]
       
            #path_here = os.path.dirname(os.path.abspath( __file__ ) )
            #store_path = '/traffic_sign_storage/' + str(self.file_num) + '.png'
            #save_path = path_here + store_path
            save_path = self.store_path + '/' + str(self.file_num) + '.png'
            cv2.imwrite(save_path, self.rst_img)
            self.file_num = self.file_num + 1

        else:
            pass
            #print("Can not detect a traffic sign")


if __name__ == "__main__":
    rospy.init_node("sign_detection", anonymous=True)
    signdetect = sign_detect()

    rospy.Subscriber("/vds_node_localhost_2218/image_raw",Image, signdetect.main, queue_size = 2 )
    rospy.spin()
