#!/home/j/.pyenv/versions/ros_py36/bin/python3
from pytesseract import *
import numpy as np
import rospy
import cv2
from carmaker_node.msg import *
import os
import glob

class pytesseractm():
    def __init__(self):
        self.traffic_sign_pub = rospy.Publisher('/traffic_sign', traffic_sign, queue_size=10)
        self.last_file_num = 0

    def main(self):
        print("[start ] pytesseract process")
        rospy.init_node('filter', anonymous=True)
        rate = rospy.Rate(8)
        while not rospy.is_shutdown():
            self.Pytesseract()
            rate.sleep()


    def Pytesseract(self):
        text = 'reset'
        text_list = []

        path_here = os.path.dirname(os.path.abspath( __file__ ) )
        store_path = '/traffic_sign_storage/'
        save_path = path_here + store_path
        imgs = glob.glob(os.path.join(save_path, "*.png"))

        last_img_num = len(imgs) - 1
        last_img_num_str = str(last_img_num)
        last_img_path = save_path + last_img_num_str + '.png'	
        
        if last_img_num >= 0:
                text = pytesseract.image_to_string(last_img_path, lang=None)
                text_list = list(text)
                self.filters(text_list,text)


    def filters(self,text_list,text):
        if '1' in text_list:
            if '0' in text_list:
                print(text)
                self.publish(10)
            elif '5' in text_list:    
                print(text)
                self.publish(15)
        elif '2' in text_list:
            if '0' in text_list:
                print(text)
                self.publish(20)
            elif '5' in text_list:    
                print(text)
                self.publish(25)
        elif '3' in text_list:
            if '0' in text_list:
                print(text)
                self.publish(30)
            elif '5' in text_list:    
                print(text)
                self.publish(35)
        elif '4' in text_list:
            if '0' in text_list:
                print(text)
                self.publish(40)
            elif '5' in text_list:    
                print(text)
                self.publish(45)
        elif '5' in text_list:
            if '0' in text_list:
                print(text)
                self.publish(50)
            elif '5' in text_list:    
                print(text)
                self.publish(55)
        else:
            pass
    

    def publish(self,text):
        print("[start ] publish ros topic")
        pubMsg = traffic_sign()
        pubMsg.limit_num = text
        self.traffic_sign_pub.publish(pubMsg)


if __name__ == "__main__":
    p = pytesseractm()
    try:
        p.main()
    except rospy.ROSInterruptException: pass
