#!/usr/bin/python3
import numpy as np
from imutils.object_detection import non_max_suppression
import cv2
from cv_bridge import CvBridge
import rospy
from sensor_msgs.msg import Image


HOGCV = cv2.HOGDescriptor()
HOGCV.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())


def Detector(frame):
    
    bridge = CvBridge()
    cv_frame = bridge.imgmsg_to_cv2(frame,desired_encoding="passthrough")
    resized_frame = cv2.resize(cv_frame,(640,480))
    # USing Sliding window concept
    rects, weights = HOGCV.detectMultiScale(resized_frame, winStride=(4, 4), padding=(8, 8), scale=1.03)
    rects = np.array([[x, y, x + w, y + h] for (x, y, w, h) in rects])
    pick = non_max_suppression(rects, probs=None, overlapThresh=0.65)
    c = 0
    for x, y, w, h in pick:
        #cv2.rectangle(frame, (x, y), (w, h), (139, 34, 104), 2)
        #cv2.rectangle(frame, (x, y - 20), (w,y), (139, 34, 104), -1)
        #cv2.putText(frame, f'P{c}', (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        c += 1
        
    if c!=0:
        print('There is some guy')
        c=0
    #cv2.putText(frame, f'Total Persons : {c - 1}', (20, 450), cv2.FONT_HERSHEY_DUPLEX, 0.8, (255, 255,255), 2)
    #cv2.imshow('output', frame)
    #return frame

if __name__=='__main__':

    rospy.init_node('cmd_vel_cam_center')
    #pub = rospy.Publisher('/cmd_vel_cam',Twist,queue_size=10)
    sub = rospy.Subscriber('/usb_cam/image_raw',Image,Detector)
    rospy.spin()
