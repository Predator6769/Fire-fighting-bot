#!/usr/bin/python3
#from ultralytics import YOLO
import cv2
import math
from std_msgs.msg import Float64MultiArray    
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
# Running real time from webcam
cap = cv2.VideoCapture('/dev/video0')
#model = YOLO('/home/pi/catkin_ws/src/ros_fire_fighting_rover/config/final_firedet.pt')


# Reading the classes
#classnames = ['fire']
rospy.init_node('cam_node')
pub = rospy.Publisher('/image_frame',Image,queue_size=10)
arr_msg = Image()
bridge = CvBridge()
    #print(frame)

while not rospy.is_shutdown():
    ret,frame = cap.read()
    cv_frame = bridge.cv2_to_imgmsg(frame,"bgr8")
    #print(frame)
    #frame = cv2.resize(frame,(640,480))
    #cv_frame = bridge.imgmsg_to_cv2(cv_frame,desired_encoding="passthrough")
    pub.publish(cv_frame) 
    #result = model(frame,stream=True)
    #cv2.imwrite('/home/pi/catkin_ws/src/ros_fire_fighting_rover/test.jpg',cv_frame)

    # Getting bbox,confidence and class names informations to work with
    #for info in result:
        #boxes = info.boxes
        #for box in boxes:
        #    confidence = box.conf[0]
        #    confidence = math.ceil(confidence * 100)
        #   Class = int(box.cls[0])
        #    print(Class)
        #    if confidence > 50:
        #        x1,y1,x2,y2 = box.xyxy[0]
        #        x1, y1, x2, y2 = int(x1),int(y1),int(x2),int(y2)
        #        print(x1,y1,x2,y2)
        #        cv2.rectangle(frame,(x1,y1),(x2,y2),(0,0,255),5)
        #        cv2.circle(frame, (int((x1+x2)/2),int((y1+y2)/2)), radius=0, color=(0, 255, 0), thickness=-1)
        #        cv2.line(frame,(int((x1+x2)/2),int((y1+y2)/2)),(320,240),color=(0, 255, 0), thickness=3)
                # cvzone.putTextRect(frame, f'{classnames[Class]} {confidence}%', [x1 + 8, y1 + 100],
                #                    scale=1.5,thickness=2)




    #cv2.imshow('frame',frame)
    #cv2.waitKey(1)