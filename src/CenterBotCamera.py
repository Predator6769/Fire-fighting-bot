#!/usr/bin/python3
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
from roboflow import Roboflow

rf = Roboflow(api_key="fZgl5tqZ3XY9ySlPGpoM")
project = rf.workspace().project("fire-detection-data-pre")
model = project.version(4).model
prev_twist = 0

def translate(sensor_val, in_from, in_to, out_from, out_to):
    out_range = out_to - out_from
    in_range = in_to - in_from
    in_val = sensor_val - in_from
    val=(float(in_val)/in_range)*out_range
    out_val = out_from+val
    return out_val

def send_velocityCommands(frame):
    global prev_twist
    twist = Twist()
    bridge = CvBridge()
    cv_frame = bridge.imgmsg_to_cv2(frame,desired_encoding="passthrough")
    resized_frame = cv2.resize(cv_frame,(640,480))
    result = model.predict(resized_frame, confidence=40, overlap=30).json()
    fire = result["predictions"]
    if len(fire) == 0:
        twist.angular.z = 1.0
    else:
          
        fire_center_x = fire[0]["x"]
        fire_width = fire[0]["width"]
        fire_height = fire[0]["height"]
        if fire_width ==0 or fire_height ==0:
            twist.angular.z = 0.07
        else:    
            print(fire[0]["width"],fire[0]["height"])
            center_x = 320
            diff_x = fire_center_x - center_x
            if abs(diff_x) > 30 and diff_x > 0:
                twist.angular.z = translate(diff_x,0,320,0,10)
            elif abs(diff_x) > 30 and diff_x <0:
                twist.angular.z = translate(diff_x,0,320,0,10)
            else:
                twist.angular.z = 0        
    
    

    pub.publish(twist)
    prev_twist = twist

if __name__=='__main__':
    rospy.init_node('cmd_vel_cam_center')
    pub = rospy.Publisher('/cmd_vel_cam',Twist,queue_size=10)
    sub = rospy.Subscriber('/usb_cam/image_raw',Image,send_velocityCommands)
    rospy.spin()