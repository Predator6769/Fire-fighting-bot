#!/usr/bin/python3
import rospy
from geometry_msgs.msg import Twist

flame_c = Twist()
cam_c = Twist()
ultra_cmd = Twist()
def flame_cmd_cb(msg):
    global flame_c
    flame_c = msg
   
def cam_cmd_cb(msg):
    global cam_c,ultra_cmd,flame_c
    cam_c = msg
    if ultra_cmd.linear.x != 0 or ultra_cmd.angular.z != 0:
        #final_twist = ultra_cmd
        pub.publish(ultra_cmd) 
    elif cam_c.angular.z == 0:
        #final_twist = flame_c
        pub.publish(flame_c)
    else:
        #final_twist = cam_c
        pub.publish(cam_c)

def ultrasonic_cmd_cb(msg):
    global ultra_cmd
    ultra_cmd = msg

if __name__ == '__main__':
    rospy.init_node('final_cmd')
    pub = rospy.Publisher('/final_cmd',Twist,queue_size=10)
    sub = rospy.Subscriber('/flame_cmd',Twist,flame_cmd_cb)
    sub1 = rospy.Subscriber('/cmd_vel_cam',Twist,cam_cmd_cb)
    sub2 = rospy.Subscriber('/ultrasonic_cmd',Twist,ultrasonic_cmd_cb)    
        
    rospy.spin()