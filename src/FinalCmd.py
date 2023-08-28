#!/usr/bin/python3
import rospy
from geometry_msgs.msg import Twist

flame_c = Twist()
cam_c = Twist()
ultra_cmd = Twist()
mannual_cmd = Twist()
def flame_cmd_cb(msg):
    global flame_c
    flame_c = msg
   
def cam_cmd_cb(msg):
    global cam_c
    cam_c = msg

def mannual_cmd_cb(msg):
    global mannual_cmd
    mannual_cmd = msg

def ultrasonic_cmd_cb(msg):
    global ultra_cmd,cam_c,flame_c,mannual_cmd
    ultra_cmd = msg

    if mannual_cmd.linear.x!= 0 or mannual_cmd.angular.z != 0:
        pub.publish(mannual_cmd)
    elif ultra_cmd.linear.x != 0 or ultra_cmd.angular.z != 0:
        #final_twist = ultra_cmd
        pub.publish(ultra_cmd) 
    elif cam_c.angular.z == 0:
        #final_twist = flame_c
        pub.publish(flame_c)
    else:
        #final_twist = cam_c
        pub.publish(cam_c)

if __name__ == '__main__':
    rospy.init_node('final_cmd')
    pub = rospy.Publisher('/final_cmd',Twist,queue_size=10)
    sub = rospy.Subscriber('/flame_cmd',Twist,flame_cmd_cb)
    sub1 = rospy.Subscriber('/cmd_vel_cam',Twist,cam_cmd_cb)
    sub3 = rospy.Subscriber('/mannual_cmd_vel',Twist,mannual_cmd_cb)
    sub2 = rospy.Subscriber('/ultrasonic_cmd',Twist,ultrasonic_cmd_cb)    
        
    rospy.spin()