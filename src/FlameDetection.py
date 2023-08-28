#!/usr/bin/python3
import rospy
from geometry_msgs.msg import Twist
import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BCM)
flame_sensor_pin = 17
GPIO.setup(flame_sensor_pin,GPIO.IN)

if __name__ == '__main__':
    twist = Twist()
    rospy.init_node('flame_detection_sensor_node')
    pub = rospy.Publisher('/flame_cmd',Twist,queue_size=10)
    while True:
        if GPIO.input(flame_sensor_pin) == GPIO.HIGH:
            twist.linear.x = 0.1
            pub.publish(twist)
        else:
            twist.linear.x = 0
            pub.publish(twist)
            print("fire")
            #water pump code goes here   
        
