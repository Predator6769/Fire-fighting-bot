import RPi.GPIO as GPIO
import time
from geometry_msgs.msg import Twist
import rospy
 
#GPIO Mode (BOARD / BCM)
GPIO.setmode(GPIO.BCM)
 
#set GPIO Pins
GPIO_TRIGGER1 = 18
GPIO_ECHO1 = 24
GPIO_TRIGGER2 = 21
GPIO_ECHO2 = 22
GPIO_TRIGGER2 = 21
GPIO_ECHO2 = 22
GPIO_TRIGGER3 = 23
GPIO_ECHO3 = 25
  
#set GPIO direction (IN / OUT)
GPIO.setup(GPIO_TRIGGER1, GPIO.OUT)
GPIO.setup(GPIO_ECHO1, GPIO.IN)
GPIO.setup(GPIO_TRIGGER2, GPIO.OUT)
GPIO.setup(GPIO_ECHO2, GPIO.IN)
GPIO.setup(GPIO_TRIGGER3, GPIO.OUT)
GPIO.setup(GPIO_ECHO3, GPIO.IN)
 
def distance(gpio_t,gpio_e):
    # set Trigger to HIGH
    GPIO.output(gpio_t, True)
 
    # set Trigger after 0.01ms to LOW
    time.sleep(0.00001)
    GPIO.output(gpio_t, False)
 
    StartTime = time.time()
    StopTime = time.time()
 
    # save StartTime
    while GPIO.input(gpio_e) == 0:
        StartTime = time.time()
 
    # save time of arrival
    while GPIO.input(gpio_e) == 1:
        StopTime = time.time()
 
    # time difference between start and arrival
    TimeElapsed = StopTime - StartTime
    # multiply with the sonic speed (34300 cm/s)
    # and divide by 2, because there and back
    distance = (TimeElapsed * 34300) / 2
 
    return distance
 
if __name__ == '__main__':
    rospy.init_node('object_avoidance_node')
    pub = rospy.Publisher('/ultrasonic_cmd',Twist,queue_size=10)

    twist = Twist()
    while True:

        dist1 = distance(GPIO_TRIGGER1,GPIO_ECHO1)
        dist2 = distance(GPIO_TRIGGER2,GPIO_ECHO2)
        dist3 = distance(GPIO_TRIGGER3,GPIO_ECHO3)
        if dist2 <=10 and dist1<=dist3 :
            twist.angular.z = -1.0
            twist.linear.x = 0
        elif dist2 <=10 and dist1>=dist3:
            twist.angular.z = 1.0
            twist.linear.x = 0
        elif dist3<=10 or dist1<=10:
            twist.linear.x = 0.1
            twist.angular.z = 0
        else:    
            twist.linear.x = 0
            twist.angular.z = 0
        pub.publish(twist)
        time.sleep(0.11)
 
        # Reset by pressing CTRL + C