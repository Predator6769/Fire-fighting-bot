#!/usr/bin/python3
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


def velocity_callback(vel):
    odom = Odometry()
    current_time = rospy.Time.now()
    odom.header.stamp = current_time
    odom.header.frame_id = "odom"
    odom.child_frame_id = "base_link"
    odom.twist.twist = vel
    pub.publish(odom)

if __name__ == '__main__':
    rospy.init_node('odometry_generator')
    sub = rospy.Subscriber('/velocity_publisher',Twist,velocity_callback)
    pub = rospy.Publisher('/odom_publisher',Odometry,queue_size=10)
    rospy.spin()