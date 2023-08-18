#!/usr/bin/python3
import rospy
from mpu6050 import mpu6050
from sensor_msgs.msg import Imu

sensor = mpu6050(0x68)
imu_sensor = Imu() 

if __name__ == '__main__':
  rospy.init_node('Imu_generator')
  pub = rospy.Publisher('/Imu_publisher',Imu,queue_size=10)
  

  while True:
    accel = sensor.get_accel_data()
    gyro_data = sensor.get_gyro_data()
    imu_sensor.header.stamp = rospy.time.now()   
    imu_sensor.header.frame_id = "base_link"
    imu_sensor.angular_velocity.z = gyro_data*0.0174533
    imu_sensor.linear_acceleration.x = accel * 9.81
    pub.publish(imu_sensor)