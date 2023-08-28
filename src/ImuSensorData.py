#!/usr/bin/python3
import rospy
#from mpu6050 import mpu6050
from sensor_msgs.msg import Imu

#sensor = mpu6050(0x68)
 

import smbus					#import SMBus module of I2C
from time import sleep          #import

imu_sensor = Imu()
#some MPU6050 Registers and their Address
PWR_MGMT_1   = 0x6B
SMPLRT_DIV   = 0x19
CONFIG       = 0x1A
GYRO_CONFIG  = 0x1B
INT_ENABLE   = 0x38
ACCEL_XOUT_H = 0x3B
ACCEL_YOUT_H = 0x3D
ACCEL_ZOUT_H = 0x3F
GYRO_XOUT_H  = 0x43
GYRO_YOUT_H  = 0x45
GYRO_ZOUT_H  = 0x47


def MPU_Init():
	#write to sample rate register
	bus.write_byte_data(Device_Address, SMPLRT_DIV, 7)
	
	#Write to power management register
	bus.write_byte_data(Device_Address, PWR_MGMT_1, 1)
	
	#Write to Configuration register
	bus.write_byte_data(Device_Address, CONFIG, 0)
	
	#Write to Gyro configuration register
	bus.write_byte_data(Device_Address, GYRO_CONFIG, 24)
	
	#Write to interrupt enable register
	bus.write_byte_data(Device_Address, INT_ENABLE, 1)

def read_raw_data(addr):
	#Accelero and Gyro value are 16-bit
  high = bus.read_byte_data(Device_Address, addr)
  low = bus.read_byte_data(Device_Address, addr+1)
    
        #concatenate higher and lower value
  value = ((high << 8) | low)
        
        #to get signed value from mpu6050
  if(value > 32768):
    value = value - 65536
  return value





if __name__ == '__main__':
  rospy.init_node('Imu_generator')
  pub = rospy.Publisher('/Imu_publisher',Imu,queue_size=10)
  bus = smbus.SMBus(1) 	# or bus = smbus.SMBus(0) for older version boards
  Device_Address = 0x68   # MPU6050 device address

  MPU_Init()
  

  while True:
    #accel = sensor.get_accel_data()
    #gyro_data = sensor.get_gyro_data()
    
    acc_x = read_raw_data(ACCEL_XOUT_H)
    gyro_z = read_raw_data(GYRO_ZOUT_H)
    Ax = acc_x/16384.0
    Gz = gyro_z/131.0
    #print("gyro data :",gyro_data,"accel: ",accel)
    imu_sensor.header.stamp = rospy.Time.now()   
    imu_sensor.header.frame_id = "base_link"
    #imu_sensor.angular_velocity.z = (gyro_data['z'] *0.0174533)
    #imu_sensor.linear_acceleration.x = (accel['x'] * 9.81) + 6
    imu_sensor.angular_velocity.z = (Gz *0.0174533)
    imu_sensor.linear_acceleration.x = (Ax * 9.81)
    imu_sensor.orientation_covariance[0] = -1
    pub.publish(imu_sensor)