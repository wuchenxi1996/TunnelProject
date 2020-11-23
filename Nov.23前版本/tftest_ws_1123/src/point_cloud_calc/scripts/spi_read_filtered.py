#!/usr/bin/env python
import rospy
import spidev
from sensor_msgs.msg import Imu
from pibot_msgs.msg import RawImu
import os 
import sys

XGYRO = 0x04
YGYRO = 0x06
ZGYRO = 0x08
XACCL = 0x0A
YACCL = 0x0C
ZACCL = 0x0E
SMPL_PRD = 0x3600


calibration_samples = 8
taken_samples = 0
acce = [0.0,0.0,0.0]
angu = [0.0,0.0,0.0]

def adis_getdata(addr):

    spi.writebytes([addr])
    spi.writebytes([0x00])
    #delay_10us
    
    receive = spi.readbytes(2)
    return receive
def spi_readID():
	Temp = 0
	spi.writebytes([0x56])
	spi.writebytes([0x00])
	#delay 10us
	#spi.writebytes([0x00])
	#spi.writebytes([0x00])
	ID = spi.readbytes(2)
	return ID

def adis_test():
   # spi.writebytes()
    PortID = spi_readID();
    if(PortID != 0):
        print("Communications established.")
        print(PortID)
    else:
        print("Communications failed, closing spi")

def data_convert(gx, gy, gz, ax, ay, az):
    g_x = get_trueform(gx) /100.0 * 3.1415926 / 180
    g_y = get_trueform(gy) /100.0 * 3.1415926 / 180
    g_z = get_trueform(gz) /100.0 * 3.1415926 / 180
    a_x = get_trueform(ax) / 4000.0 * 9.81 
    a_y = get_trueform(ay) / 4000.0 * 9.81
    a_z = get_trueform(az) /4000.0 * 9.81
    return g_x, g_y, g_z, a_x, a_y, a_z 

def get_trueform(num):
	if ((num[0]*256+num[1]) & 0x8000 == 0):
		return num[0] * 256 + num[1]
	else:
		result = (num[0]*256+num[1])^0xffff
		result = result + 1
		result = -result
		return result

def filter():
	l = 1000
	xg_sum = [0] * l
	yg_sum = [0] * l
	zg_sum = [0] * l
	xa_sum = [0] * l
	ya_sum = [0] * l
	za_sum = [0] * l
	xg_sum[0] = 0.0
	yg_sum[0] = 0.0
	zg_sum[0] = 0.0
	xa_sum[0] = 0.0
	ya_sum[0] = 0.0
	za_sum[0] = 0.0
	
	for i in range(1000):
		Xg = adis_getdata(XGYRO)
		Yg = adis_getdata(YGYRO)
		Zg = adis_getdata(ZGYRO)
		Xa = adis_getdata(XACCL)
		Ya = adis_getdata(YACCL)
		Za = adis_getdata(ZACCL)
		if (i >= 1):
			xg_sum[i] = xg_sum[i-1] + get_trueform(Xg) /100.0 * 3.1415926 / 180
        	yg_sum[i] = yg_sum[i-1] + get_trueform(Yg) /100.0 * 3.1415926 / 180
        	zg_sum[i] = zg_sum[i-1] + get_trueform(Zg) /100.0 * 3.1415926 / 180
        	xa_sum[i] = xa_sum[i-1] + get_trueform(Xa) /4000.0 * 9.81
        	ya_sum[i] = ya_sum[i-1] + get_trueform(Ya) /4000.0 * 9.81
        	za_sum[i] = za_sum[i-1] + get_trueform(Za) /4000.0 * 9.81
	return xg_sum[999]/1000, yg_sum[999]/1000, zg_sum[999]/1000, xa_sum[999]/1000, ya_sum[999]/1000, za_sum[999]/1000

def median_filter():
	global taken_samples
	if(taken_samples < calibration_samples):
		Xg = adis_getdata(XGYRO)
		Yg = adis_getdata(YGYRO)
		Zg = adis_getdata(ZGYRO)
		Xa = adis_getdata(XACCL)
		Ya = adis_getdata(YACCL)
		Za = adis_getdata(ZACCL)
		[g_x, g_y, g_z, a_x, a_y, a_z] = data_convert(Xg, Yg, Zg , Xa, Ya, Za)
		angu[0] = angu[0] + g_x - xg_bias
		angu[1] = angu[1] + g_y - yg_bias
		angu[2] = angu[2] + g_z - zg_bias
		acce[0] = acce[0] + a_x - xa_bias
		acce[1] = acce[1] + a_y - ya_bias
		acce[2] = acce[2] + a_z - za_bias
		taken_samples = taken_samples + 1
	else:
		acce[0] = acce[0]/calibration_samples
		acce[1] = acce[1]/calibration_samples
		acce[2] = acce[2]/calibration_samples 
		angu[0] = angu[0]/calibration_samples
		angu[1] = angu[1]/calibration_samples
		angu[2] = angu[2]/calibration_samples
		taken_samples = 0
		imu_msg.linear_acceleration.x = acce[0] 
		imu_msg.linear_acceleration.y = acce[1]
		imu_msg.linear_acceleration.z = acce[2] + 9.81
		imu_msg.angular_velocity.x    = angu[0]
		imu_msg.angular_velocity.y    = angu[1]
		imu_msg.angular_velocity.z    = angu[2]
		adis_pub = rospy.Publisher('imu_adis', Imu, queue_size = 1)
		adis_pub.publish(imu_msg)
		print(imu_msg.linear_acceleration.x, imu_msg.linear_acceleration.y, imu_msg.linear_acceleration.z)

if __name__ == '__main__':
	
  	spi = spidev.SpiDev()
	bus = 0
	device = 1
	spi.close()
	if os.geteuid():
		args = [sys.executable] + sys.argv
		os.execlp('su','su','-c',' '.join(args))
	spi.open(bus,device)
# Settings 
	spi.max_speed_hz = 1000000 #Sampling rate is 1MHz
	spi.mode = 0b11
	print("SPI Initialized.")    
#Test if ADIS PortID can be obtained
	adis_test()
#Start Data Transfer
	
	rospy.init_node('ADIS_Read_Node', anonymous=True)
	rate = rospy.Rate(1000)
	imu_msg = Imu()
	seq = 0
    
	[xg_bias, yg_bias, zg_bias, xa_bias, ya_bias, za_bias] = filter()
	print("Calibration processing, wait a sec pls.")
	while not rospy.is_shutdown():
		imu_msg.orientation.x = 0
		imu_msg.orientation.y = 0
		imu_msg.orientation.z = 0
		imu_msg.orientation.w = 1
		imu_msg.header.stamp = rospy.Time.now()
		imu_msg.header.frame_id = 'imu_link'
		imu_msg.header.seq = seq
		seq = seq + 1
		median_filter()
		#adis_pub = rospy.Publisher('imu_adis', Imu, queue_size = 1)
		#adis_pub.publish(imu_msg)
		#print(imu_msg.linear_acceleration.x, imu_msg.linear_acceleration.y, imu_msg.linear_acceleration.z)
		rate.sleep()
