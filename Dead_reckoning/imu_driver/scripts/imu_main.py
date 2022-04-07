#!/usr/bin/env python


import serial
import rospy
import datetime
import utm
from imu_driver.msg import imu
from sensor_msgs.msg import Imu
from sensor_msgs.msg import MagneticField
import numpy as np
from tf.transformations import quaternion_from_euler

ser = serial.Serial('/dev/ttyUSB0')
ser.baudrate = 115200
line_splitter = []

global latitude_deg
msg = Imu()
msg1 = MagneticField()
msg2 = imu()

def talker():
    pub = rospy.Publisher('imu_message', Imu, queue_size=10) #declare a topic /imu_message to publish the values from IMU to sensor_msgs/msg/Imu.msg
    pub1 = rospy.Publisher('mag_message', MagneticField, queue_size=10)
    pub2 = rospy.Publisher('imu_raw',imu, queue_size=10)
    rospy.init_node('imu_talker', anonymous=True)
    r = rospy.Rate(40)
       

    while not rospy.is_shutdown():
            
        input_serial = str(ser.readline())
        line_splitter = input_serial.split("b'") #every line gets this b', hence used it as a delimiter
        comma_splitter = line_splitter[1].split(",") #to split each line with commas to get latitude, longitude easily
    
        # print(line_splitter)

        yaw = float(comma_splitter[1])
        pitch = float(comma_splitter[2])
        roll = float(comma_splitter[3])
        magx = float(comma_splitter[4])
        magy = float(comma_splitter[5])
        magz = float(comma_splitter[6])
        accx = float(comma_splitter[7])
        accy = float(comma_splitter[8])
        accz = float(comma_splitter[9])
        gyrox = comma_splitter[9]
        gyrox = float(gyrox[:10])
        gyroy = comma_splitter[11]
        gyroy = float(gyroy[:10])
        gyroz = comma_splitter[12]
        gyroz = float(gyroz[:10])
        # r = [yaw, pitch, roll]
        quarternion = quaternion_from_euler(roll, pitch, yaw)
        angular_velocity = [gyrox, gyroy, gyroz]
        acceleration = [accx, accy ,accz]
        magnetic = [magx, magy, magz]
        # print(quarternion)


        msg2.yaw = float(yaw)
        msg2.pitch = float(pitch)
        msg2.roll = float(roll)
        msg2.magx = float(magx)
        msg2.magy = float(magy)
        msg2.magz = float(magz)
        msg2.accx = float(accx)
        msg2.accy = float(accy)
        msg2.accz = float(accz)
        msg2.gyrox = float(gyrox)
        msg2.gyroy = float(gyroy)
        msg2.gyroz = float(gyroz)

        msg.orientation.x = quarternion[0]
        msg.orientation.y = quarternion[1]
        msg.orientation.z = quarternion[2]
        msg.orientation.w = quarternion[3]
        msg.angular_velocity.x = gyrox
        msg.angular_velocity.y = gyroy
        msg.angular_velocity.z = gyroz
        msg.linear_acceleration.x = accx
        msg.linear_acceleration.y = accy
        msg.linear_acceleration.z = accz
        msg1.magnetic_field.x = magx
        msg1.magnetic_field.y = magy
        msg1.magnetic_field.z = magz

        rospy.loginfo(msg)
        pub.publish(msg)
        rospy.loginfo(msg1)
        pub1.publish(msg1)
        rospy.loginfo(msg2)
        pub2.publish(msg2)

        # r.sleep()


if __name__ == '__main__':

    talker()

