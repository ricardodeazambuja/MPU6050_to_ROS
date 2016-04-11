#!/usr/bin/python

'''
Interface for the MPU6050 using a Raspberry PI





Based on:
http://blog.bitify.co.uk/2013/11/reading-data-from-mpu-6050-on-raspberry.html


First it is necessary to activate the I2C:
sudo raspi-config
=>Advanced Options
==>I2C
===>YES (enable)

To connect the pins, have a look here:
https://ms-iot.github.io/content/en-US/win10/samples/PinMappingsRPi2.htm


Then it's necessary to install the i2c-tools:
sudo apt-get install i2c-tools

List the i2c adapters available:
i2cdetect -l
You are going to see something like this:
>>i2c-1	i2c       	3f804000.i2c                    	I2C adapter
The device can be reached by "/dev/i2c-1".
And i2c_bus=1 in this situation.

Connect the MPU6050 and verify the i2c address
(where 1 is the number after 'i2c-' the first line generated):
i2cdetect -y 1
You are going to see something like this:
     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
00:          -- -- -- -- -- -- -- -- -- -- -- -- --
10: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
20: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
30: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
40: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
50: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
60: -- -- -- -- -- -- -- -- 68 -- -- -- -- -- -- --
70: -- -- -- -- -- -- -- --
And i2c_address=68 in this situation.

For debugging, it's possible to access individual registers using
the commands i2cset and i2cget.
https://www.olimex.com/Products/Modules/Sensors/MOD-MPU6050/resources/RM-MPU-60xxA_rev_4.pdf
https://learn.sparkfun.com/tutorials/i2c

You will also need to install this:
sudo apt-get install python-smbus

The full list of things you are supposed to install:
sudo apt-get install i2c-tools libi2c-dev python-dev python-smbus

To query the functionalities of an  I2C  bus:
i2cdetect -F 1

'''

import smbus
import math
import time
import os

import websocket
import json

import sys

import argparse


class mpu6050_PI(object):

    def __init__(self, url, FS_SEL=0, AFS_SEL=3, i2c_bus=1, i2c_address=68, delay=0.1, debug=False):
        '''
        url => the ip address to find Rosbridge
        i2c_bus => run the command: i2cdetect -l
        i2c_address => run the command: i2cdetect -y 1
        (1 is the i2c_bus returned later)

        GYROSCOPE SENSITIVITY (datasheet, page 12)
        Full-Scale Range
        FS_SEL=0 => +/-250degree/s
        FS_SEL=1 => +/-500degree/s
        FS_SEL=2 => +/-1000degree/s
        FS_SEL=3 => +/-2000degree/s

        ACCELEROMETER SENSITIVITY (datasheet, page 13)
        Full-Scale Range
        AFS_SEL=0 => +/-2g
        AFS_SEL=1 => +/-4g
        AFS_SEL=2 => +/-8g
        AFS_SEL=3 => +/-16g
        '''

        # URL pointing where rosbridge_server is running
        self.rosbridge_url = "ws://" + url + ":9090"

        self.debug = debug
        self.delay = delay

        self.bus = smbus.SMBus(i2c_bus)
        self.address = int('0x'+str(i2c_address),0)

        # Power management registers
        power_mgmt_1 = 0x6b # MPU-6000-Register-Map1.pdf, page 40.
        power_mgmt_2 = 0x6c # MPU-6000-Register-Map1.pdf, page 42.

        # Now wake the 6050 up as it starts in sleep mode
        self.bus.write_byte_data(self.address, power_mgmt_1, 0)

        GYRO_CONFIG = self.bus.read_byte_data(self.address, 0x1B)
        self.bus.write_byte_data(self.address, 0x1B, GYRO_CONFIG | (FS_SEL<<3))
        # GYROSCOPE Sensitivity Scale Factor - MPU-6000-Datasheet1.pdf, page 12.
        self.scale_factor_gyro=131.0/(2**int(FS_SEL))

        ACCEL_CONFIG = self.bus.read_byte_data(self.address, 0x1C)
        self.bus.write_byte_data(self.address, 0x1C, ACCEL_CONFIG | (AFS_SEL<<3))
        # ACCELEROMETER Sensitivity Scale Factor - MPU-6000-Datasheet1.pdf, page 13.
        self.scale_factor_acc = float(2**(11-int(AFS_SEL)))

        accel_xout_scaled_max = 0
        accel_yout_scaled_max = 0
        accel_zout_scaled_max = 0


        # Creates the websocket to publish data
        self.publish = websocket.create_connection(self.rosbridge_url)


        self.publish.send(json.dumps({"op": "advertise",\
                                      "id": "IMU_WS_AC",\
                                      "topic": "/MPU6050/Accel",\
                                      "type": "geometry_msgs/Vector3"}))

        self.publish.send(json.dumps({"op": "advertise",\
                                      "id": "IMU_WS_GY",\
                                      "topic": "/MPU6050/Gyro",\
                                      "type": "geometry_msgs/Vector3"}))

        self.accel_dict={'op':'publish', 'id':'MPU_accel', 'topic':'/MPU6050/Accel', 'msg':{'x':0.0,'y':0.0,'z':0.0}}

        self.gyro_dict={'op':'publish', 'id':'MPU_gyro', 'topic':'/MPU6050/Gyro', 'msg':{'x':0.0,'y':0.0,'z':0.0}}

        self.send2ros()

    def read_word(self,adr):
        high = self.bus.read_byte_data(self.address, adr)
        low = self.bus.read_byte_data(self.address, adr+1)
        val = (high << 8) + low
        return val

    def read_word_2c(self,adr):
        val = self.read_word(adr)
        if (val >= 0x8000):
            return -((65535 - val) + 1)
        else:
            return val

    @staticmethod
    def dist(a,b):
        return math.sqrt((a*a)+(b*b))

    @staticmethod
    def get_y_rotation(x,y,z):
        radians = math.atan2(x, dist(y,z))
        return -math.degrees(radians)

    @staticmethod
    def get_x_rotation(x,y,z):
        radians = math.atan2(y, dist(x,z))
        return math.degrees(radians)


    def send2ros(self):
        print "\nmpu6050_PI has started!"

        try:
            while True:

                start_time = time.time()

                gyro_xout = self.read_word_2c(0x43)
                gyro_yout = self.read_word_2c(0x45)
                gyro_zout = self.read_word_2c(0x47)

                self.gyro_dict['msg']['x'] = gyro_xout / self.scale_factor_gyro
                self.gyro_dict['msg']['y'] = gyro_yout / self.scale_factor_gyro
                self.gyro_dict['msg']['z'] = gyro_zout / self.scale_factor_gyro

                accel_xout = self.read_word_2c(0x3b)
                accel_yout = self.read_word_2c(0x3d)
                accel_zout = self.read_word_2c(0x3f)

                self.accel_dict['msg']['x'] = accel_xout / self.scale_factor_acc
                self.accel_dict['msg']['y'] = accel_yout / self.scale_factor_acc
                self.accel_dict['msg']['z'] = accel_zout / self.scale_factor_acc


                if self.debug:
                    os.system("clear") # Clean the screen

                    print "gyro data"
                    print "---------"

                    print "gyro_xout: ", gyro_xout, " scaled: ", self.gyro_dict['msg']['x']
                    print "gyro_yout: ", gyro_yout, " scaled: ", self.gyro_dict['msg']['y']
                    print "gyro_zout: ", gyro_zout, " scaled: ", self.gyro_dict['msg']['z']

                    print
                    print "accelerometer data"
                    print "------------------"

                    if accel_xout_scaled>accel_xout_scaled_max:
                        accel_xout_scaled_max=self.accel_dict['msg']['x']

                    if accel_yout_scaled>accel_yout_scaled_max:
                        accel_yout_scaled_max=self.accel_dict['msg']['y']

                    if accel_zout_scaled>accel_zout_scaled_max:
                        accel_zout_scaled_max=self.accel_dict['msg']['z']

                    print "accel_xout: ", accel_xout, " scaled: ", self.accel_dict['msg']['x']
                    print "accel_yout: ", accel_yout, " scaled: ", self.accel_dict['msg']['y']
                    print "accel_zout: ", accel_zout, " scaled: ", self.accel_dict['msg']['z']

                    print "accel_xout_scaled_max: ", accel_xout_scaled_max
                    print "accel_yout_scaled_max: ", accel_yout_scaled_max
                    print "accel_zout_scaled_max: ", accel_zout_scaled_max

                    print "x rotation: " , get_x_rotation(self.accel_dict['msg']['x'], self.accel_dict['msg']['y'], self.accel_dict['msg']['z'])
                    print "y rotation: " , get_y_rotation(self.accel_dict['msg']['x'], self.accel_dict['msg']['y'], self.accel_dict['msg']['z'])

                self.publish.send(json.dumps(self.accel_dict))

                self.publish.send(json.dumps(self.gyro_dict))

                curr_delay=self.delay-(time.time()-start_time)
                if curr_delay>0:
                    time.sleep(curr_delay)

        except KeyboardInterrupt:
            print "\nmpu6050_PI has finished!"

        except:
            print "We have a problem..."
            raise




if __name__=="__main__":
    parser = argparse.ArgumentParser(description="Sends MPU6050 data to ROS using websocket+rosbridge")

    parser.add_argument("--delay", help="Delay between reads", type=float, default=0.1, required=False)

    parser.add_argument("--FS_SEL", help="GYROSCOPE sensitivity full-Scale Range(datasheet, page 12)", type=int, default=0, required=False)

    parser.add_argument("--AFS_SEL", help="ACCELEROMETER sensitivity full-Scale Range(datasheet, page 13)", type=int, default=3, required=False)

    parser.add_argument("--i2c_bus", help="i2c_bus according to the output from i2cdetect -l", type=int, default=1, required=False)

    parser.add_argument("--i2c_address", help="i2c_address according to the output from i2cdetect -y <i2c_bus>", type=int, default=68, required=False)

    parser.add_argument("--url", help="Rosbridge URL", type=str, required=True)

    parser.add_argument("--debug", help="Print the values", action="store_true")

    args=parser.parse_args() # processes everything

    mpu6050_PI(args.url, args.FS_SEL, args.AFS_SEL, args.i2c_bus, args.i2c_address, args.delay, args.debug)
