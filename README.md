# MPU6050_to_ROS
## Reads data from MPU6050 using a RaspberryPI and sends to ROS (Rosbridge)
### [How to install and launch Rosbridge](http://wiki.ros.org/rosbridge_suite/Tutorials/RunningRosbridge)  
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
i2c-1	i2c       	3f804000.i2c                    	I2C adapter  
The device can be reached by "/dev/i2c-1".  
And i2c_bus=1 in this situation.  

Connect the MPU6050 and verify the i2c address:  
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

For debugging, it's possible to access individual registers using the commands i2cset and i2cget.  
https://www.olimex.com/Products/Modules/Sensors/MOD-MPU6050/resources/RM-MPU-60xxA_rev_4.pdf  
https://learn.sparkfun.com/tutorials/i2c  

You will also need to install this:  
sudo apt-get install python-smbus  

The full list of things you are supposed to install:  
sudo apt-get install i2c-tools libi2c-dev python-dev python-smbus  

To query the functionalities of an  I2C  bus:  
i2cdetect -F 1  


http://ricardodeazambuja.com/
