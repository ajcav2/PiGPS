#!/usr/bin/python

import RPi.GPIO as GPIO
import smbus
import math
import time
import serial # and milk
import subprocess
import os
import signal
import threading
import thread
import io 

# To read GPS data from USB
ser = serial.Serial('/dev/ttyACM0', 9600)

# Power management registers
power_mgmt_1 = 0x6b
power_mgmt_2 = 0x6c

bus = smbus.SMBus(1)
address = 0x68       # This is the address value read via the i2cdetect command

# Now wake the 6050 up as it starts in sleep mode
bus.write_byte_data(address, power_mgmt_1, 0)

# Setup GPIO pins
led = 27
GPIO.setmode(GPIO.BCM)
GPIO.setup(17,GPIO.IN,pull_up_down=GPIO.PUD_UP)
GPIO.setup(led,GPIO.OUT)

record = False
x = []
y = []
z = []
gyro_x = []
gyro_y = []
gyro_z = []
rot_x = []
rot_y = []
GPS_alt = []

def serialRecorder():
    global record
    while record:
        line = ser.readline()
        if len(line) > 1 and "PUBX" in line:
            line = line.split('W')[1]
            line = line[1:]
            line = line.split(',')[0]
            GPS_alt.append(line)
    
def beginRecording():
    global record
    global GPS
    while not record:
        time.sleep(1.5)
        print("Recording: "+str(record))
    GPIO.output(led,True)

    # Start serial read on a new thread so that the MPU doesn't have
    # to wait for the serial read
    record_thread = threading.Thread(target=serialRecorder, args=[])
    record_thread.daemon = True
    record_thread.start()
    
    while record:

        print "gyro data"
        print "---------"

        gyro_xout = read_word_2c(0x43)
        gyro_yout = read_word_2c(0x45)
        gyro_zout = read_word_2c(0x47)

        print "gyro_xout: ", gyro_xout, " scaled: ", (gyro_xout / 131)
        print "gyro_yout: ", gyro_yout, " scaled: ", (gyro_yout / 131)
        print "gyro_zout: ", gyro_zout, " scaled: ", (gyro_zout / 131)

        print
        print "accelerometer data"
        print "------------------"

        accel_xout = read_word_2c(0x3b)
        accel_yout = read_word_2c(0x3d)
        accel_zout = read_word_2c(0x3f)

        accel_xout_scaled = accel_xout / 16384.0
        accel_yout_scaled = accel_yout / 16384.0
        accel_zout_scaled = accel_zout / 16384.0

        print "accel_xout: ", accel_xout, " scaled: ", accel_xout_scaled
        print "accel_yout: ", accel_yout, " scaled: ", accel_yout_scaled
        print "accel_zout: ", accel_zout, " scaled: ", accel_zout_scaled

        print "x rotation: " , get_x_rotation(accel_xout_scaled, accel_yout_scaled, accel_zout_scaled)
        print "y rotation: " , get_y_rotation(accel_xout_scaled, accel_yout_scaled, accel_zout_scaled)

        x.append(accel_xout_scaled)
        y.append(accel_yout_scaled)
        z.append(accel_zout_scaled)

        gyro_x.append(gyro_xout/131)
        gyro_y.append(gyro_yout/131)
        gyro_z.append(gyro_zout/131)

        rot_x.append(get_x_rotation(accel_xout_scaled, accel_yout_scaled, accel_zout_scaled))
        rot_y.append(get_y_rotation(accel_xout_scaled, accel_yout_scaled, accel_zout_scaled))

        
    GPIO.output(led,False)
    print("Writing...")
    write(x,"x_accel_MPU")
    write(y,"y_accel_MPU")
    write(z,"z_accel_MPU")
    write(gyro_x,"gyro_x")
    write(gyro_y,"gyro_y")
    write(gyro_z,"gyro_z")
    write(rot_x,"rot_x")
    write(rot_y,"rot_y")
    write(GPS_alt,"GPS_alt")
    print("Done.")

def write(lst,fname):
    with open("/home/pi/Documents/ae456final/data/raw/"+fname,"w") as f:
        for val in lst:
            f.write("%s\n" % val)
        f.close()

def read_byte(adr):
    return bus.read_byte_data(address, adr)

def read_word(adr):
    high = bus.read_byte_data(address, adr)
    low = bus.read_byte_data(address, adr+1)
    val = (high << 8) + low
    return val

def read_word_2c(adr):
    val = read_word(adr)
    if (val >= 0x8000):
        return -((65535 - val) + 1)
    else:
        return val

def dist(a,b):
    return math.sqrt((a*a)+(b*b))

def get_y_rotation(x,y,z):
    radians = math.atan2(x, dist(y,z))
    return -math.degrees(radians)

def get_x_rotation(x,y,z):
    radians = math.atan2(y, dist(x,z))
    return math.degrees(radians)

def toggleRecording(self):
    global record
    record = not record

if __name__ == "__main__":
    GPIO.add_event_detect(17,GPIO.FALLING,callback=toggleRecording,bouncetime=300)
    beginRecording()
    
