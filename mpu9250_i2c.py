#########################################
# Copyright (c) 2020 Maker Portal LLC
# Author: Joshua Hrisko
#########################################
#
# This code handles the smbus 
# communications between the RPi and the
# MPU9250 IMU. For testing the MPU9250
# see: imu_test.py
#
#########################################
#
import time
import serial

# port=input("Enter the port number: ")

arduino = serial.Serial(port="/dev/ttyACM0", baudrate=115200, timeout=.1)

def write(x):
    arduino.write(bytes(x + "\n", 'utf-8'))
    time.sleep(0.05)

def read(check):
    data = arduino.readline().decode("utf-8")
    while check not in data:
        data = arduino.readline().decode("utf-8")
    time.sleep(0.05)
    return data

def mpu6050_conv():
    # get list of imu values from serial connection
    write("imu")
    accel = read("ACCEL").split(":")[1][:-2].split("\t") # must be m/s^2
    gyro = read("GYRO").split(":")[1][:-2].split("\t") # must be degrees/sec
    
    return float(accel[0]), float(accel[1]), float(accel[2]), float(gyro[0]), float(gyro[1]), float(gyro[2])

def read_magnetometer():
    # get list of magnetometer values from serial connection
    write("mag")

    mag = read("MAG").split(":")[1][:-2].split("\t") # must be gauss
    
    return float(mag[0]), float(mag[1]), float(mag[2])

# print(read_mag())