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
    return data


def MPU6050_start():
    # TODO: repopulate with serial confirmation of start and with IMU config value settings from serial connection (refer to source of fork)
    return 250.0, 2.0

def mpu6050_conv():
    # get list of imu values from serial connection
    write("imu")
    accel = read("ACCEL").split(":")[1][:-2].split("\t")
    gyro = read("GYRO").split(":")[1][:-2].split("\t")
    
    return float(accel[0]), float(accel[1]), float(accel[2]), float(gyro[0]), float(gyro[1]), float(gyro[2])

# def AK8963_start():
#     bus.write_byte_data(AK8963_ADDR,AK8963_CNTL,0x00)
#     time.sleep(0.1)
#     bus.write_byte_data(AK8963_ADDR,AK8963_CNTL,0x0F)
#     time.sleep(0.1)
#     coeff_data = bus.read_i2c_block_data(AK8963_ADDR,AK8963_ASAX,3)
#     AK8963_coeffx = (0.5*(coeff_data[0]-128)) / 256.0 + 1.0
#     AK8963_coeffy = (0.5*(coeff_data[1]-128)) / 256.0 + 1.0
#     AK8963_coeffz = (0.5*(coeff_data[2]-128)) / 256.0 + 1.0
#     time.sleep(0.1)
#     bus.write_byte_data(AK8963_ADDR,AK8963_CNTL,0x00)
#     time.sleep(0.1)
#     AK8963_bit_res = 0b0001 # 0b0001 = 16-bit
#     AK8963_samp_rate = 0b0110 # 0b0010 = 8 Hz, 0b0110 = 100 Hz
#     AK8963_mode = (AK8963_bit_res <<4)+AK8963_samp_rate # bit conversion
#     bus.write_byte_data(AK8963_ADDR,AK8963_CNTL,AK8963_mode)
#     time.sleep(0.1)
#     return [AK8963_coeffx,AK8963_coeffy,AK8963_coeffz] 
    
# def AK8963_reader(register):
#     # read magnetometer values
#     low = bus.read_byte_data(AK8963_ADDR, register-1)
#     high = bus.read_byte_data(AK8963_ADDR, register)
#     # combine higha and low for unsigned bit value
#     value = ((high << 8) | low)
#     # convert to +- value
#     if(value > 32768):
#         value -= 65536
    
#     return value

# def AK8963_conv():
#     # raw magnetometer bits
#     while 1:
# ##        if ((bus.read_byte_data(AK8963_ADDR,AK8963_ST1) & 0x01))!=1:
# ##            return 0,0,0
#         mag_x = AK8963_reader(HXH)
#         mag_y = AK8963_reader(HYH)
#         mag_z = AK8963_reader(HZH)

#         # the next line is needed for AK8963
#         if (bus.read_byte_data(AK8963_ADDR,AK8963_ST2)) & 0x08!=0x08:
#             break
        
#     #convert to acceleration in g and gyro dps
# ##    m_x = AK8963_coeffs[0]*(mag_x/(2.0**15.0))*mag_sens
# ##    m_y = AK8963_coeffs[1]*(mag_y/(2.0**15.0))*mag_sens
# ##    m_z = AK8963_coeffs[2]*(mag_z/(2.0**15.0))*mag_sens
#     m_x = (mag_x/(2.0**15.0))*mag_sens
#     m_y = (mag_y/(2.0**15.0))*mag_sens
#     m_z = (mag_z/(2.0**15.0))*mag_sens
#     return m_x,m_y,m_z

# mag_sens = 4800.0 # magnetometer sensitivity: 4800 uT

gyro_sens,accel_sens = MPU6050_start() # instantiate gyro/accel
time.sleep(0.1)
# AK8963_coeffs = AK8963_start() # instantiate magnetometer
# time.sleep(0.1)

