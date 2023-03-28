######################################################
# Copyright (c) 2021 Maker Portal LLC
# Author: Joshua Hrisko
######################################################
#
# This code reads data from the MPU9250/MPU9265 board
# (MPU6050 - accel/gyro, AK8963 - mag)
# and solves for calibration coefficients for the
# accelerometer
#
#
######################################################
#
# wait 5-sec for IMU to connect
import time,sys
sys.path.append('../')
t0 = time.time()
start_bool = False # if IMU start fails - stop calibration
while time.time()-t0<5:
    try: 
        from mpu9250_i2c import *
        start_bool = True
        break
    except:
        continue
import numpy as np
import csv,datetime
import matplotlib.pyplot as plt
from scipy.optimize import curve_fit
from scipy.integrate import cumtrapz
from scipy import signal

time.sleep(2) # wait for MPU to load and settle
# 
#####################################
# Accel Calibration (gravity)
#####################################
#
def get_accel():
    ax,ay,az,_,_,_ = mpu6050_conv() # read and convert accel data
    return ax,ay,az

def imu_integrator():
    #############################
    # Main Loop to Integrate IMU
    #############################
    #
    data_indx = 1 # index of variable to integrate
    dt_stop = 5 # seconds to record and integrate

    plt.style.use('ggplot')
    plt.ion()
    fig,axs = plt.subplots(3,1,figsize=(12,9))
    break_bool = False
    while True:
        #
        ##################################
        # Reading and Printing IMU values 
        ##################################
        #
        accel_array,t_array = [],[]
        print("Starting Data Acquisition")
        [axs[ii].clear() for ii in range(0,3)]
        t0 = time.time()
        loop_bool = False
        while True:
            try:
                ax,ay,az,wx,wy,wz = mpu6050_conv() # read and convert mpu6050 data
                mx,my,mz = read_magnetometer() # read magnetometer data
                t_array.append(time.time()-t0)
                data_array = [ax,ay,az,wx,wy,wz,mx,my,mz]

                if not loop_bool:
                    loop_bool = True
                    print("Start Moving IMU...")
                else:
                    # print progress %
                    print("\r{:2.0f}%".format(100*(time.time()-t0)/dt_stop),end='') 
            except:
                continue
            if time.time()-t0>dt_stop:
                print("Data Acquisition Stopped")
                break
            
        if break_bool:
            break
        #
        ##################################
        # Signal Filtering
        ##################################
        #
        Fs_approx = len(accel_array)/dt_stop
        b_filt,a_filt = signal.butter(4,5,'low',fs=Fs_approx)
        accel_array = signal.filtfilt(b_filt,a_filt,accel_array)
        accel_array = np.multiply(accel_array,9.80665)
        #
        ##################################
        # Print Sample Rate and Accel
        # Integration Value
        ##################################
        #
        print("Sample Rate: {0:2.0f}Hz".format(len(accel_array)/dt_stop))
        veloc_array = np.append(0.0,cumtrapz(accel_array,x=t_array))
        dist_approx = np.trapz(veloc_array,x=t_array)
        dist_array = np.append(0.0,cumtrapz(veloc_array,x=t_array))
        print("Displace in y-dir: {0:2.2f}m".format(dist_approx))
        axs[0].plot(t_array,accel_array,label="$"+mpu_labels[data_indx]+"$",
                    color=plt.cm.Set1(0),linewidth=2.5)
        axs[1].plot(t_array,veloc_array,
                    label="$v_"+mpu_labels[data_indx].split("_")[1]+"$",
                    color=plt.cm.Set1(1),linewidth=2.5)
        axs[2].plot(t_array,dist_array,
                    label="$d_"+mpu_labels[data_indx].split("_")[1]+"$",
                    color=plt.cm.Set1(2),linewidth=2.5)
        [axs[ii].legend() for ii in range(0,len(axs))]
        axs[0].set_ylabel('Acceleration [m$\cdot$s$^{-2}$]',fontsize=16)
        axs[1].set_ylabel('Velocity [m$\cdot$s$^{-1}$]',fontsize=16)
        axs[2].set_ylabel('Displacement [m]',fontsize=16)
        axs[2].set_xlabel('Time [s]',fontsize=18)
        axs[0].set_title("MPU9250 Accelerometer Integration",fontsize=18)
        plt.pause(0.01)
        plt.savefig("accel_veloc_displace_integration.png",dpi=300,
                    bbox_inches='tight',facecolor="#FCFCFC")

if __name__ == '__main__':
    if not start_bool:
        print("IMU not Started - Check Wiring and I2C")
    else:
        mpu_labels = ['a_x','a_y','a_z'] # gyro labels for plots

        ###################################
        # integration over time
        ###################################
        #
        imu_integrator()
