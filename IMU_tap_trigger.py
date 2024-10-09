# This work is licensed under the MIT license.
# Copyright (c) 2013-2023 OpenMV LLC. All rights reserved.
# https://github.com/openmv/openmv/blob/master/LICENSE
#
# Hello World Example
#
# Welcome to the OpenMV IDE! Click on the green run arrow button below to run the script!

import sensor
import time, os
import math
import machine
from lsm6dsox import LSM6DSOX
from machine import Pin, SPI, LED

# Initialize the LSM6DSOX sensor
spi = SPI(5)
cs = Pin("PF6", Pin.OUT_PP, Pin.PULL_UP)
lsm = LSM6DSOX(spi, cs)

# Initialize the sensor parameters
sr = 104   # sr of IMU, default is 104 Hz
acc_window_t = 100   # window size to aggregate sensor data, ms
window_acc_z = []
window_acc_x = []
window_acc_y = []
window_gyro_x = []
window_gyro_y = []
window_gyro_z = []
window_acc_z_tmp = []
window_acc_x_tmp = []
window_acc_y_tmp = []
window_gyro_x_tmp = []
window_gyro_y_tmp = []
window_gyro_z_tmp = []

# Initialize the camera sensor
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time=2000)

# Initialize the LED
led = LED("LED_BLUE")
capture_interval = 3000   # LED duration, ms
last_capture_time = time.ticks_ms()

tap = 0   # tap indicator
tap_count = 0
flg_sleep = 0

start_time = time.ticks_ms()

while True:
    current_time = time.ticks_ms()
    # get IMU data
    accel_x, accel_y, accel_z = lsm.accel()
    gyro_x, gyro_y, gyro_z = lsm.gyro()
    #accel_x = math.sqrt(accel_x**2)
    #accel_y = math.sqrt(accel_y**2)
    #accel_z = math.sqrt(accel_z**2)

    # if LED is on, keep waiting
    if flg_sleep == 1:
        flg_sleep = 0
        start_time = current_time
        continue
    # aggregate IMU for every acc_window_t ms to improve robustness
    if time.ticks_diff(current_time, start_time) <= acc_window_t:
        window_acc_z_tmp.append(accel_z)
        window_acc_x_tmp.append(accel_x)
        window_acc_y_tmp.append(accel_y)
        window_gyro_x_tmp.append(gyro_x)
        window_gyro_y_tmp.append(gyro_y)
        window_gyro_z_tmp.append(gyro_z)
        continue
    # execute every acc_window_t ms
    acc_z_tmp = sum(window_acc_z_tmp) / len(window_acc_z_tmp)
    acc_x_tmp = sum(window_acc_x_tmp) / len(window_acc_x_tmp)
    acc_y_tmp = sum(window_acc_y_tmp) / len(window_acc_y_tmp)
    gyro_x_tmp = sum(window_gyro_x_tmp) / len(window_gyro_x_tmp)
    gyro_y_tmp = sum(window_gyro_y_tmp) / len(window_gyro_y_tmp)
    gyro_z_tmp = sum(window_gyro_z_tmp) / len(window_gyro_z_tmp)
    window_acc_z.append(acc_z_tmp)
    window_acc_x.append(acc_x_tmp)
    window_acc_y.append(acc_y_tmp)
    window_gyro_x.append(gyro_x_tmp)
    window_gyro_y.append(gyro_y_tmp)
    window_gyro_z.append(gyro_z_tmp)
    start_time = current_time
    window_acc_z_tmp = []
    window_acc_x_tmp = []
    window_acc_y_tmp = []
    window_gyro_x_tmp = []
    window_gyro_y_tmp = []
    window_gyro_z_tmp = []

    if tap == 1:
        tap_wait += 1
        # do not count if adjacent taps too far apart
        if tap_wait > 8:
            tap = 0
            tap_wait = 0
    # tap triggering condition
    #if abs(acc_z_tmp) > 1.0 and abs(gyro_y_tmp) < 50 and abs(gyro_z_tmp) < 20:
    if abs(acc_z_tmp) > 0.8:
        if tap == 0:
            tap = 1
            tap_wait = 0   # N, unit: N * acc_window_t ms
            #print("sr:", "acc:", acc_x_tmp, acc_y_tmp, acc_z_tmp, "gyr:", gyro_x_tmp, gyro_y_tmp, gyro_z_tmp)
        else:
            # do not count if adjacent taps too close
            if tap_wait < 2:
                tap = 0
                tap_wait = 0
            else:
                tap_count += 1
                print("double tap # %d" % tap_count)
                led.on()
                tap = 0
                tap_wait = 0
                time.sleep_ms(1000)
                img = sensor.snapshot()
                img.save("/tmp/%d.jpeg" % tap_count)
                flg_sleep = 1
                led.off()


    #print("Accelerometer: x:{:>8.3f} y:{:>8.3f} z:{:>8.3f}".format(accel_x, accel_y, accel_z))
    #print("Gyroscope:     x:{:>8.3f} y:{:>8.3f} z:{:>8.3f}".format(gyro_x, gyro_y, gyro_z))
    #print("")


with open('/tmp/aaa.csv','w') as file:
        #for line in file:
            file.write(str(window_acc_z))
            #line_Str=line_Str.rstrip('\n')
            #line_Str=line_Str.rstrip('\r')
file.close()
