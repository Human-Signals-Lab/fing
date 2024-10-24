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
from vl53l1x import VL53L1X
from machine import Pin, SPI, LED, I2C

# Constants
DOUBLE_TAP_THRESHOLD_MIN = 20  # threshold for detecting a tap, mm
DOUBLE_TAP_THRESHOLD_MAX = 5  # threshold for detecting a tap, mm
TAP_TIME_LIMIT = 2000  # Time limit in seconds for detecting a double tap, ms


# Initialize the LSM6DSOX sensor
spi = SPI(5)
cs = Pin("PF6", Pin.OUT_PP, Pin.PULL_UP)
lsm = LSM6DSOX(spi, cs)

tof = VL53L1X(I2C(2))

# Initialize the sensor parameters
duration_to_save = 10000   # duration to save sensor readings, ms
sr = 104   # sr of IMU, default is 104 Hz
acc_window_t = 100   # window size to aggregate sensor data, ms

# Initialize the camera sensor
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time=1000)

# Initialize the LED
led = LED("LED_BLUE")
capture_interval = 3000   # LED duration, ms
last_capture_time = time.ticks_ms()
led2 = LED("LED_GREEN")

def nod():
    tap = 0   # tap indicator
    tap_count = 0
    flg_sleep = 0

    IMU_idx = ["time"]
    window_acc_x = ["acc_x"]
    window_acc_y = ["acc_y"]
    window_acc_z = ["acc_z"]
    window_gyro_x = ["gyr_x"]
    window_gyro_y = ["gyr_y"]
    window_gyro_z = ["gyr_z"]
    window_acc_z_tmp = []
    window_acc_x_tmp = []
    window_acc_y_tmp = []
    window_gyro_x_tmp = []
    window_gyro_y_tmp = []
    window_gyro_z_tmp = []

    start_time_for_note = time.ticks_ms()

    while True:
        current_time_for_note = time.ticks_ms()
        # note the IMU readings for evert x milliseconds
        if abs(time.ticks_diff(current_time_for_note, start_time_for_note)) > duration_to_save:
            led2.on()
            with open('/tmp/IMU.csv','a') as file:
                    #for line in file:
                    str_to_save = "\n" + str(IMU_idx).strip("[]") + \
                    "\n" + str(window_acc_x).strip("[]") + \
                    "\n" + str(window_acc_y).strip("[]") + \
                    "\n" + str(window_acc_z).strip("[]") + \
                    "\n" + str(window_gyro_x).strip("[]") + \
                    "\n" + str(window_gyro_y).strip("[]") + \
                    "\n" + str(window_gyro_z).strip("[]")
                    file.write(str_to_save)
                        #line_Str=line_Str.rstrip('\n')
                        #line_Str=line_Str.rstrip('\r')
            file.close()
            # clear the saved window
            IMU_idx = ["time"]
            window_acc_x = ["acc_x"]
            window_acc_y = ["acc_y"]
            window_acc_z = ["acc_z"]
            window_gyro_x = ["gyr_x"]
            window_gyro_y = ["gyr_y"]
            window_gyro_z = ["gyr_z"]
            start_time_for_note = current_time_for_note
            time.sleep_ms(1000)
            led2.off()

        # aggregate IMU for every acc_window_t ms to improve robustness
        start_time = time.ticks_ms()
        current_time = start_time
        while abs(time.ticks_diff(current_time, start_time)) <= acc_window_t:
            # get IMU data
            accel_x, accel_y, accel_z = lsm.accel()
            gyro_x, gyro_y, gyro_z = lsm.gyro()
            #print(time.ticks_diff(current_time, start_time))
            window_acc_z_tmp.append(accel_z)
            window_acc_x_tmp.append(accel_x)
            window_acc_y_tmp.append(accel_y)
            window_gyro_x_tmp.append(gyro_x)
            window_gyro_y_tmp.append(gyro_y)
            window_gyro_z_tmp.append(gyro_z)
            current_time = time.ticks_ms()

        # execute every acc_window_t ms
        acc_z_tmp = sum(window_acc_z_tmp) / len(window_acc_z_tmp)
        acc_x_tmp = sum(window_acc_x_tmp) / len(window_acc_x_tmp)
        acc_y_tmp = sum(window_acc_y_tmp) / len(window_acc_y_tmp)
        gyro_x_tmp = sum(window_gyro_x_tmp) / len(window_gyro_x_tmp)
        gyro_y_tmp = sum(window_gyro_y_tmp) / len(window_gyro_y_tmp)
        gyro_z_tmp = sum(window_gyro_z_tmp) / len(window_gyro_z_tmp)
        # new window to save
        IMU_time_stamp = time.strftime("%Y-%m-%d-%H-%M-%S", time.localtime())
        IMU_idx.append(IMU_time_stamp)
        window_acc_z.append(acc_z_tmp)
        window_acc_x.append(acc_x_tmp)
        window_acc_y.append(acc_y_tmp)
        window_gyro_x.append(gyro_x_tmp)
        window_gyro_y.append(gyro_y_tmp)
        window_gyro_z.append(gyro_z_tmp)
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
        if abs(acc_z_tmp) > 0.6 and tof.read() < 500:   # tof: mm
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
                    print("time stamp:", IMU_time_stamp)
                    print(f"Distance: {tof.read()}mm")
                    led.on()
                    tap = 0
                    tap_wait = 0
                    img = sensor.snapshot()
                    #file_name = time.strftime("%Y-%m-%d-%H-%M-%S", time.localtime())
                    img.save("/tmp/%s.jpeg" % IMU_time_stamp)
                    time.sleep_ms(1000)
                    led.off()


# Function to detect double tap
def tap_condition(dis):
    if dis < DOUBLE_TAP_THRESHOLD_MIN:
        print("min:", dis)
        # sleep for a bit to see if it is still triggered
        time.sleep_ms(500)
        dis2 = tof.read()
        print("max:", dis2)
        if abs(dis - dis2) > DOUBLE_TAP_THRESHOLD_MAX:
            return 1
    return 0

def detect_double_tap():
    tap = 0   # tap indicator
    tap_count = 0
    flg_sleep = 0

    IMU_idx = ["time"]
    window_acc_x = ["acc_x"]
    window_acc_y = ["acc_y"]
    window_acc_z = ["acc_z"]
    window_gyro_x = ["gyr_x"]
    window_gyro_y = ["gyr_y"]
    window_gyro_z = ["gyr_z"]
    window_acc_z_tmp = []
    window_acc_x_tmp = []
    window_acc_y_tmp = []
    window_gyro_x_tmp = []
    window_gyro_y_tmp = []
    window_gyro_z_tmp = []

    start_time_for_note = time.ticks_ms()

    while True:
        current_time_for_note = time.ticks_ms()
        # note the IMU readings for evert x milliseconds
        if abs(time.ticks_diff(current_time_for_note, start_time_for_note)) > duration_to_save:
            led2.on()
            with open('/tmp/IMU.csv','a') as file:
                    #for line in file:
                    str_to_save = "\n" + str(IMU_idx).strip("[]") + \
                    "\n" + str(window_acc_x).strip("[]") + \
                    "\n" + str(window_acc_y).strip("[]") + \
                    "\n" + str(window_acc_z).strip("[]") + \
                    "\n" + str(window_gyro_x).strip("[]") + \
                    "\n" + str(window_gyro_y).strip("[]") + \
                    "\n" + str(window_gyro_z).strip("[]")
                    file.write(str_to_save)
                        #line_Str=line_Str.rstrip('\n')
                        #line_Str=line_Str.rstrip('\r')
            file.close()
            # clear the saved window
            IMU_idx = ["time"]
            window_acc_x = ["acc_x"]
            window_acc_y = ["acc_y"]
            window_acc_z = ["acc_z"]
            window_gyro_x = ["gyr_x"]
            window_gyro_y = ["gyr_y"]
            window_gyro_z = ["gyr_z"]
            start_time_for_note = current_time_for_note
            time.sleep_ms(1000)
            led2.off()

        # aggregate IMU for every acc_window_t ms to improve robustness
        start_time = time.ticks_ms()
        current_time = start_time
        while abs(time.ticks_diff(current_time, start_time)) <= acc_window_t:
            # get IMU data
            accel_x, accel_y, accel_z = lsm.accel()
            gyro_x, gyro_y, gyro_z = lsm.gyro()
            #print(time.ticks_diff(current_time, start_time))
            window_acc_z_tmp.append(accel_z)
            window_acc_x_tmp.append(accel_x)
            window_acc_y_tmp.append(accel_y)
            window_gyro_x_tmp.append(gyro_x)
            window_gyro_y_tmp.append(gyro_y)
            window_gyro_z_tmp.append(gyro_z)
            current_time = time.ticks_ms()

        # execute every acc_window_t ms
        acc_z_tmp = sum(window_acc_z_tmp) / len(window_acc_z_tmp)
        acc_x_tmp = sum(window_acc_x_tmp) / len(window_acc_x_tmp)
        acc_y_tmp = sum(window_acc_y_tmp) / len(window_acc_y_tmp)
        gyro_x_tmp = sum(window_gyro_x_tmp) / len(window_gyro_x_tmp)
        gyro_y_tmp = sum(window_gyro_y_tmp) / len(window_gyro_y_tmp)
        gyro_z_tmp = sum(window_gyro_z_tmp) / len(window_gyro_z_tmp)
        # new window to save
        IMU_time_stamp = time.strftime("%Y-%m-%d-%H-%M-%S", time.localtime())
        IMU_idx.append(IMU_time_stamp)
        window_acc_z.append(acc_z_tmp)
        window_acc_x.append(acc_x_tmp)
        window_acc_y.append(acc_y_tmp)
        window_gyro_x.append(gyro_x_tmp)
        window_gyro_y.append(gyro_y_tmp)
        window_gyro_z.append(gyro_z_tmp)
        window_acc_z_tmp = []
        window_acc_x_tmp = []
        window_acc_y_tmp = []
        window_gyro_x_tmp = []
        window_gyro_y_tmp = []
        window_gyro_z_tmp = []

        dis = tof.read()
        # Check if a tap exists
        if tap_condition(dis=dis):
            if tap == 0:
                tap = 1
                tap_point = time.ticks_ms()
                continue
            elif abs(time.ticks_diff(tap_point, time.ticks_ms())) < TAP_TIME_LIMIT:
                tap_count += 1
                print("double tap # %d" % tap_count)
                print("time stamp:", IMU_time_stamp)
                print(f"Distance: {tof.read()}mm")
                led.on()
                tap = 0
                img = sensor.snapshot()
                #file_name = time.strftime("%Y-%m-%d-%H-%M-%S", time.localtime())
                img.save("/tmp/%s.jpeg" % IMU_time_stamp)
                time.sleep_ms(1000)
                led.off()
            else:
                tap = 0


#detect_double_tap()
nod()
