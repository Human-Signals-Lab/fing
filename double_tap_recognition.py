# Double Tap Recognition - By: Bobby Chiu - Tue Oct 15 2024

import sensor, image, time, math

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time = 2000)

clock = time.clock()

from lsm6dsox import LSM6DSOX
from vl53l1x import VL53L1X
from machine import Pin, SPI, LED, I2C

# Initialize the LSM6DSOX sensor
spi = SPI(5)
cs = Pin("PF6", Pin.OUT_PP, Pin.PULL_UP)
lsm = LSM6DSOX(spi, cs)

tof = VL53L1X(I2C(2))

# Constants
DOUBLE_TAP_THRESHOLD = 1.2  # Acceleration threshold for detecting a tap
TAP_TIME_LIMIT = 0.5  # Time limit in seconds for detecting a double tap
SAMPLE_RATE = 10  # Sample rate in Hz (adjust as needed)

# Function to detect double tap
def detect_double_tap():
    tap_times = []

    while True:
        accel_x, accel_y, accel_z = lsm.accel()
        # Calculate the magnitude of acceleration
        accel_magnitude = math.sqrt(accel_x**2 + accel_y**2 + accel_z**2)
#        print(f"Accel mag: {accel_magnitude}")

        # Check if the acceleration exceeds the threshold
        if accel_magnitude > DOUBLE_TAP_THRESHOLD and accel_magnitude < 2 and tof.read() < 300 and tof.read() > 100:
            tap_times.append(time.time())
            # Remove taps older than TAP_TIME_LIMIT seconds
            tap_times = [t for t in tap_times if t > time.time() - TAP_TIME_LIMIT]
#            print(len(tap_times))

        # Check if we have two taps within the time limit
        if len(tap_times) == 2:
            print("Double tap detected!")
            tap_times.clear()  # Reset for the next detection

        time.sleep(1/SAMPLE_RATE)  # Simulate sampling rate

detect_double_tap()

