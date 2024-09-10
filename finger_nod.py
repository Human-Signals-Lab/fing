import sensor
import time
import machine
from lsm6dsox import LSM6DSOX
from machine import Pin, SPI, LED

# Initialize the LSM6DSOX sensor
spi = SPI(5)
cs = Pin("PF6", Pin.OUT_PP, Pin.PULL_UP)
lsm = LSM6DSOX(spi, cs)

# Initialize the camera sensor
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time=2000)

# Initialize the LED
led = LED("LED_BLUE")
capture_interval = 3000
last_capture_time = time.ticks_ms()

picture_taken = False
passing = False
nods = 0
while True:
    current_time = time.ticks_ms()
    if time.ticks_diff(current_time, last_capture_time) >= capture_interval and passing:
        if not picture_taken:
            print("Taking picture...")
            led.on()
            time.sleep_ms(1000)
            img = sensor.snapshot()
            img.save("example.jpg")
            led.off()

#            picture_taken = True
#            passing = False
            break


        last_capture_time = current_time

    accel_x, accel_y, accel_z = lsm.accel()
    gyro_x, gyro_y, gyro_z = lsm.gyro()

    if abs(gyro_y) > 100 and abs(gyro_x) < 50 and abs(gyro_z) < 50:
        nods +=1
    if nods == 2:
        passing = True
        nods = 0



    print("Accelerometer: x:{:>8.3f} y:{:>8.3f} z:{:>8.3f}".format(accel_x, accel_y, accel_z))
    print("Gyroscope:     x:{:>8.3f} y:{:>8.3f} z:{:>8.3f}".format(gyro_x, gyro_y, gyro_z))
    print(f"Nods:", nods)
    print("")

    sensor.snapshot()

    time.sleep_ms(50)
