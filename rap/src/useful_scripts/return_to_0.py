#!/usr/bin/env python3

__authors__ = "David Ho"

import time

from board import SCL, SDA
import busio
from adafruit_motor import servo
from adafruit_pca9685 import PCA9685


def main():
    i2c = busio.I2C(SCL, SDA)
    pca = PCA9685(i2c)
    pca.frequency = 50

    # Initializing each servos arcordingly

    S0 = servo.Servo(pca.channels[0], min_pulse=500, max_pulse=2500, actuation_range=270)
    S1 = servo.Servo(pca.channels[1], min_pulse=500, max_pulse=2500, actuation_range=270)
    S2 = servo.Servo(pca.channels[2], min_pulse=500, max_pulse=2500, actuation_range=270)
    S3 = servo.Servo(pca.channels[3], min_pulse=500, max_pulse=2500, actuation_range=270)
    S4 = servo.Servo(pca.channels[4], min_pulse=500, max_pulse=2500, actuation_range=270)
    S5 = servo.Servo(pca.channels[5], min_pulse=500, max_pulse=2500, actuation_range=270)
    S6 = servo.Servo(pca.channels[6], min_pulse=500, max_pulse=2500, actuation_range=270)
    S7 = servo.Servo(pca.channels[7], min_pulse=500, max_pulse=2500, actuation_range=270)
    S8 = servo.Servo(pca.channels[8], min_pulse=500, max_pulse=2500, actuation_range=270)
    S9 = servo.Servo(pca.channels[9], min_pulse=500, max_pulse=2500, actuation_range=270)
    S10 = servo.Servo(pca.channels[10], min_pulse=500, max_pulse=2500, actuation_range=270)
    
    time.sleep(1)

    while True:
        theta = 0

        S0.angle = theta
        S1.angle = theta
        time.sleep(1)
        S2.angle = theta
        S3.angle = theta
        time.sleep(1)
        S4.angle = theta
        S5.angle = theta
        time.sleep(1)
        S6.angle = theta
        S7.angle = theta
        time.sleep(1)
        S8.angle = theta
        S9.angle = theta
        S10.angle = theta
        time.sleep(1)

        # time.sleep(5)

        theta = 270

        S0.angle = theta
        S1.angle = theta
        time.sleep(1)
        S2.angle = theta
        S3.angle = theta
        time.sleep(1)
        S4.angle = theta
        S5.angle = theta
        time.sleep(1)
        S6.angle = theta
        S7.angle = theta
        time.sleep(1)
        S8.angle = theta
        S9.angle = theta
        S10.angle = theta
        time.sleep(1)

        # time.sleep(5)

if __name__ == '__main__':
    try:
        main()

    except (KeyboardInterrupt, SystemExit):
        pass
