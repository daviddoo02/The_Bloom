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

    S00 = servo.Servo(pca.channels[0], min_pulse=500, max_pulse=2600, actuation_range=270)
    S01 = servo.Servo(pca.channels[1], min_pulse=500, max_pulse=2600, actuation_range=270)
    S10 = servo.Servo(pca.channels[2], min_pulse=500, max_pulse=2600, actuation_range=270)
    S11 = servo.Servo(pca.channels[3], min_pulse=500, max_pulse=2600, actuation_range=270)
    S20 = servo.Servo(pca.channels[4], min_pulse=500, max_pulse=2600, actuation_range=270)
    S21 = servo.Servo(pca.channels[5], min_pulse=500, max_pulse=2600, actuation_range=270)
    S30 = servo.Servo(pca.channels[6], min_pulse=500, max_pulse=2600, actuation_range=270)
    S31 = servo.Servo(pca.channels[7], min_pulse=500, max_pulse=2600, actuation_range=270)
    B0 = servo.Servo(pca.channels[8], min_pulse=500, max_pulse=2600, actuation_range=270)
    B1 = servo.Servo(pca.channels[9], min_pulse=500, max_pulse=2600, actuation_range=270)
    B2 = servo.Servo(pca.channels[10], min_pulse=500, max_pulse=2600, actuation_range=270)
    
    time.sleep(1)

    while True:
        theta = 0
        
        B0.angle = theta
        time.sleep(0.5)
        S30.angle = theta
        S31.angle = theta
        time.sleep(0.5)
        S10.angle = theta
        S11.angle = theta
        time.sleep(0.5)
        S00.angle = theta
        S01.angle = theta
        time.sleep(0.5)
        S20.angle = theta
        S21.angle = theta
        time.sleep(0.5)
        B2.angle = theta
        time.sleep(0.5)
        B1.angle = theta
        time.sleep(2)

        # time.sleep(5)

        theta = 270

        B1.angle = theta
        time.sleep(0.5)
        B2.angle = theta
        time.sleep(0.5)
        S20.angle = theta
        S21.angle = theta
        time.sleep(0.5)
        S00.angle = theta
        S01.angle = theta
        time.sleep(0.5)
        S10.angle = theta
        S11.angle = theta
        time.sleep(0.5)
        S30.angle = theta
        S31.angle = theta
        time.sleep(0.5)
        B0.angle = theta
        time.sleep(2)

        # time.sleep(5)

if __name__ == '__main__':
    try:
        main()

    except (KeyboardInterrupt, SystemExit):
        pass
