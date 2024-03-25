#!/usr/bin/python3

import RPi.GPIO as GPIO
import time


class UltraSonic:
    def __init__(self):
        # GPIO Mode (BOARD / BCM)
        GPIO.setmode(GPIO.BCM)

        # set GPIO Pins
        self.GPIO_TRIGGER = 4
        self.GPIO_ECHO = 17

        # set GPIO direction (IN / OUT)
        GPIO.setup(self.GPIO_TRIGGER, GPIO.OUT)
        GPIO.setup(self.GPIO_ECHO, GPIO.IN)

    def distance(self):
        # set Trigger to HIGH
        GPIO.output(self.GPIO_TRIGGER, True)

        # set Trigger after 0.01ms to LOW
        time.sleep(0.00001)
        GPIO.output(self.GPIO_TRIGGER, False)

        starttime = time.time()
        stoptime = time.time()

        # save StartTime
        while GPIO.input(self.GPIO_ECHO) == 0:
            starttime = time.time()

        # save time of arrival
        while GPIO.input(self.GPIO_ECHO) == 1:
            stoptime = time.time()

        # time difference between start and arrival
        timeelapsed = stoptime - starttime
        # multiply with the sonic speed (34300 cm/s)
        # and divide by 2, because there and back
        distance = (timeelapsed * 34300) / 2

        return distance


if __name__ == '__main__':
    try:
        us = UltraSonic()
        while True:
            dist = us.distance()
            print ("Measured Distance = %.1f cm" % dist)
            time.sleep(0.1)

    except KeyboardInterrupt:
        print("Measurement stopped by User")
        GPIO.cleanup()