#!/usr/bin/env python3

__authors__ = "David Ho"

import rospy
from rap.msg import timing
import time

import numpy as np

from board import SCL, SDA
import busio
from adafruit_motor import servo
from adafruit_pca9685 import PCA9685


class S_Type:
    def __init__(self) -> None:

        i2c = busio.I2C(SCL, SDA)
        self.pca = PCA9685(i2c)
        self.pca.frequency = 50

        rospy.init_node('s_type_node')

        self.initialize_servos()

        sub_topic = 'timing'

        self.sub = rospy.Subscriber(sub_topic, timing, self.actuate)

        while not rospy.is_shutdown():
            rospy.spin()

    def initialize_servos(self):

        # Declaring pin numbers of each servos on servo driver

        Servo1 = 0
        Servo2 = 1

        # Initializing each servos arcordingly

        self.Servo_1 = servo.Servo(
            self.pca.channels[Servo1], min_pulse=500, max_pulse=2600, actuation_range=270)
        self.Servo_2 = servo.Servo(
            self.pca.channels[Servo2], min_pulse=500, max_pulse=2600, actuation_range=270)
        
        self.Servo_1.angle = 0
        self.Servo_2.angle = 0
        time.sleep(2)
    
    def timing_callback(self, msg):
        return msg.timing_index[0]
    
    def actuate(self, msg):
        theta1 = 270
        theta2 = 270

        self.Servo_1.angle = theta1
        self.Servo_2.angle = theta2
        time.sleep(2)

        self.Servo_1.angle = 0
        self.Servo_2.angle = 0
        time.sleep(2)

if __name__ == '__main__':
    try:
        s_type = S_Type()

    except (rospy.ROSInterruptException, KeyboardInterrupt, SystemExit):
        pass
