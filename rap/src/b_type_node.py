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
import argparse


class S_Type:
    def __init__(self, motor1) -> None:

        i2c = busio.I2C(SCL, SDA)
        self.pca = PCA9685(i2c)
        self.pca.frequency = 50

        self.smoothing_factor = 0.2
        self.current_angle = 0

        rospy.init_node('B_type_node')

        self.initialize_servos(motor1)

        sub_topic = 'timing'

        self.sub = rospy.Subscriber(sub_topic, timing, self.actuate)

        while not rospy.is_shutdown():
            rospy.spin()

    def initialize_servos(self, motor1):

        # Declaring pin numbers of each servos on servo driver

        Servo1_index = motor1

        # Initializing each servos arcordingly

        self.Servo_1 = servo.Servo(
            self.pca.channels[Servo1_index], min_pulse=500, max_pulse=2600, actuation_range=270)
        
        self.Servo_1.angle = 0
        time.sleep(1)
    
    def get_angle(self, t):
        theta = 270/2 * np.cos(t/10 + np.pi) + 270/2
        return theta
    
    def actuate(self, msg):
        index = msg.timing_index

        theta = self.get_angle(index)
        
        self.current_angle = self.smoothing_factor * theta + (1 - self.smoothing_factor) * self.current_angle

        self.Servo_1.angle = self.current_angle

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='B Type Node')
    parser.add_argument('--motor1_index', metavar='motor1_index', type=int, help='Index of 1st motor in the b type unit')
    args, unknown = parser.parse_known_args()

    try:
        s_type = S_Type(args.motor1_index)

    except (rospy.ROSInterruptException, KeyboardInterrupt, SystemExit):
        pass
