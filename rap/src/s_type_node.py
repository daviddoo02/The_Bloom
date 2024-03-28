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
import csv


class S_Type:
    def __init__(self, csv_file, motor1, motor2) -> None:

        i2c = busio.I2C(SCL, SDA)
        self.pca = PCA9685(i2c)
        self.pca.frequency = 50

        self.smoothing_factor = 0.2
        self.current_angle = 0

        rospy.init_node('s_type_node')

        self.initialize_servos(motor1, motor2)

        self.angle_values = self.load_angle_values(csv_file)
        self.index = 0

        rospy.Timer(rospy.Duration(0.2), self.actuate)

        while not rospy.is_shutdown():
            rospy.spin()

    def load_angle_values(self, csv_file):
        # Load angle values from CSV file
        angle_values = []
        with open(csv_file, 'r') as csvfile:
            reader = csv.reader(csvfile)
            for row in reader:
                angle_values.append(float(row[0]))  # Assuming only one angle per row
        return angle_values

    def initialize_servos(self, motor1, motor2):

        # Declaring pin numbers of each servos on servo driver

        Servo1_index = motor1
        Servo2_index = motor2

        # Initializing each servos arcordingly

        self.Servo_1 = servo.Servo(
            self.pca.channels[Servo1_index], min_pulse=500, max_pulse=2600, actuation_range=270)
        self.Servo_2 = servo.Servo(
            self.pca.channels[Servo2_index], min_pulse=500, max_pulse=2600, actuation_range=270)
        
        self.Servo_1.angle = 0
        self.Servo_2.angle = 0
        time.sleep(1)
    
    def get_angle(self, t):
        theta = 145/2 * np.cos(t/10 + np.pi) + 145/2
        return theta
    
    def actuate(self, timer):
        theta = self.angle_values[self.index]
        
        self.current_angle = self.smoothing_factor * theta + (1 - self.smoothing_factor) * self.current_angle

        self.Servo_1.angle = self.current_angle
        self.Servo_2.angle = self.current_angle

        self.index += 1
        if self.index >= len(self.angle_values):
            self.index = 0

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='S Type Node')
    parser.add_argument('--motor1_index', metavar='motor1_index', type=int, help='Index of 1st motor in the s type unit')
    parser.add_argument('--motor2_index', metavar='motor2_index', type=int, help='Index of 2nd motor in the s type unit')
    args, unknown = parser.parse_known_args()

    try:
        csv_file = 'angles_S_type.csv'
        s_type = S_Type(csv_file, args.motor1_index, args.motor2_index)

    except (rospy.ROSInterruptException, KeyboardInterrupt, SystemExit):
        pass