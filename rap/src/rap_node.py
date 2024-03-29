#!/usr/bin/env python3

__authors__ = "David Ho"

import rospy
from rap.msg import distance
import time

import numpy as np

from board import SCL, SDA
import busio
from adafruit_motor import servo
from adafruit_pca9685 import PCA9685
import argparse
import csv


class RAP:
    def __init__(self) -> None:

        i2c = busio.I2C(SCL, SDA)
        self.pca = PCA9685(i2c)
        self.pca.frequency = 50

        self.smoothing_factor = 0.2
        self.current_angle = 0

        rospy.init_node('rap_node')

        self.initialize_servos()

        s_type_csv = 'csv/angles_S_type.csv'
        b_type_csv = 'csv/angles_B_type.csv'

        self.s_angles = self.load_angle_values(s_type_csv)
        self.b_angles = self.load_angle_values(b_type_csv)
        self.index = 0

        rospy.Timer(rospy.Duration(0.2), self.actuate)

        rospy.spin()

        rospy.on_shutdown(self.shut_down)

    def load_angle_values(self, csv_file):
        # Load angle values from CSV file
        angle_values = []
        with open(csv_file, 'r') as csvfile:
            reader = csv.reader(csvfile)
            for row in reader:
                angle_values.append(float(row[0]))  # Assuming only one angle per row
        return angle_values

    def initialize_servos(self):
        # Initializing each servos arcordingly

        # S0
        self.S0_0 = servo.Servo(
            self.pca.channels[0], min_pulse=500, max_pulse=2600, actuation_range=270)
        self.S0_1 = servo.Servo(
            self.pca.channels[1], min_pulse=500, max_pulse=2600, actuation_range=270)

        # S1
        self.S1_0 = servo.Servo(
            self.pca.channels[2], min_pulse=500, max_pulse=2600, actuation_range=270)
        self.S1_1 = servo.Servo(
            self.pca.channels[3], min_pulse=500, max_pulse=2600, actuation_range=270)
        
        # S2
        self.S2_0 = servo.Servo(
            self.pca.channels[4], min_pulse=500, max_pulse=2600, actuation_range=270)
        self.S2_1 = servo.Servo(
            self.pca.channels[5], min_pulse=500, max_pulse=2600, actuation_range=270)
        
        # S3
        self.S3_0 = servo.Servo(
            self.pca.channels[6], min_pulse=500, max_pulse=2600, actuation_range=270)
        self.S3_1 = servo.Servo(
            self.pca.channels[7], min_pulse=500, max_pulse=2600, actuation_range=270)
        
        # B0
        self.B0 = servo.Servo(
            self.pca.channels[8], min_pulse=500, max_pulse=2600, actuation_range=270)
        
        # B1
        self.B1 = servo.Servo(
            self.pca.channels[9], min_pulse=500, max_pulse=2600, actuation_range=270)
        
        # B2
        self.B2 = servo.Servo(
            self.pca.channels[10], min_pulse=500, max_pulse=2600, actuation_range=270)

        self.Servo_1.angle = 0
        self.Servo_2.angle = 0
        time.sleep(1)

    def get_next_angle(self, previous_angle, angle_list, index, multiplier):
        """
        Function:
            Get next angle and index for servo
        
        """
        theta = angle_list[index]

        next_angle = self.smoothing_factor * theta + (1 - self.smoothing_factor) * previous_angle

        index += 1 * multiplier
        if index >= len(angle_list):
            index = 0
        
        return next_angle, theta
    
    def actuate(self, timer):
        # theta = self.angle_values[self.index]

        # self.current_angle = self.smoothing_factor * theta + (1 - self.smoothing_factor) * self.current_angle

        self.Servo_1.angle = self.current_angle
        self.Servo_2.angle = self.current_angle

        # self.index += 1
        # if self.index >= len(self.angle_values):
        #     self.index = 0

    def shutdown(self):
        return

if __name__ == '__main__':
    try:
        demo_time = RAP()

    except (rospy.ROSInterruptException, KeyboardInterrupt, SystemExit):
        pass