#!/usr/bin/env python3

__authors__ = "David Ho"

import rospy
from rap.msg import distance, timing
import time

import numpy as np

from board import SCL, SDA
import busio
from adafruit_motor import servo
from adafruit_pca9685 import PCA9685
import csv


class RAP:
    def __init__(self) -> None:

        i2c = busio.I2C(SCL, SDA)
        self.pca = PCA9685(i2c)
        self.pca.frequency = 50

        self.smoothing_factor = 0.05

        rospy.init_node('rap_node')

        self.initialize_servos()

        # s_type_csv = 'csv/angles_S_type.csv'
        # b_type_csv = 'csv/angles_B_type.csv'

        # self.s_angles = self.load_angle_values(s_type_csv)
        # self.b_angles = self.load_angle_values(b_type_csv)

        self.ids0 = 0
        self.ids1 = 0
        self.ids2 = 0
        self.ids3 = 0
        self.idb0 = 0
        self.idb1 = 0
        self.idb2 = 0

        self.thb0 = 0

        self.ths3 = 145     # offset = np.pi/2
        self.ths1 = 115     # offset = np.pi/5
        self.ths0 = 115     # offset = np.pi/5
        self.ths2 = 0       # offset = -np.pi/2

        self.thb2 = 0
        self.thb1 = 0

        # Control speed of each panel --> connect to sonar sensor later
        
        n = 5

        self.nb0 = n
        self.ns3 = n
        self.ns1 = n
        self.ns0 = n
        self.ns2 = n        
        self.nb2 = n
        self.nb1 = n

        rospy.Timer(rospy.Duration(0.3), self.actuate)
        # sub_topic = 'timing'

        # self.move_flowers = rospy.Subscriber(sub_topic, timing, self.actuate)

        while not rospy.is_shutdown():
            rospy.spin()

        rospy.on_shutdown(self.shutdown)

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
        
        time.sleep(1)

        self.shutdown()

        return

    def get_next_angle(self, previous_angle, angle_list, index, multiplier):
        """
        Function:
            Get next angle and index for servo
        
        """
        theta = angle_list[index]

        # next_angle = self.smoothing_factor * theta + (1 - self.smoothing_factor) * previous_angle
        next_angle = theta

        index += 1 * multiplier
        if index >= len(angle_list):
            index = 0
        
        return next_angle, index
    
    def get_angle_S_type(self, previous_angle, t, step_size, offset):
        w = 50                       
        theta = 200/2 * np.sin(t/w + offset) + 200/2
        next_angle = self.smoothing_factor * theta + (1 - self.smoothing_factor) * previous_angle
        t += step_size
        if t >= (2*np.pi*w):
            t = 0
        return round(next_angle, 1), t

    def get_angle_B_type(self, previous_angle, t, step_size):
        w = 100                      
        theta = 200/2 * np.cos(t/w + np.pi) + 200/2
        next_angle = self.smoothing_factor * theta + (1 - self.smoothing_factor) * previous_angle
        t += step_size
        if t >= (2*np.pi*w):
            t = 0
        return round(next_angle, 1), t
    
    def actuate(self, msg):
        """
        self.ths0, self.ids0
        self.ths1, self.ids1
        self.ths2, self.ids2
        self.ths3, self.ids3
        self.thb0, self.idb0
        self.thb1, self.idb1
        self.thb2, self.idb2

        self.ns0
        self.ns1
        self.ns2
        self.ns3
        self.nb0
        self.nb1
        self.nb2
        """

        self.B0.angle = self.thb0
        # time.sleep(0.1)

        self.S3_0.angle = self.ths3
        self.S3_1.angle = self.ths3
        # time.sleep(0.1)

        self.S1_0.angle = self.ths1
        self.S1_1.angle = self.ths1
        # time.sleep(0.1)
        
        self.S0_0.angle = self.ths0
        self.S0_1.angle = self.ths0
        # time.sleep(0.1)

        self.S2_0.angle = self.ths2
        self.S2_1.angle = self.ths2
        # time.sleep(0.1)
        
        self.B2.angle = self.thb2
        # time.sleep(0.1)

        self.B1.angle = self.thb1
        # time.sleep(0.1)

        # self.ths0, self.ids0 = self.get_next_angle(self.ths0, self.s_angles, self.ids0, self.ns0)
        # self.ths1, self.ids1 = self.get_next_angle(self.ths1, self.s_angles, self.ids1, self.ns1)
        # self.ths2, self.ids2 = self.get_next_angle(self.ths2, self.s_angles, self.ids2, self.ns2)
        # self.ths3, self.ids3 = self.get_next_angle(self.ths3, self.s_angles, self.ids3, self.ns3)
        # self.thb0, self.idb0 = self.get_next_angle(self.thb0, self.b_angles, self.idb0, self.nb0)
        # self.thb1, self.idb1 = self.get_next_angle(self.thb1, self.b_angles, self.idb1, self.nb1)
        # self.thb2, self.idb2 = self.get_next_angle(self.thb2, self.b_angles, self.idb2, self.nb2)
        
        self.ths0, self.ids0 = self.get_angle_S_type(self.ths0, t=self.ids0, step_size=self.ns0, offset=np.pi/5)
        self.ths1, self.ids1 = self.get_angle_S_type(self.ths1, t=self.ids1, step_size=self.ns1, offset=np.pi/5)
        self.ths2, self.ids2 = self.get_angle_S_type(self.ths2, t=self.ids2, step_size=self.ns2, offset=-np.pi/2)
        self.ths3, self.ids3 = self.get_angle_S_type(self.ths3, t=self.ids3, step_size=self.ns3, offset=np.pi/2)
        self.thb0, self.idb0 = self.get_angle_B_type(self.thb0, t=self.idb0, step_size=self.nb0)
        self.thb1, self.idb1 = self.get_angle_B_type(self.thb1, t=self.idb1, step_size=self.nb1)
        self.thb2, self.idb2 = self.get_angle_B_type(self.thb2, t=self.idb2, step_size=self.nb2)

        return
    
    def shutdown(self):
        """
        self.S0_0
        self.S0_1
        self.S1_0
        self.S1_1
        self.S2_0
        self.S2_1
        self.S3_0
        self.S3_1
        self.B0
        self.B1
        self.B2
        """
        self.B0.angle = 0
        time.sleep(1)

        self.S3_0.angle = 145
        self.S3_1.angle = 145
        time.sleep(1)

        self.S1_0.angle = 145
        self.S1_1.angle = 145
        time.sleep(1)
        
        self.S0_0.angle = 145
        self.S0_1.angle = 145
        time.sleep(1)

        self.S2_0.angle = 145
        self.S2_1.angle = 145
        time.sleep(1)
        
        self.B2.angle = 0
        time.sleep(1)

        self.B1.angle = 0
        time.sleep(1)

        return

if __name__ == '__main__':
    try:
        demo_time = RAP()

    except (rospy.ROSInterruptException, KeyboardInterrupt, SystemExit):
        pass