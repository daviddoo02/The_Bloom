#!/usr/bin/env python3

__authors__ = "David Ho"

import rospy
from rap.msg import animation
from std_msgs.msg import Bool
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
        self.animation_id = 2

        rospy.init_node('rap_node')

        self.continue_scan = rospy.Publisher('keep_scanning', Bool, queue_size=10)

        self.start = True

        self.initialize_servos()

        rospy.Subscriber('animation', animation, self.get_animation_index)

        while not rospy.is_shutdown():
            if self.start:
                self.continue_scan.publish(True)
                print("Start")
            rospy.spin()

        rospy.on_shutdown(self.shutdown)

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

        # Periods of each function, small = faster
        self.w_s = 20     
        self.w_b = 45

        # time id of each servo
        self.ids0 = 0
        self.ids1 = 0
        self.ids2 = 0
        self.ids3 = 0
        self.idb0 = 0
        self.idb1 = 0
        self.idb2 = 0
        
        # Initialize angles
        self.thb0 = 0
        self.ths3 = 145     # offset = np.pi/2
        self.ths1 = 115     # offset = np.pi/5
        self.ths0 = 115     # offset = np.pi/5
        self.ths2 = 0       # offset = -np.pi/2
        self.thb2 = 0
        self.thb1 = 0

        # Control resolution - smaller = finer        
        n = 1
        self.nb0 = n
        self.ns3 = n
        self.ns1 = n
        self.ns0 = n
        self.ns2 = n        
        self.nb2 = n
        self.nb1 = n

        self.shutdown()

        return
    
    def get_angle_S_type(self, previous_angle, t, step_size, offset):                  
        theta = 180/2 * np.sin(t/self.w_s + offset) + 180/2
        next_angle = self.smoothing_factor * theta + (1 - self.smoothing_factor) * previous_angle
        t += step_size
        # if t >= (2*np.pi*self.w_s):
        #     t = 0
        return round(next_angle, 1), t

    def get_angle_B_type(self, previous_angle, t, step_size, offset):
        theta = 200/2 * np.cos(t/self.w_b + offset) + 200/2
        next_angle = self.smoothing_factor * theta + (1 - self.smoothing_factor) * previous_angle
        t += step_size
        # if t >= (2*np.pi*self.w_b):
        #     t = 0
        return round(next_angle, 1), t
    
    def get_animation_index(self, msg):
        self.animation_id = msg.animation_index
        print(self.animation_id)
        self.continue_scan.publish(False)
        self.start = False

        self.loop = True

        if self.loop:
            # print("Entering loop")
            # rospy.Timer(rospy.Duration(0.1), self.actuate)
            self.actuate()
        
        self.continue_scan.publish(True)
    
    def actuate(self, timer = 0):
        """
        """
        print("Actuating")
        
        if self.animation_id == 0:   # Left Bloom
            self.Left_Bloom()
        if self.animation_id == 1:   # Right Bloom
            self.Right_Bloom()
        if self.animation_id == 2:   # Default
            self.Default()
        else:
            self.loop = False
    
    def Left_Bloom(self):              # Left bloom animation
        print("Left")
        self.S2_0.angle = self.ths2
        self.S2_1.angle = self.ths2
        self.ths2, self.ids2 = self.get_angle_S_type(self.ths2, t=self.ids2, step_size=self.ns2, offset=0)

        if self.ids2 > (np.pi*self.w_s):
            self.S1_0.angle = self.ths1
            self.S1_1.angle = self.ths1
            self.ths1, self.ids1 = self.get_angle_S_type(self.ths1, t=self.ids1, step_size=self.ns1, offset=0)

        if self.ids1 > (np.pi*self.w_s):
            self.S0_0.angle = self.ths0
            self.S0_1.angle = self.ths0
            self.ths0, self.ids0 = self.get_angle_S_type(self.ths0, t=self.ids0, step_size=self.ns0, offset=0)

        if self.ids0 > (np.pi*self.w_s):
            self.S3_0.angle = self.ths3
            self.S3_1.angle = self.ths3
            self.ths3, self.ids3 = self.get_angle_S_type(self.ths3, t=self.ids3, step_size=self.ns3, offset=0)

        if self.ids3 > (np.pi*self.w_s):
            self.B0.angle = self.thb0
            self.thb0, self.idb0 = self.get_angle_B_type(self.thb0, t=self.idb0, step_size=self.nb0, offset=0)

        if self.idb0 > (np.pi*self.w_b):
            self.B1.angle = self.thb1
            self.thb1, self.idb1 = self.get_angle_B_type(self.thb1, t=self.idb1, step_size=self.nb1, offset=0)

        if self.idb1 > (np.pi*self.w_b):
            self.B2.angle = self.thb2
            self.thb2, self.idb2 = self.get_angle_B_type(self.thb2, t=self.idb2, step_size=self.nb2, offset=0)

        if self.idb2 > 4 * (np.pi*self.w_b):        # Determines how long each loop is
            self.loop = False


    def Right_Bloom(self):              # Right bloom animation
        print("Right")
        self.B2.angle = self.thb2
        self.thb2, self.idb2 = self.get_angle_B_type(self.thb2, t=self.idb2, step_size=self.nb2, offset=0)

        if self.idb2 > (np.pi*self.w_b):
            self.B1.angle = self.thb1
            self.thb1, self.idb1 = self.get_angle_B_type(self.thb1, t=self.idb1, step_size=self.nb1, offset=0)

        if self.idb1 > (np.pi*self.w_b):
            self.B0.angle = self.thb0
            self.thb0, self.idb0 = self.get_angle_B_type(self.thb0, t=self.idb0, step_size=self.nb0, offset=0)

        if self.idb0 > (np.pi*self.w_b):
            self.S3_0.angle = self.ths3
            self.S3_1.angle = self.ths3
            self.ths3, self.ids3 = self.get_angle_S_type(self.ths3, t=self.ids3, step_size=self.ns3, offset=0)

        if self.ids3 > (np.pi*self.w_s):
            self.S0_0.angle = self.ths0
            self.S0_1.angle = self.ths0
            self.ths0, self.ids0 = self.get_angle_S_type(self.ths0, t=self.ids0, step_size=self.ns0, offset=0)

        if self.ids0 > (np.pi*self.w_s):
            self.S1_0.angle = self.ths1
            self.S1_1.angle = self.ths1
            self.ths1, self.ids1 = self.get_angle_S_type(self.ths1, t=self.ids1, step_size=self.ns1, offset=0)
        
        if self.ids1 > (np.pi*self.w_s):
            self.S2_0.angle = self.ths2
            self.S2_1.angle = self.ths2
            self.ths2, self.ids2 = self.get_angle_S_type(self.ths2, t=self.ids2, step_size=self.ns2, offset=0)

        if self.ids2 > 4 * (np.pi*self.w_s):        # Determines how long each loop is
            self.loop = False
    
    def Default(self):                  # Default animation
        print("Default")
        self.S2_0.angle = self.ths2
        self.S2_1.angle = self.ths2
        self.ths2, self.ids2 = self.get_angle_S_type(self.ths2, t=self.ids2, step_size=self.ns2, offset=0)

        self.S1_0.angle = self.ths1
        self.S1_1.angle = self.ths1
        self.ths1, self.ids1 = self.get_angle_S_type(self.ths1, t=self.ids1, step_size=self.ns1, offset=0)

        self.S0_0.angle = self.ths0
        self.S0_1.angle = self.ths0
        self.ths0, self.ids0 = self.get_angle_S_type(self.ths0, t=self.ids0, step_size=self.ns0, offset=0)

        self.S3_0.angle = self.ths3
        self.S3_1.angle = self.ths3
        self.ths3, self.ids3 = self.get_angle_S_type(self.ths3, t=self.ids3, step_size=self.ns3, offset=0)

        self.B0.angle = self.thb0
        self.thb0, self.idb0 = self.get_angle_B_type(self.thb0, t=self.idb0, step_size=self.nb0, offset=0)

        self.B1.angle = self.thb1
        self.thb1, self.idb1 = self.get_angle_B_type(self.thb1, t=self.idb1, step_size=self.nb1, offset=0)

        self.B2.angle = self.thb2
        self.thb2, self.idb2 = self.get_angle_B_type(self.thb2, t=self.idb2, step_size=self.nb2, offset=0)

        print(self.idb2, '    ', self.thb2)

        if self.idb2 > 4 * (np.pi*self.w_b):        # Determines how long each loop is
            self.loop = False
            print("Exiting")
    
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
        self.S2_0.angle = 145
        self.S2_1.angle = 145
        time.sleep(1)

        self.S1_0.angle = 145
        self.S1_1.angle = 145
        time.sleep(1)

        self.S0_0.angle = 145
        self.S0_1.angle = 145
        time.sleep(1)

        self.S3_0.angle = 145
        self.S3_1.angle = 145
        time.sleep(1)

        self.B0.angle = 0
        time.sleep(1)

        self.B1.angle = 0
        time.sleep(1)

        self.B2.angle = 0
        time.sleep(1)
        return

if __name__ == '__main__':
    try:
        demo_time = RAP()

    except (rospy.ROSInterruptException, KeyboardInterrupt, SystemExit):
        pass