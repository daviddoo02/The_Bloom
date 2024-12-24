#!/usr/bin/env python3

__authors__ = "David Ho"

import rospy
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
        self.scan = True
        self.start = True
        self.count = 0

        rospy.init_node('rap_node')

        self.initialize_system()

        rospy.Timer(rospy.Duration(0.1), self.actuate)

        while not rospy.is_shutdown():
            rospy.spin()

        rospy.on_shutdown(self.shutdown)

    def initialize_system(self):
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
        self.w_b = 70

        # time id of each servo
        self.ids0 = 0
        self.ids1 = 0
        self.ids2 = 0
        self.ids3 = 0
        self.idb0 = 0
        self.idb1 = 0
        self.idb2 = 0

        # reset status
        self.reset_s0 = True
        self.reset_s1 = True
        self.reset_s2 = True
        self.reset_s3 = True
        self.reset_b0 = True
        self.reset_b1 = True
        self.reset_b2 = True
        
        # Initialize angles
        self.thb0 = 0
        self.ths3 = 0     # offset = np.pi/2
        self.ths1 = 0     # offset = np.pi/5
        self.ths0 = 0     # offset = np.pi/5
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
    
    def get_angle_S_type(self, init_angle, previous_angle, t, step_size, offset, count=0, reset=True):
        if t < 0:
            next_angle = previous_angle
        else:
            theta = 180/2 * np.sin(t/self.w_s + offset) + 180/2
            next_angle = self.smoothing_factor * theta + (1 - self.smoothing_factor) * previous_angle
        
        t += step_size
        count += 1
        
        if t >= (2*np.pi*self.w_s):
            t = 0
            # if reset:
                # t = 0
                # reset = False
            # else:
            #     theta = init_angle
            #     next_angle = self.smoothing_factor * theta + (1 - self.smoothing_factor) * previous_angle
        
        return round(next_angle, 1), t, count, reset

    def get_angle_B_type(self, init_angle, previous_angle, t, step_size, offset, count=0, reset=True):
        if t < 0:
            next_angle = previous_angle
        else:
            theta = 200/2 * np.cos(t/self.w_b + offset) + 200/2
            next_angle = self.smoothing_factor * theta + (1 - self.smoothing_factor) * previous_angle
        
        t += step_size
        count += 1
        
        if t >= (2*np.pi*self.w_b):
            t = 0
            # if reset:
            #     t = 0
            #     reset = False
            # else:
            #     theta = init_angle
            #     next_angle = self.smoothing_factor * theta + (1 - self.smoothing_factor) * previous_angle
        
        return round(next_angle, 1), t, count, reset

    def reset_all(self):
        # reset status
        self.reset_s0 = False
        self.reset_s1 = False
        self.reset_s2 = False
        self.reset_s3 = False
        self.reset_b0 = False
        self.reset_b1 = False
        self.reset_b2 = False
    
    def get_animation(self):
        d_l = self.left_sonar.get_distance()
        d_r = self.right_sonar.get_distance()
        print("d_l:", d_l, "d_r:", d_r)

        if d_l < 100:
            print("Animation: ", 0)
            return 0
        elif d_r < 100:
            print("Animation: ", 1)
            return 1
        else:
            print("Animation: ", 2)
            return 2
    
    def actuate(self, timer = 0):
        self.Default()

        # if self.scan:
        #     self.animation_id = self.get_animation()
        #     self.scan = False
        
        # if self.animation_id == 0:      # Left Bloom
        #     if self.start:
        #         n = round(np.pi*self.w_s / 3, 0)
        #         self.ids2 -= n * 0
        #         self.ids1 -= n * 1
        #         self.ids3 -= n * 1
        #         self.ids0 -= n * 2
        #         self.idb0 -= n * 3
        #         self.idb2 -= n * 3
        #         self.idb1 -= n * 4
        #         self.start = False
        #     self.Left_Bloom()
        # elif self.animation_id == 1:    # Right Bloom
        #     self.Right_Bloom(self.start)
        # elif self.animation_id == 2:    # Default
        #     self.Default(self.start)
        # else:
        #     self.Default()
    
    def Left_Bloom(self):               # Left bloom animation
        print("Left")
        # Delays
        self.S2_0.angle = self.ths2
        self.S2_1.angle = self.ths2
        self.ths2, self.ids2, _, self.reset_s2 = self.get_angle_S_type(self.S2_start_angle, self.ths2, t=self.ids2, step_size=self.ns2, offset=0, reset=self.reset_s2)

        self.S1_0.angle = self.ths1
        self.S1_1.angle = self.ths1
        self.ths1, self.ids1, _, self.reset_s1 = self.get_angle_S_type(self.S1_start_angle, self.ths1, t=self.ids1, step_size=self.ns1, offset=0, reset=self.reset_s1)

        self.S3_0.angle = self.ths3
        self.S3_1.angle = self.ths3
        self.ths3, self.ids3, _, self.reset_s3 = self.get_angle_S_type(self.S3_start_angle, self.ths3, t=self.ids3, step_size=self.ns3, offset=0, reset=self.reset_s3)

        self.S0_0.angle = self.ths0
        self.S0_1.angle = self.ths0
        self.ths0, self.ids0, _, self.reset_s0 = self.get_angle_S_type(self.S0_start_angle, self.ths0, t=self.ids0, step_size=self.ns0, offset=0, reset=self.reset_s0)

        self.B0.angle = self.thb0
        self.thb0, self.idb0, _, self.reset_b0 = self.get_angle_B_type(self.B0_start_angle, self.thb0, t=self.idb0, step_size=self.nb0, offset=0, reset=self.reset_b0)

        self.B1.angle = self.thb1
        self.thb1, self.idb1, _, self.reset_b1 = self.get_angle_B_type(self.B1_start_angle, self.thb1, t=self.idb1, step_size=self.nb1, offset=0, reset=self.reset_b1)

        self.B2.angle = self.thb2
        self.thb2, self.idb2, self.count_b2, self.reset_b2 = self.get_angle_B_type(self.B2_start_angle, self.thb2, t=self.idb2, step_size=self.nb2, offset=0, count=self.count, reset=self.reset_b2)

        print("idb2: ", self.idb2)

        print(self.count_b2)

        if self.idb2 > 1 * (2*np.pi*self.w_b):        # Determines how long each loop is
            self.scan = True
            self.start = True
            # self.count_b2 = 0
            self.reset_all_status()
            print("Exiting")

    def Right_Bloom(self, start):              # Right bloom animation
        print("Right")
        # Delays
        if start:
            n = round(np.pi*self.w_s / 4, 0)
            self.idb2 -= n * 0
            self.idb1 -= n * 1
            self.idb0 -= n * 2
            self.ids0 -= n * 3
            self.ids3 -= n * 3
            self.ids1 -= n * 4
            self.ids2 -= n * 5

        self.B2.angle = self.thb2
        self.thb2, self.idb2, _ = self.get_angle_B_type(self.thb2, t=self.idb2, step_size=self.nb2, offset=0)

        self.B1.angle = self.thb1
        self.thb1, self.idb1, _ = self.get_angle_B_type(self.thb1, t=self.idb1, step_size=self.nb1, offset=0)
        print('idb1:', self.idb1)

        self.B0.angle = self.thb0
        self.thb0, self.idb0, _ = self.get_angle_B_type(self.thb0, t=self.idb0, step_size=self.nb0, offset=0)

        self.S0_0.angle = self.ths0
        self.S0_1.angle = self.ths0
        self.ths0, self.ids0, _ = self.get_angle_S_type(self.ths0, t=self.ids0, step_size=self.ns0, offset=0)

        self.S3_0.angle = self.ths3
        self.S3_1.angle = self.ths3
        self.ths3, self.ids3, _ = self.get_angle_S_type(self.ths3, t=self.ids3, step_size=self.ns3, offset=0)

        self.S1_0.angle = self.ths1
        self.S1_1.angle = self.ths1
        self.ths1, self.ids1, _ = self.get_angle_S_type(self.ths1, t=self.ids1, step_size=self.ns1, offset=0)

        self.S2_0.angle = self.ths2
        self.S2_1.angle = self.ths2
        self.ths2, self.ids2, self.count = self.get_angle_S_type(self.ths2, t=self.ids2, step_size=self.ns2, offset=0, count=self.count)

        print(self.count)

        if self.count > 2 * (2*np.pi*self.w_s):        # Determines how long each loop is
            self.scan = True
            self.count = 0
            self.start = True
            # self.reset_servos()
            print("Exiting")
    
    def Default(self):                  # Default animation
        print("Default")
        self.S2_0.angle = self.ths2
        self.S2_1.angle = self.ths2
        self.ths2, self.ids2, _, self.reset_s2 = self.get_angle_S_type(self.S2_start_angle, self.ths2, t=self.ids2, step_size=self.ns2, offset=-np.pi/2, reset=self.reset_s2)

        self.S1_0.angle = self.ths1
        self.S1_1.angle = self.ths1
        self.ths1, self.ids1, _, self.reset_s1 = self.get_angle_S_type(self.S1_start_angle, self.ths1, t=self.ids1, step_size=self.ns1, offset=np.pi/5, reset=self.reset_s1)

        self.S3_0.angle = self.ths3
        self.S3_1.angle = self.ths3
        self.ths3, self.ids3, _, self.reset_s3 = self.get_angle_S_type(self.S3_start_angle, self.ths3, t=self.ids3, step_size=self.ns3, offset=np.pi/2, reset=self.reset_s3)

        self.S0_0.angle = self.ths0
        self.S0_1.angle = self.ths0
        self.ths0, self.ids0, _, self.reset_s0 = self.get_angle_S_type(self.S0_start_angle, self.ths0, t=self.ids0, step_size=self.ns0, offset=np.pi/5, reset=self.reset_s0)

        self.B0.angle = self.thb0
        self.thb0, self.idb0, _, self.reset_b0 = self.get_angle_B_type(self.B0_start_angle, self.thb0, t=self.idb0, step_size=self.nb0, offset=np.pi/3, reset=self.reset_b0)

        self.B1.angle = self.thb1
        self.thb1, self.idb1, _, self.reset_b1 = self.get_angle_B_type(self.B1_start_angle, self.thb1, t=self.idb1, step_size=self.nb1, offset=np.pi, reset=self.reset_b1)

        self.B2.angle = self.thb2
        self.thb2, self.idb2, self.count, self.reset_b2 = self.get_angle_B_type(self.B2_start_angle, self.thb2, t=self.idb2, step_size=self.nb2, offset=np.pi/2, count=self.count, reset=self.reset_b2)

        print(self.count)
        
        if self.count > (2 * np.pi*self.w_b):        # Determines how long each loop is
            self.scan = True
            self.count = 0
            self.start = True
            # self.reset_servos()
            print("Exiting")

    def reset_all_status(self):
        self.reset_s0 = True
        self.reset_s1 = True
        self.reset_s2 = True
        self.reset_s3 = True
        self.reset_b0 = True
        self.reset_b1 = True
        self.reset_b2 = True
    
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
        self.S2_start_angle = 0
        self.S1_start_angle = 0
        self.S0_start_angle = 0
        self.S3_start_angle = 0
        self.B0_start_angle = 0
        self.B1_start_angle = 0
        self.B2_start_angle = 0
        
        self.S2_0.angle = self.S2_start_angle
        self.S2_1.angle = self.S2_start_angle
        # time.sleep(1)

        self.S1_0.angle = self.S1_start_angle
        self.S1_1.angle = self.S1_start_angle
        # time.sleep(1)

        self.S0_0.angle = self.S0_start_angle
        self.S0_1.angle = self.S0_start_angle
        # time.sleep(1)

        self.S3_0.angle = self.S3_start_angle
        self.S3_1.angle = self.S3_start_angle
        # time.sleep(1)

        self.B0.angle = self.B0_start_angle
        # time.sleep(1)

        self.B1.angle = self.B1_start_angle
        # time.sleep(1)

        self.B2.angle = self.B2_start_angle
        # time.sleep(1)
        return

if __name__ == '__main__':
    try:
        demo_time = RAP()

    except (rospy.ROSInterruptException, KeyboardInterrupt, SystemExit):
        pass