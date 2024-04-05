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

class UltraSonic:
    def __init__(self, trigger, echo):
        # GPIO Mode (BOARD / BCM)
        GPIO.setmode(GPIO.BCM)

        # set GPIO Pins
        self.GPIO_TRIGGER = trigger
        self.GPIO_ECHO = echo

        # set GPIO direction (IN / OUT)
        GPIO.setup(self.GPIO_TRIGGER, GPIO.OUT)
        GPIO.setup(self.GPIO_ECHO, GPIO.IN)

    def get_distance(self):
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
    
    def shut_down(self):
        GPIO.cleanup()
        return

class RAP:
    def __init__(self) -> None:

        i2c = busio.I2C(SCL, SDA)
        self.pca = PCA9685(i2c)
        self.pca.frequency = 50

        self.smoothing_factor = 0.05
        self.animation_id = 2
        self.scan = True
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
        
        self.left_sonar = UltraSonic(17, 27)        # 17, 27
        self.right_sonar = UltraSonic(23, 24)       # 23, 24
        
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
    
    def get_angle_S_type(self, previous_angle, t, step_size, offset, count):
        if t < 0:
            next_angle = previous_angle
        else:
            theta = 180/2 * np.sin(t/self.w_s + offset) + 180/2
            next_angle = self.smoothing_factor * theta + (1 - self.smoothing_factor) * previous_angle
        
        t += step_size
        count += 1
        
        if t >= (2*np.pi*self.w_s):
            t = 0
        
        return round(next_angle, 1), t, count

    def get_angle_B_type(self, previous_angle, t, step_size, offset, count):
        if t < 0:
            next_angle = previous_angle
        else:
            theta = 200/2 * np.cos(t/self.w_b + offset) + 200/2
            next_angle = self.smoothing_factor * theta + (1 - self.smoothing_factor) * previous_angle
        
        t += step_size
        count += 1
        
        if t >= (2*np.pi*self.w_b):
            t = 0
        
        return round(next_angle, 1), t, count

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
        if self.scan:
            self.animation_id = self.get_animation()
            self.scan = False
        
        if self.animation_id == 0:      # Left Bloom
            self.Left_Bloom()
        elif self.animation_id == 1:    # Right Bloom
            self.Right_Bloom()
        elif self.animation_id == 2:    # Default
            self.Default()
        else:
            self.Default()
    
    def Left_Bloom(self):               # Left bloom animation
        print("Left")
        # Delays
        n = round(np.pi*self.w_b, 0)
        self.ids2 -= n * 0
        self.ids1 -= n * 1
        self.ids0 -= n * 2
        self.ids3 -= n * 3
        self.idb0 -= n * 4
        self.idb1 -= n * 5
        self.idb2 -= n * 6

        self.S2_0.angle = self.ths2
        self.S2_1.angle = self.ths2
        self.ths2, self.ids2, _ = self.get_angle_S_type(self.ths2, t=self.ids2, step_size=self.ns2, offset=0)

        self.S1_0.angle = self.ths1
        self.S1_1.angle = self.ths1
        self.ths1, self.ids1, _ = self.get_angle_S_type(self.ths1, t=self.ids1, step_size=self.ns1, offset=0)

        self.S0_0.angle = self.ths0
        self.S0_1.angle = self.ths0
        self.ths0, self.ids0, _ = self.get_angle_S_type(self.ths0, t=self.ids0, step_size=self.ns0, offset=0)

        self.S3_0.angle = self.ths3
        self.S3_1.angle = self.ths3
        self.ths3, self.ids3, _ = self.get_angle_S_type(self.ths3, t=self.ids3, step_size=self.ns3, offset=0)

        self.B0.angle = self.thb0
        self.thb0, self.idb0, _ = self.get_angle_B_type(self.thb0, t=self.idb0, step_size=self.nb0, offset=0)

        self.B1.angle = self.thb1
        self.thb1, self.idb1, _ = self.get_angle_B_type(self.thb1, t=self.idb1, step_size=self.nb1, offset=0)

        self.B2.angle = self.thb2
        self.thb2, self.idb2, self.count = self.get_angle_B_type(self.thb2, t=self.idb2, step_size=self.nb2, offset=0)

        if self.count > 1 * (2*np.pi*self.w_b):        # Determines how long each loop is
            self.scan = True
            self.count = 0
            print("Exiting")

    def Right_Bloom(self):              # Right bloom animation
        print("Right")
        # Delays
        n = round(np.pi*self.w_s, 0)
        self.idb2 -= n * 0
        self.idb1 -= n * 1
        self.idb0 -= n * 2
        self.ids3 -= n * 3
        self.ids0 -= n * 4
        self.ids1 -= n * 5
        self.ids2 -= n * 6

        self.B2.angle = self.thb2
        self.thb2, self.idb2, _ = self.get_angle_B_type(self.thb2, t=self.idb2, step_size=self.nb2, offset=0)

        self.B1.angle = self.thb1
        self.thb1, self.idb1, _ = self.get_angle_B_type(self.thb1, t=self.idb1, step_size=self.nb1, offset=0)

        self.B0.angle = self.thb0
        self.thb0, self.idb0, _ = self.get_angle_B_type(self.thb0, t=self.idb0, step_size=self.nb0, offset=0)

        self.S3_0.angle = self.ths3
        self.S3_1.angle = self.ths3
        self.ths3, self.ids3, _ = self.get_angle_S_type(self.ths3, t=self.ids3, step_size=self.ns3, offset=0)

        self.S0_0.angle = self.ths0
        self.S0_1.angle = self.ths0
        self.ths0, self.ids0, _ = self.get_angle_S_type(self.ths0, t=self.ids0, step_size=self.ns0, offset=0)

        self.S1_0.angle = self.ths1
        self.S1_1.angle = self.ths1
        self.ths1, self.ids1, _ = self.get_angle_S_type(self.ths1, t=self.ids1, step_size=self.ns1, offset=0)

        self.S2_0.angle = self.ths2
        self.S2_1.angle = self.ths2
        self.ths2, self.ids2, self.count = self.get_angle_S_type(self.ths2, t=self.ids2, step_size=self.ns2, offset=0)

        if self.count > 1 * (2*np.pi*self.w_s):        # Determines how long each loop is
            self.scan = True
            self.count = 0
            print("Exiting")
    
    def Default(self):                  # Default animation
        print("Default")
        self.S2_0.angle = self.ths2
        self.S2_1.angle = self.ths2
        self.ths2, self.ids2, _ = self.get_angle_S_type(self.ths2, t=self.ids2, step_size=self.ns2, offset=-np.pi/2)

        self.S1_0.angle = self.ths1
        self.S1_1.angle = self.ths1
        self.ths1, self.ids1, _ = self.get_angle_S_type(self.ths1, t=self.ids1, step_size=self.ns1, offset=np.pi/5)

        self.S0_0.angle = self.ths0
        self.S0_1.angle = self.ths0
        self.ths0, self.ids0, _ = self.get_angle_S_type(self.ths0, t=self.ids0, step_size=self.ns0, offset=np.pi/5)

        self.S3_0.angle = self.ths3
        self.S3_1.angle = self.ths3
        self.ths3, self.ids3, _ = self.get_angle_S_type(self.ths3, t=self.ids3, step_size=self.ns3, offset=np.pi/2)

        self.B0.angle = self.thb0
        self.thb0, self.idb0, _ = self.get_angle_B_type(self.thb0, t=self.idb0, step_size=self.nb0, offset=0)

        self.B1.angle = self.thb1
        self.thb1, self.idb1, _ = self.get_angle_B_type(self.thb1, t=self.idb1, step_size=self.nb1, offset=np.pi)

        self.B2.angle = self.thb2
        self.thb2, self.idb2, self.count = self.get_angle_B_type(self.thb2, t=self.idb2, step_size=self.nb2, offset=np.pi/2)

        if self.count > 1 * (2 * np.pi*self.w_b):        # Determines how long each loop is
            self.scan = True
            self.count = 0
            print("Exiting")

    def reset_id(self):
        # time id of each servo
        self.ids0 = 0
        self.ids1 = 0
        self.ids2 = 0
        self.ids3 = 0
        self.idb0 = 0
        self.idb1 = 0
        self.idb2 = 0
    
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