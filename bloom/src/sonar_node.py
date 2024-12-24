#!/usr/bin/python3

import RPi.GPIO as GPIO
import rospy
from std_msgs.msg import Bool
from bloom.msg import animation
import time

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
    
class Sonar_node:
    def __init__(self):

        self.left_sonar = UltraSonic(17, 27)        # 17, 27
        self.right_sonar = UltraSonic(23, 24)       # 23, 24

        rospy.init_node('sonar_node')

        sub_topic = "keep_scanning"
        rospy.Subscriber(sub_topic, Bool , self.to_scan_or_not_to_scan)
        self.animation_msg = animation()
        self.pub = rospy.Publisher('animation', animation, queue_size=10)

        while not rospy.is_shutdown():
            rospy.spin()

        rospy.on_shutdown(self.shutdown)

    def to_scan_or_not_to_scan(self, msg):
        scan = msg.data
        if scan:
            self.get_animation()

    def get_animation(self):
        d_l = self.left_sonar.get_distance()
        d_r = self.right_sonar.get_distance()
        print("d_l:", d_l, "d_r:", d_r)

        if d_l < 100:
            self.animation_msg.animation_index = 0
            rospy.loginfo(self.animation_msg.animation_index)
            self.pub.publish(self.animation_msg)
            time.sleep(0.1)
        elif d_r < 100:
            self.animation_msg.animation_index = 1
            rospy.loginfo(self.animation_msg.animation_index)
            self.pub.publish(self.animation_msg)
            time.sleep(0.1)
        else:
            self.animation_msg.animation_index = 2
            rospy.loginfo(self.animation_msg.animation_index)
            self.pub.publish(self.animation_msg)
            time.sleep(0.1)
        
    def shutdown(self):
        GPIO.cleanup()
        return

if __name__ == '__main__':

    try:
        sonar = Sonar_node()

    except (rospy.ROSInterruptException, KeyboardInterrupt, SystemExit):
        pass