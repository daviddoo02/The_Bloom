#!/usr/bin/python3

import RPi.GPIO as GPIO
import time
import rospy
from rap.msg import distance
import argparse


class UltraSonic:
    def __init__(self, index, trigger, echo):
        # GPIO Mode (BOARD / BCM)
        GPIO.setmode(GPIO.BCM)

        # set GPIO Pins
        self.GPIO_TRIGGER = trigger
        self.GPIO_ECHO = echo
        self.index = index

        # set GPIO direction (IN / OUT)
        GPIO.setup(self.GPIO_TRIGGER, GPIO.OUT)
        GPIO.setup(self.GPIO_ECHO, GPIO.IN)

        pub = rospy.Publisher('distance', distance, queue_size=10)
        rospy.init_node('sonar_sensor_node' + str(self.index))
        rate = rospy.Rate(10) # hz
        self.protocol_obj = distance()

        while not rospy.is_shutdown():
            self.protocol_obj.distance[self.index] = self.get_distance()
            rospy.loginfo(self.protocol_obj.distance[self.index])
            pub.publish(self.protocol_obj)
            
            rate.sleep()

        rospy.on_shutdown(self.shut_down)

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


if __name__ == '__main__':
    try:
        parser = argparse.ArgumentParser(description='Sonar Sensor Node')
        parser.add_argument('--index', metavar='index', type=int, help='Index of sonar sensor')
        parser.add_argument('--trigger', metavar='trigger', type=int, help='Trigger pin in BCM')
        parser.add_argument('--echo', metavar='echo', type=int, help='Echo pin in BCM')
        args, unknown = parser.parse_known_args()

        us = UltraSonic(args.index, args.trigger, args.echo)

    except (rospy.ROSInterruptException, KeyboardInterrupt, SystemExit):
        GPIO.cleanup()