#!/usr/bin/env python3

import rospy
from rap.msg import timing

def talker(protocol_obj):
    pub = rospy.Publisher('timing', timing, queue_size=10)
    rospy.init_node('protocol_node', anonymous=True)
    rate = rospy.Rate(3) # hz
    t_index = 0
    while not rospy.is_shutdown():
        protocol_obj.timing_index = t_index
        rospy.loginfo(protocol_obj)
        pub.publish(protocol_obj)
        
        t_index += 1

        if t_index > 100:
            t_index = 0
        
        rate.sleep()

if __name__ == '__main__':
    try:
        protocol_msg = timing()
        talker(protocol_msg)
    except rospy.ROSInterruptException:
        pass