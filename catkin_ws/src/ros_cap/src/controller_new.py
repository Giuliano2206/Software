#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Point
from duckietown_msgs.msg import  Twist2DStamped, BoolStamped

data = {}


def main():
    rospy.init_node('controller')
    rospy.loginfo('controller')
    sub = rospy.Subscriber('/duckiebot/wheels_driver_node/possible_cmd', Twist2DStamped, process_callback_1)
    sub_2 = rospy.Subscriber('/duckiebot/street_data', dict, process_callback_2)
    rospy.spin()


def process_callback_2(msg_2):
    global data
    data = msg_2
    

def process_callback_1(msg_1):
    global data
    print(data)
    base_pub1 = rospy.Publisher('/duckiebot/wheels_driver_node/car_cmd', Twist2DStamped, queue_size=1)
    base_pub1.publish(msg_1)

if __name__ == '__main__':
    main()
