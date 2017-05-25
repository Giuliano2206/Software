#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy
from duckietown_msgs.msg import  Twist2DStamped, BoolStamped

def main():
    rospy.init_node('test_subscriber')
    rospy.loginfo('test_subscriber')
    # Subscriber for joint states
    sub = rospy.Subscriber('/duckiebot/wheels_driver_node/possible_cmd', Twist2DStamped, process_callback1)
    sub2 = rospy.Subscriber('/duckiebot/punto',Point, process_callback2)
	rospy.spin()

def process_callback1(msg):
    base_pub = rospy.Publisher('/duckiebot/wheels_driver_node/car_cmd', Twist2DStamped, queue_size=1)
    msga=Twist2DStamped()
    msga.header.stamp = rospy.get_rostime()     
    rospy.loginfo(msg)
    if 
   # if msg.buttons[1]==0:       
    #    msga.omega=9*(msg.axes[0])        
     #   msga.v=msg.axes[1]
   # else:
    #    msga.v=0
     #   msga.v=0    
    base_pub.publish(msga)
   
if __name__ == '__main__':
    main()
