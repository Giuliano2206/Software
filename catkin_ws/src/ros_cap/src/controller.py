#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Point
from duckietown_msgs.msg import  Twist2DStamped, BoolStamped

z=10000000000

def main():
    rospy.init_node('filtro_antiatropello')
    rospy.loginfo('filtro_antiatropello')
    # Subscriber for joint states
    sub = rospy.Subscriber('/duckiebot/wheels_driver_node/possible_cmd', Twist2DStamped, process_callback1)
    sub2 = rospy.Subscriber('/duckiebot/punto',Point, process_callback2)
    rospy.spin()

def process_callback2(msgc):
    global z    
    z=msgc.z
    

def process_callback1(msgf):
    global z
    print z
    base_pub1 = rospy.Publisher('/duckiebot/wheels_driver_node/car_cmd', Twist2DStamped, queue_size=1)
    msg=Twist2DStamped()
    msg.header.stamp = rospy.get_rostime()     
    #rospy.loginfo(msg)
    if z>=30:
        base_pub1.publish(msgf)
    else:
        msg.omega=msgf.omega        
        msg.v=0
        base_pub1.publish(msg)
if __name__ == '__main__':
    main()
