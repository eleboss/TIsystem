#!/usr/bin/python2.7

from __future__ import division
import rospy 
import roslib
import time
import numpy as np
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped
from mavros_msgs.msg import PositionTarget, AttitudeTarget, State
from geometry_msgs.msg import TransformStamped, PoseStamped, Point, PointStamped, Vector3, Vector3Stamped, TwistStamped, QuaternionStamped
from tf.transformations import quaternion_from_euler, euler_from_quaternion

counter = 500000 #50hz 8*60*60*50
r = 8
theta = 0
set_position = PoseStamped()
set_position.pose.position.x = 0
set_position.pose.position.y = 0
set_position.pose.position.z = 1
set_yaw = 0
wn = 0.03

def callback_odom(pose):
    global r, counter,theta, set_position,set_yaw,wn
    
    print 'looping:', counter
    if counter < 500000 and counter > 450000:
        
        if counter % 200 == 0:
            set_position.x = 0
            set_position.y = 0
            set_position.z = 1
            set_yaw = 0
        print "random fly ",'X:',set_position.x,'Y:',set_position.y,'Z:',set_position.z
        stamp = rospy.get_rostime()
        set_position.header.stamp = stamp
        position_pub.publish(set_position)


    if counter == 0:
        print 'Flying test over hold still at 1 1 1'
        set_position.z = 0

        stamp = rospy.get_rostime()
        set_position.header.stamp = stamp
        position_pub.publish(set_position)

    counter = counter -1

rospy.init_node('fly_control')

subodom = rospy.Subscriber('/mavros/local_position/odom', Odometry, callback_odom)
position_pub = rospy.Publisher('/setpoint_PID', PoseStamped, queue_size=1)

rospy.spin()